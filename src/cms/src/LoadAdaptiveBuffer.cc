// SPDX-License-Identifier: BSD-3-Clause
//
// Load-adaptive mesh buffer sizing — mirrors clocksyn's grid_t::buffer_grid()
// (clocksyn/src/grid.C:186) in `buffer_entire_grid = true` mode:
//
//   For each grid intersection:
//     load_i = local_wire_cap_i + sum(sink_pin_caps assigned to i)
//     buffer_master_i = smallest master with max_capacitance >= load_i
//
// Unlike clocksyn we read max_capacitance directly from Liberty rather than
// computing buf_fixed_gain * size * gatecappersize from a tech file. The
// candidate library is whatever buffer masters the user passes in.

#include "cms/LoadAdaptiveBuffer.hh"

#include <algorithm>
#include <cctype>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "db_sta/dbNetwork.hh"
#include "odb/db.h"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "utl/Logger.h"

namespace cms {

using utl::CMS;

namespace {

struct MasterEntry {
  std::string name;
  odb::dbMaster* master;
  float max_load_pF;     // Liberty max_capacitance on the output pin
  float input_cap_pF;    // Liberty input-pin cap (informational only)
  int   drive_strength;  // parsed "xN" from master name (0 if unknown)
  float drain_pF;        // estimated own-output drain cap = N * unit_drain
};

// ASAP7 tech: GATE_OUTCAP = 2.85 fF for an INVx1 (unit-size). The buffer's
// own output-stage source/drain parasitic scales with drive strength, so
// drain_pF ≈ size × kGateOutCapPerSizePF. We deduct this from a master's
// caplimit before checking coverage / picking — a buffer's output drain
// loads its own output and eats into the drive budget.
constexpr float kGateOutCapPerSizePF = 0.00285f;

// ----------------------------------------------------------------------------
// CAPLIMIT SOURCE — clocksyn-style gain × size × gatecappersize
// ----------------------------------------------------------------------------
// We currently use clocksyn's gain-based drive budget instead of Liberty's
// max_capacitance because Liberty's values on ASAP7 are very permissive
// (BUFx2 = 92 fF Liberty vs. 3.7 fF clocksyn) and would collapse every
// adaptive/set-cover decision to BUFx2. The gain-based formula gives tight,
// physically-meaningful budgets that match what clocksyn would compute on the
// same design, enabling apples-to-apples comparison.
//
// Formula:  caplimit_pF = kBufFixedGain × drive_strength × kGateInCapPerSizePF
// ASAP7   : buf_fixed_gain = 3, GATE_INCAP = 0.619928 fF / unit size
//           → BUFx2 → 3.72 fF, BUFx4 → 7.44 fF, BUFx24 → 44.63 fF
//
// TODO: expose `kBufFixedGain` and `kGateInCapPerSizePF` as TCL knobs
//       (e.g., -buf_gain, -gate_cap_per_size) so the user can:
//         * tune the gain for tighter/looser sizing
//         * point at a different tech without recompiling
//         * flip back to Liberty max_capacitance (call queryBufferMaxLoad)
constexpr float kBufFixedGain        = 3.0f;        // clocksyn buf_fixed_gain
constexpr float kGateInCapPerSizePF  = 0.000619928f; // ASAP7 GATE_INCAP (pF/unit)

// Clocksyn-style caplimit in pF. Returns 0 if drive_strength is unparseable.
float caplimitClocksynStylePF(int drive_strength)
{
  if (drive_strength <= 0) return 0.0f;
  return kBufFixedGain * static_cast<float>(drive_strength)
       * kGateInCapPerSizePF;
}

// Parses "BUFx<N>_..." or "INVx<N>_..." style drive-strength integers.
// Returns 0 if no "x<digits>" token is found.
int parseDriveStrength(const std::string& name)
{
  for (size_t i = 0; i + 1 < name.size(); ++i) {
    if ((name[i] == 'x' || name[i] == 'X')
        && std::isdigit(static_cast<unsigned char>(name[i + 1]))) {
      int n = 0;
      size_t j = i + 1;
      while (j < name.size() && std::isdigit(static_cast<unsigned char>(name[j]))) {
        n = n * 10 + (name[j] - '0');
        ++j;
      }
      if (n > 0) {
        return n;
      }
    }
  }
  return 0;
}

// Returns Liberty cell for a dbMaster, or nullptr if not in any library.
sta::LibertyCell* libertyCellFor(sta::dbNetwork* network, odb::dbMaster* master)
{
  if (!network || !master) {
    return nullptr;
  }
  sta::Cell* cell = network->dbToSta(master);
  if (!cell) {
    return nullptr;
  }
  return network->libertyCell(cell);
}

// Reads max_capacitance (farads) from the buffer's OUTPUT port. Falls back to
// `drive_strength * fallback_unit_cap_pF` if Liberty has no limit (e.g.,
// abstract LEF-only flows). Returns picofarads.
float queryBufferMaxLoad(sta::dbNetwork* network,
                         odb::dbMaster* master,
                         const std::string& master_name,
                         float fallback_unit_cap_pF)
{
  sta::LibertyCell* lib = libertyCellFor(network, master);
  if (lib && lib->isBuffer()) {
    sta::LibertyPort* in_port = nullptr;
    sta::LibertyPort* out_port = nullptr;
    lib->bufferPorts(in_port, out_port);
    if (out_port) {
      float limit_F = 0.0f;
      bool exists = false;
      out_port->capacitanceLimit(sta::MinMax::max(), limit_F, exists);
      if (exists && limit_F > 0.0f) {
        return limit_F * 1e12f;  // F → pF
      }
    }
  }
  // Liberty unavailable / no max_cap → use clocksyn-style heuristic:
  // caplimit ≈ drive_strength × fallback_unit_cap.
  int n = parseDriveStrength(master_name);
  return (n > 0) ? n * fallback_unit_cap_pF : 0.0f;
}

// Reads input-pin capacitance (picofarads) from the buffer's INPUT port.
// 0.0 if Liberty unavailable.
float queryBufferInputCap(sta::dbNetwork* network, odb::dbMaster* master)
{
  sta::LibertyCell* lib = libertyCellFor(network, master);
  if (!lib || !lib->isBuffer()) {
    return 0.0f;
  }
  sta::LibertyPort* in_port = nullptr;
  sta::LibertyPort* out_port = nullptr;
  lib->bufferPorts(in_port, out_port);
  if (!in_port) {
    return 0.0f;
  }
  return in_port->capacitance() * 1e12f;  // F → pF
}

// Liberty input-pin capacitance (pF) for a specific (master, mterm) pin.
float queryPinInputCap(sta::dbNetwork* network,
                       odb::dbMaster* master,
                       const std::string& pin_name)
{
  sta::LibertyCell* lib = libertyCellFor(network, master);
  if (!lib) {
    return 0.0f;
  }
  sta::LibertyPort* port = lib->findLibertyPort(pin_name.c_str());
  if (!port) {
    return 0.0f;
  }
  return port->capacitance() * 1e12f;
}

// Returns wire capacitance per micron for a routing layer:
//   c_per_um = width * area_cap + 2 * edge_cap   (pF/um)
// Uses dbTechLayer::getCapacitance() (pF/um²) and getEdgeCapacitance() (pF/um).
float layerCapPerUmPF(odb::dbBlock* block, odb::dbTechLayer* layer)
{
  if (!layer || !block) {
    return 0.0f;
  }
  const double dbu_per_um = block->getDbUnitsPerMicron();
  const double width_um = layer->getWidth() / dbu_per_um;
  const double area_cap = layer->getCapacitance();   // pF / um²
  const double edge_cap = layer->getEdgeCapacitance();  // pF / um
  return static_cast<float>(width_um * area_cap + 2.0 * edge_cap);
}

// Distributes wire cap onto intersections. For each mesh-wire segment, we
// find the intersections that sit on it (matching layer + axis-aligned
// coordinate), sort them along the wire's primary axis, and assign each
// inter-intersection sub-segment's cap equally to its two endpoint
// intersections. End-of-wire stubs (between the wire boundary and the first/
// last intersection) are also charged to the nearest endpoint intersection.
std::vector<float> computeWireCapPerIntersection(
    odb::dbBlock* block,
    const std::vector<MeshWire>& mesh_wires,
    const std::vector<GridIntersection>& grid_intersections,
    odb::dbTechLayer* h_layer,
    odb::dbTechLayer* v_layer)
{
  const size_t N = grid_intersections.size();
  std::vector<float> wire_cap(N, 0.0f);

  if (N == 0 || mesh_wires.empty() || !block) {
    return wire_cap;
  }
  const double dbu_per_um = block->getDbUnitsPerMicron();
  const float h_cap_per_um = layerCapPerUmPF(block, h_layer);
  const float v_cap_per_um = layerCapPerUmPF(block, v_layer);

  for (const MeshWire& wire : mesh_wires) {
    const bool horiz = wire.is_horizontal;
    const float cap_per_um = horiz ? h_cap_per_um : v_cap_per_um;
    if (cap_per_um <= 0.0f) {
      continue;
    }
    const int wire_x_min = wire.rect.xMin();
    const int wire_x_max = wire.rect.xMax();
    const int wire_y_min = wire.rect.yMin();
    const int wire_y_max = wire.rect.yMax();
    const int wire_axis = horiz
        ? (wire_y_min + wire_y_max) / 2
        : (wire_x_min + wire_x_max) / 2;

    // Find intersections sitting on this wire on the same layer.
    // Indices into grid_intersections.
    std::vector<std::pair<int, size_t>> on_wire;
    on_wire.reserve(8);
    for (size_t i = 0; i < N; ++i) {
      const GridIntersection& inter = grid_intersections[i];
      if (horiz) {
        if (inter.y != wire_axis) continue;
        if (inter.x < wire_x_min || inter.x > wire_x_max) continue;
        on_wire.emplace_back(inter.x, i);
      } else {
        if (inter.x != wire_axis) continue;
        if (inter.y < wire_y_min || inter.y > wire_y_max) continue;
        on_wire.emplace_back(inter.y, i);
      }
    }
    if (on_wire.empty()) {
      continue;
    }
    std::sort(on_wire.begin(), on_wire.end());

    const int wire_lo = horiz ? wire_x_min : wire_y_min;
    const int wire_hi = horiz ? wire_x_max : wire_y_max;

    // Leading stub: [wire_lo .. first_inter]
    {
      const int len_dbu = on_wire.front().first - wire_lo;
      if (len_dbu > 0) {
        const float len_um = len_dbu / dbu_per_um;
        wire_cap[on_wire.front().second] += len_um * cap_per_um;
      }
    }
    // Interior segments: split half/half between endpoints.
    for (size_t k = 0; k + 1 < on_wire.size(); ++k) {
      const int len_dbu = on_wire[k + 1].first - on_wire[k].first;
      if (len_dbu <= 0) continue;
      const float len_um = len_dbu / dbu_per_um;
      const float seg_cap = len_um * cap_per_um;
      wire_cap[on_wire[k].second] += 0.5f * seg_cap;
      wire_cap[on_wire[k + 1].second] += 0.5f * seg_cap;
    }
    // Trailing stub: [last_inter .. wire_hi]
    {
      const int len_dbu = wire_hi - on_wire.back().first;
      if (len_dbu > 0) {
        const float len_um = len_dbu / dbu_per_um;
        wire_cap[on_wire.back().second] += len_um * cap_per_um;
      }
    }
  }
  return wire_cap;
}

// Voronoi-style assignment: each sink contributes its Liberty input cap (pF)
// to its nearest grid intersection (squared-distance, no ties broken).
std::vector<float> computeSinkCapPerIntersection(
    sta::dbNetwork* network,
    const std::vector<ClockSink>& sinks,
    const std::vector<GridIntersection>& grid_intersections)
{
  const size_t N = grid_intersections.size();
  std::vector<float> sink_cap(N, 0.0f);
  if (N == 0) {
    return sink_cap;
  }
  for (const ClockSink& sink : sinks) {
    if (!sink.iterm) continue;

    int64_t best_d2 = std::numeric_limits<int64_t>::max();
    size_t best_idx = 0;
    for (size_t i = 0; i < N; ++i) {
      const int64_t dx = static_cast<int64_t>(grid_intersections[i].x) - sink.x;
      const int64_t dy = static_cast<int64_t>(grid_intersections[i].y) - sink.y;
      const int64_t d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = i;
      }
    }

    // Uniform clocksyn-style pin cap (see assignSinksVoronoi for rationale).
    sink_cap[best_idx] += kGateInCapPerSizePF;
  }
  return sink_cap;
}

}  // namespace

void placeBuffersLoadAdaptive(
    odb::dbBlock* block,
    sta::dbNetwork* network,
    utl::Logger* logger,
    odb::dbTechLayer* h_layer,
    odb::dbTechLayer* v_layer,
    const std::vector<MeshWire>& mesh_wires,
    const std::vector<ClockSink>& sinks,
    std::vector<GridIntersection>& grid_intersections,
    const std::vector<std::string>& buffer_masters)
{
  if (!block || !logger) {
    return;
  }
  if (grid_intersections.empty()) {
    logger->warn(CMS, 309, "Load-adaptive: no grid intersections to buffer");
    return;
  }
  if (buffer_masters.empty()) {
    logger->error(CMS, 310, "Load-adaptive: empty buffer master list");
    return;
  }

  // Resolve & characterize candidate masters. Skip ones not in the DB.
  // Caplimit comes from clocksyn's gain-based formula (see top-of-file comment
  // block on kBufFixedGain), NOT Liberty max_capacitance. Switch to
  // queryBufferMaxLoad(...) here to revert to the Liberty path.
  std::vector<MasterEntry> candidates;
  candidates.reserve(buffer_masters.size());
  for (const std::string& name : buffer_masters) {
    odb::dbMaster* m = block->getDataBase()->findMaster(name.c_str());
    if (!m) {
      logger->warn(CMS, 311, "Load-adaptive: buffer master '{}' not found, skipping", name);
      continue;
    }
    MasterEntry e;
    e.name = name;
    e.master = m;
    e.drive_strength = parseDriveStrength(name);
    e.max_load_pF    = caplimitClocksynStylePF(e.drive_strength);
    e.input_cap_pF   = queryBufferInputCap(network, m);
    e.drain_pF       = e.drive_strength * kGateOutCapPerSizePF;
    if (e.max_load_pF <= 0.0f) {
      logger->warn(CMS, 312,
                   "Load-adaptive: '{}' has no parseable drive strength (xN) "
                   "for clocksyn-style caplimit, skipping", name);
      continue;
    }
    candidates.push_back(e);
  }
  if (candidates.empty()) {
    logger->error(CMS, 313, "Load-adaptive: no usable buffer masters resolved");
    return;
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const MasterEntry& a, const MasterEntry& b) {
              return a.max_load_pF < b.max_load_pF;
            });

  logger->info(CMS, 314,
               "Load-adaptive buffer library ({} candidates, clocksyn-style "
               "caplimit = {} x size x {:.4g} pF):",
               candidates.size(), kBufFixedGain, kGateInCapPerSizePF);
  for (const MasterEntry& e : candidates) {
    logger->info(CMS, 315,
                 "  {}: size={}, caplimit = {:.4g} pF, "
                 "(informational: drain = {:.4g} pF, Liberty_in_cap = {:.4g} pF)",
                 e.name, e.drive_strength, e.max_load_pF,
                 e.drain_pF, e.input_cap_pF);
  }

  // Per-intersection load = wire cap (half-segment apportioned) + sink caps.
  std::vector<float> wire_cap = computeWireCapPerIntersection(
      block, mesh_wires, grid_intersections, h_layer, v_layer);
  std::vector<float> sink_cap = computeSinkCapPerIntersection(
      network, sinks, grid_intersections);

  // Place a buffer at every intersection, sized to local load.
  const float largest_cap = candidates.back().max_load_pF;
  std::map<std::string, int> size_counts;
  int placed = 0;
  int overloaded = 0;
  float min_load = std::numeric_limits<float>::max();
  float max_load = 0.0f;

  for (size_t i = 0; i < grid_intersections.size(); ++i) {
    GridIntersection& inter = grid_intersections[i];
    const float load_pF = wire_cap[i] + sink_cap[i];
    min_load = std::min(min_load, load_pF);
    max_load = std::max(max_load, load_pF);

    // Smallest master whose caplimit covers this intersection's load.
    // We do NOT subtract the candidate's own drain here: clocksyn's
    // compute_node_total_cap buffer-drain term is applied to PRE-EXISTING
    // buffers in the graph, and during the initial single-pass placement
    // the `buffers` vector is empty, so that contribution is zero in clocksyn
    // too. The `drain_pF` field is retained for future iterative refinement
    // (re-running coverage after some buffers are already placed).
    const MasterEntry* pick = nullptr;
    for (const MasterEntry& e : candidates) {
      if (e.max_load_pF >= load_pF) {
        pick = &e;
        break;
      }
    }
    if (!pick) {
      pick = &candidates.back();
      overloaded++;
    }

    const std::string buf_name = "mesh_buf_"
        + std::to_string(inter.x) + "_" + std::to_string(inter.y);
    odb::dbInst* buf_inst = odb::dbInst::create(block, pick->master, buf_name.c_str());
    if (!buf_inst) {
      logger->warn(CMS, 316, "Load-adaptive: failed to create instance '{}'", buf_name);
      continue;
    }
    buf_inst->setLocation(inter.x, inter.y);
    buf_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    inter.buffer_inst = buf_inst;
    inter.has_buffer = true;
    size_counts[pick->name]++;
    placed++;
  }

  logger->info(CMS, 317,
               "Load-adaptive: placed {} buffers, load range [{:.4g}, {:.4g}] pF",
               placed, min_load, max_load);
  for (const auto& [name, count] : size_counts) {
    logger->info(CMS, 318, "  {}: {} instances", name, count);
  }
  if (overloaded > 0) {
    logger->warn(CMS, 319,
                 "Load-adaptive: {} intersections exceed largest master's max_load ({:.4g} pF) — using largest",
                 overloaded, largest_cap);
  }
}

namespace {

// Edge of the mesh-graph between two intersections.
// `cap_pF`   — segment capacitance for budget accounting (BFS exhaust cap).
// `r_ohm`    — segment resistance for Dijkstra priority (lumped R from start).
struct Edge {
  size_t nb;
  float  cap_pF;
  float  r_ohm;
};

// Build N/S/E/W adjacency for intersections by walking mesh wires.
// adj[i] = list of {neighbor, cap, r}.
std::vector<std::vector<Edge>> buildAdjacency(
    odb::dbBlock* block,
    const std::vector<MeshWire>& mesh_wires,
    const std::vector<GridIntersection>& grid_intersections,
    odb::dbTechLayer* h_layer,
    odb::dbTechLayer* v_layer)
{
  const size_t N = grid_intersections.size();
  // adj[i] = list of (neighbor_idx, edge_cap_pF, edge_r_ohm)
  std::vector<std::vector<Edge>> adj(N);

  if (N == 0 || mesh_wires.empty() || !block) {
    return adj;
  }
  const double dbu_per_um = block->getDbUnitsPerMicron();
  const float h_cap_per_um = layerCapPerUmPF(block, h_layer);
  const float v_cap_per_um = layerCapPerUmPF(block, v_layer);
  // R per µm (Ω/µm). set_layer_rc -resistance stores this directly.
  const float h_r_per_um = h_layer ? (float) h_layer->getResistance() : 0.0f;
  const float v_r_per_um = v_layer ? (float) v_layer->getResistance() : 0.0f;

  for (const MeshWire& wire : mesh_wires) {
    const bool horiz = wire.is_horizontal;
    const float cap_per_um = horiz ? h_cap_per_um : v_cap_per_um;
    const float r_per_um   = horiz ? h_r_per_um   : v_r_per_um;
    if (cap_per_um <= 0.0f) continue;

    const int wire_axis = horiz
        ? (wire.rect.yMin() + wire.rect.yMax()) / 2
        : (wire.rect.xMin() + wire.rect.xMax()) / 2;
    const int wire_lo = horiz ? wire.rect.xMin() : wire.rect.yMin();
    const int wire_hi = horiz ? wire.rect.xMax() : wire.rect.yMax();

    std::vector<std::pair<int, size_t>> on_wire;
    on_wire.reserve(8);
    for (size_t i = 0; i < N; ++i) {
      const GridIntersection& inter = grid_intersections[i];
      if (horiz) {
        if (inter.y != wire_axis) continue;
        if (inter.x < wire_lo || inter.x > wire_hi) continue;
        on_wire.emplace_back(inter.x, i);
      } else {
        if (inter.x != wire_axis) continue;
        if (inter.y < wire_lo || inter.y > wire_hi) continue;
        on_wire.emplace_back(inter.y, i);
      }
    }
    if (on_wire.size() < 2) continue;
    std::sort(on_wire.begin(), on_wire.end());

    for (size_t k = 0; k + 1 < on_wire.size(); ++k) {
      const int len_dbu = on_wire[k + 1].first - on_wire[k].first;
      if (len_dbu <= 0) continue;
      const float len_um = len_dbu / dbu_per_um;
      const float seg_cap = len_um * cap_per_um;
      const float seg_r   = len_um * r_per_um;
      const size_t a = on_wire[k].second;
      const size_t b = on_wire[k + 1].second;
      adj[a].push_back({b, seg_cap, seg_r});
      adj[b].push_back({a, seg_cap, seg_r});
    }
  }
  return adj;
}

// For each sink, which intersection is its nearest (Voronoi host)?
// Returns parallel vectors: sinks_at[i] = indices into the original sink list.
// Also returns the per-intersection cap from those sinks (same as in mode 1).
struct SinkAssignment {
  std::vector<std::vector<int>> sinks_at;       // intersection -> sink indices
  std::vector<float>            sink_cap_pF;    // intersection -> total sink cap
};

// `stub_cap_per_um` (pF/µm): per-µm wire cap of the sink-to-mesh stub.
// Caller passes the appropriate layer's value (typically the mesh layer
// cap as a conservative estimate). Mirrors clocksyn's behavior where the
// sink-segment wire cap is part of each grid node's total_cap via
// compute_node_total_cap's wire-half loop.
//
// `dbu_per_um`: from block->getDbUnitsPerMicron(), used to convert
// Manhattan distance from DBU to µm.
SinkAssignment assignSinksVoronoi(
    sta::dbNetwork* network,
    const std::vector<ClockSink>& sinks,
    const std::vector<GridIntersection>& grid_intersections,
    double dbu_per_um,
    float stub_cap_per_um)
{
  const size_t N = grid_intersections.size();
  SinkAssignment out;
  out.sinks_at.assign(N, {});
  out.sink_cap_pF.assign(N, 0.0f);
  if (N == 0) return out;

  for (size_t s = 0; s < sinks.size(); ++s) {
    const ClockSink& sink = sinks[s];
    if (!sink.iterm) continue;
    int64_t best_d2 = std::numeric_limits<int64_t>::max();
    size_t best_i = 0;
    for (size_t i = 0; i < N; ++i) {
      const int64_t dx = static_cast<int64_t>(grid_intersections[i].x) - sink.x;
      const int64_t dy = static_cast<int64_t>(grid_intersections[i].y) - sink.y;
      const int64_t d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best_i = i; }
    }
    out.sinks_at[best_i].push_back(static_cast<int>(s));
    // Clocksyn-equivalent pin cap: sink.cap × gatecappersize, where sink.cap
    // is the ISPD-parsed sink size (typically 1.0 for flops) and
    // gatecappersize = kGateInCapPerSizePF (ASAP7: 0.620 fF). One uniform
    // value per sink rather than Liberty per-cell-type, mirroring clocksyn's
    // compute_node_total_cap sinks loop.
    out.sink_cap_pF[best_i] += kGateInCapPerSizePF;

    // Sink-to-mesh stub wire cap. Manhattan distance, HALF cap to host
    // intersection (matches clocksyn's π-model wire-half split: the other
    // half conceptually sits on the sink node which BFS never visits).
    const int64_t dx_dbu = std::abs(static_cast<int64_t>(grid_intersections[best_i].x) - sink.x);
    const int64_t dy_dbu = std::abs(static_cast<int64_t>(grid_intersections[best_i].y) - sink.y);
    const float stub_um = static_cast<float>((dx_dbu + dy_dbu) / dbu_per_um);
    out.sink_cap_pF[best_i] += 0.5f * stub_um * stub_cap_per_um;
  }
  return out;
}

// Dijkstra-by-lumped-R flood from `start_node` through `adj` until accumulated
// cap exceeds `budget_pF`. This matches clocksyn's `find_cap_coverage`
// (grid.C:2422): the frontier is a priority queue keyed by total resistance
// from the start node, NOT hop count. On a uniform mesh the two are
// equivalent; on a non-uniform mesh (blockages, mixed layer widths) Dijkstra
// gives the true "closest-by-R" exploration order.
std::vector<int> computeCoverage(
    size_t start_node,
    float budget_pF,
    const std::vector<std::vector<Edge>>& adj,
    const std::vector<float>& wire_cap_local,
    const SinkAssignment& sink_asgn)
{
  std::vector<int> covered;
  const size_t N = adj.size();
  if (N == 0 || start_node >= N) return covered;

  std::vector<bool> visited(N, false);
  using QEntry = std::pair<float, size_t>;  // (lumped_R_from_start, node)
  std::priority_queue<QEntry, std::vector<QEntry>, std::greater<>> q;
  q.push({0.0f, start_node});
  float accumulated = 0.0f;

  while (!q.empty()) {
    const auto [r_from_start, cur] = q.top();
    q.pop();
    if (visited[cur]) continue;
    visited[cur] = true;

    const float local = wire_cap_local[cur] + sink_asgn.sink_cap_pF[cur];
    if (accumulated + local > budget_pF) {
      break;
    }
    accumulated += local;
    for (int s_idx : sink_asgn.sinks_at[cur]) covered.push_back(s_idx);

    for (const auto& e : adj[cur]) {
      if (!visited[e.nb]) {
        q.push({r_from_start + e.r_ohm, e.nb});
      }
    }
  }
  return covered;
}

}  // anonymous namespace

void placeBuffersSetCover(
    odb::dbBlock* block,
    sta::dbNetwork* network,
    utl::Logger* logger,
    odb::dbTechLayer* h_layer,
    odb::dbTechLayer* v_layer,
    const std::vector<MeshWire>& mesh_wires,
    const std::vector<ClockSink>& sinks,
    std::vector<GridIntersection>& grid_intersections,
    const std::vector<std::string>& buffer_masters)
{
  if (!block || !logger) return;
  if (grid_intersections.empty()) {
    logger->warn(CMS, 320, "Set-cover: no grid intersections to buffer");
    return;
  }
  if (buffer_masters.empty()) {
    logger->error(CMS, 321, "Set-cover: empty buffer master list");
    return;
  }

  // Resolve & characterize candidate masters (same as load-adaptive).
  // Caplimit uses clocksyn's gain × size × gatecappersize formula — see the
  // file-top comment on kBufFixedGain. Switch to queryBufferMaxLoad(...) to
  // revert to Liberty max_capacitance.
  std::vector<MasterEntry> candidates;
  candidates.reserve(buffer_masters.size());
  for (const std::string& name : buffer_masters) {
    odb::dbMaster* m = block->getDataBase()->findMaster(name.c_str());
    if (!m) {
      logger->warn(CMS, 322, "Set-cover: buffer master '{}' not found, skipping", name);
      continue;
    }
    MasterEntry e;
    e.name = name; e.master = m;
    e.drive_strength = parseDriveStrength(name);
    e.max_load_pF    = caplimitClocksynStylePF(e.drive_strength);
    e.input_cap_pF   = queryBufferInputCap(network, m);
    e.drain_pF       = e.drive_strength * kGateOutCapPerSizePF;
    if (e.max_load_pF <= 0.0f) {
      logger->warn(CMS, 323,
                   "Set-cover: '{}' has no parseable drive strength (xN) for "
                   "clocksyn-style caplimit, skipping", name);
      continue;
    }
    candidates.push_back(e);
  }
  if (candidates.empty()) {
    logger->error(CMS, 324, "Set-cover: no usable buffer masters resolved");
    return;
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const MasterEntry& a, const MasterEntry& b) {
              return a.max_load_pF < b.max_load_pF;
            });

  const size_t N = grid_intersections.size();
  const size_t S = sinks.size();

  // Inputs to the coverage flood.
  const std::vector<float> wire_cap_local = computeWireCapPerIntersection(
      block, mesh_wires, grid_intersections, h_layer, v_layer);
  // Stub cap = mesh layer cap (use h_layer; v_layer cap is similar).
  // dbu_per_um needed to convert Manhattan distance from DBU.
  const double dbu_per_um = block->getDbUnitsPerMicron();
  const float stub_cap_per_um = layerCapPerUmPF(block, h_layer);
  const SinkAssignment sink_asgn = assignSinksVoronoi(
      network, sinks, grid_intersections, dbu_per_um, stub_cap_per_um);
  const auto adj = buildAdjacency(block, mesh_wires, grid_intersections, h_layer, v_layer);

  // Pre-compute CR[i][m] = sink indices reachable from intersection i under
  // candidates[m]'s budget. Same shape as clocksyn's CR map but indexed by
  // ints. Cost: O(N * M * |reachable|) BFS work — fine for typical meshes.
  std::vector<std::vector<std::vector<int>>> CR(N, std::vector<std::vector<int>>(candidates.size()));
  for (size_t i = 0; i < N; ++i) {
    for (size_t m = 0; m < candidates.size(); ++m) {
      // Use the candidate's full caplimit as the BFS budget. clocksyn's
      // `compute_node_total_cap` adds buffer drain only for pre-existing
      // buffers (i.e., already in the `buffers` vector); during the initial
      // single-pass set-cover the contribution is zero, so we match that.
      // The `drain_pF` field is kept on MasterEntry for future iterative use.
      CR[i][m] = computeCoverage(i, candidates[m].max_load_pF,
                                 adj, wire_cap_local, sink_asgn);
    }
  }

  logger->info(CMS, 325,
               "Set-cover: {} candidate masters, {} intersections, {} sinks",
               candidates.size(), N, S);

  // Greedy outer loop — clocksyn-equivalent (grid.C:247-310):
  //   - One buffer per intersection (line 257 in clocksyn: skip if i already
  //     has a pick of any master). NO re-pick at the same i.
  //   - No Corollary 1 pruning (not needed when single-pick is enforced).
  std::vector<bool> covered_sink(S, false);
  std::vector<std::pair<size_t, size_t>> picks;     // (intersection, master)
  std::set<size_t> picked_intersections;             // for fast "i already picked?"
  int total_covered = 0;
  int prev_covered  = -1;

  while (total_covered < static_cast<int>(S) && total_covered > prev_covered) {
    prev_covered = total_covered;

    size_t best_i = std::numeric_limits<size_t>::max();
    size_t best_m = std::numeric_limits<size_t>::max();
    double best_ceff = std::numeric_limits<double>::max();
    std::vector<int> best_new;

    for (size_t i = 0; i < N; ++i) {
      if (picked_intersections.count(i)) continue;    // ← single-pick per i
      for (size_t m = 0; m < candidates.size(); ++m) {
        std::vector<int> new_cov;
        new_cov.reserve(CR[i][m].size());
        for (int s : CR[i][m]) {
          if (!covered_sink[s]) new_cov.push_back(s);
        }
        if (new_cov.empty()) continue;
        const double size_proxy = (candidates[m].drive_strength > 0)
            ? static_cast<double>(candidates[m].drive_strength)
            : static_cast<double>(candidates[m].max_load_pF);
        const double ceff = size_proxy / static_cast<double>(new_cov.size());
        if (ceff < best_ceff) {
          best_ceff = ceff;
          best_i = i; best_m = m;
          best_new = std::move(new_cov);
        }
      }
    }
    if (best_i == std::numeric_limits<size_t>::max()) break;

    picks.emplace_back(best_i, best_m);
    picked_intersections.insert(best_i);
    for (int s : best_new) {
      if (!covered_sink[s]) { covered_sink[s] = true; total_covered++; }
    }
  }

  if (total_covered < static_cast<int>(S)) {
    logger->warn(CMS, 326,
                 "Set-cover: covered only {} of {} sinks — consider larger "
                 "buffers or denser mesh", total_covered, S);
  }

  // Single-pick greedy enforces "one master per intersection" already, so no
  // Corollary 1 pruning needed — `picks` already has unique intersections.

  // Place a buffer at every picked intersection.
  std::map<std::string, int> size_counts;
  int placed = 0;
  for (const auto& [i, m] : picks) {
    GridIntersection& inter = grid_intersections[i];
    const MasterEntry& pick = candidates[m];
    const std::string buf_name = "mesh_buf_"
        + std::to_string(inter.x) + "_" + std::to_string(inter.y);
    odb::dbInst* buf_inst = odb::dbInst::create(block, pick.master, buf_name.c_str());
    if (!buf_inst) {
      logger->warn(CMS, 327, "Set-cover: failed to create instance '{}'", buf_name);
      continue;
    }
    buf_inst->setLocation(inter.x, inter.y);
    buf_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    inter.buffer_inst = buf_inst;
    inter.has_buffer = true;
    size_counts[pick.name]++;
    placed++;
  }

  logger->info(CMS, 328,
               "Set-cover: placed {} buffers covering {}/{} sinks, "
               "{} intersections left empty",
               placed, total_covered, S, N - picks.size());
  for (const auto& [name, count] : size_counts) {
    logger->info(CMS, 329, "  {}: {} instances", name, count);
  }
}

}  // namespace cms
