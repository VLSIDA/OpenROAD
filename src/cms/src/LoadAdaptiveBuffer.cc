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
  float max_load_pF;   // Liberty max_capacitance on the output pin
  float input_cap_pF;  // Liberty input-pin cap (informational only)
};

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

    odb::dbMaster* master = sink.iterm->getInst()->getMaster();
    const std::string pin_name = sink.iterm->getMTerm()->getConstName();
    sink_cap[best_idx] += queryPinInputCap(network, master, pin_name);
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

  // Fallback unit cap (pF) when Liberty has no max_capacitance on the cell.
  // ASAP7 INVx1 input cap ≈ 0.62 fF × buf_fixed_gain ≈ 3 → 1.86 fF/unit.
  // Same magnitude is fine for clocksyn comparison; tune if needed.
  constexpr float kFallbackUnitCapPF = 0.00186f;

  // Resolve & characterize candidate masters. Skip ones not in the DB.
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
    e.max_load_pF = queryBufferMaxLoad(network, m, name, kFallbackUnitCapPF);
    e.input_cap_pF = queryBufferInputCap(network, m);
    if (e.max_load_pF <= 0.0f) {
      logger->warn(CMS, 312, "Load-adaptive: '{}' has no usable max_capacitance, skipping", name);
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

  logger->info(CMS, 314, "Load-adaptive buffer library ({} candidates):", candidates.size());
  for (const MasterEntry& e : candidates) {
    logger->info(CMS, 315, "  {}: max_load = {:.4g} pF, input_cap = {:.4g} pF",
                 e.name, e.max_load_pF, e.input_cap_pF);
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

    // Smallest master with max_load >= load_pF; fallback to largest if none.
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

}  // namespace cms
