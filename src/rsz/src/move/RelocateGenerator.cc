// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RelocateGenerator.hh"

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveCommitter.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "RelocateCandidate.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// Coordinate accessor for a Manhattan axis (0 = x, 1 = y).
int axisCoord(const odb::Point& p, int axis)
{
  return axis == 0 ? p.getX() : p.getY();
}

}  // namespace

RelocateGenerator::RelocateGenerator(const GeneratorContext& context)
    : MoveGenerator(context)
{
}

bool RelocateGenerator::resolveDriver(const Target& target,
                                      sta::Pin*& drvr_pin,
                                      sta::Instance*& drvr_inst,
                                      odb::dbInst*& db_inst) const
{
  drvr_pin = target.resolvedPin(resizer_);
  if (drvr_pin == nullptr) {
    return false;
  }

  drvr_inst = resizer_.network()->instance(drvr_pin);
  if (drvr_inst == nullptr || resizer_.dontTouch(drvr_inst)
      || !resizer_.isLogicStdCell(drvr_inst)) {
    return false;
  }

  // Only relocate combinational logic.  isLogicStdCell() accepts any CORE
  // master, which includes registers, latches and integrated clock-gating
  // cells; moving those disturbs the clock tree and CTS placement.
  // isCombinational() rejects sequential cells (isSequential()), clock gates
  // (isClockGate()), macros and pads.
  sta::LibertyCell* lib_cell = resizer_.network()->libertyCell(drvr_inst);
  if (lib_cell == nullptr || !resizer_.isCombinational(lib_cell)) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove {}: not combinational (sequential, clock "
               "gate, macro or pad)",
               resizer_.network()->pathName(drvr_pin));
    return false;
  }

  db_inst = resizer_.dbNetwork()->staToDb(drvr_inst);
  if (db_inst == nullptr) {
    return false;
  }
  // Fixed instances may not be moved (setOrigin errors on them).
  if (db_inst->isFixed()) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove {}: instance is fixed",
               resizer_.network()->pathName(drvr_pin));
    return false;
  }
  return true;
}

bool RelocateGenerator::collectAnchors(const Target& target,
                                       sta::Pin* drvr_pin,
                                       std::vector<RelocateAnchor>& anchors,
                                       double& c_load_total) const
{
  sta::dbSta* sta = resizer_.sta();
  sta::dbNetwork* db_network = resizer_.dbNetwork();
  sta::Network* network = resizer_.network();
  sta::Graph* graph = resizer_.graph();
  const sta::MinMax* min_max = resizer_.maxAnalysisMode();
  const sta::Scene* scene = target.activeScene(resizer_);

  c_load_total = 0.0;
  anchors.clear();

  // This gate's own drive resistance sets the output-net (fanout) segment R.
  const double r_g = resizer_.driveResistance(drvr_pin);

  // --- Input anchor: driver of the gate's critical input net ----------------
  // The worst path enters the gate through one input stage.  inputPath() is the
  // gate's critical input pin (its cap is C_pin); prevDriverPath() is the
  // upstream driver's output pin that feeds it -- its location is L_D and its
  // drive resistance is R_up.  When the gate is fed directly by a startpoint /
  // primary input (no upstream logic driver on this stage) anchor on the input
  // pin itself with neutral RC (R_up = R_g) so the input side adds no RC shift.
  const sta::Path* in_path = target.inputPath(resizer_);
  const sta::Path* prev_path = target.prevDriverPath(resizer_);
  const sta::Pin* in_pin
      = in_path != nullptr ? in_path->pin(resizer_.staState()) : nullptr;
  const sta::Pin* up_pin
      = prev_path != nullptr ? prev_path->pin(resizer_.staState()) : nullptr;

  odb::Point l_d;
  double r_up = r_g;
  if (up_pin != nullptr) {
    l_d = db_network->location(up_pin);
    r_up = resizer_.driveResistance(up_pin);
  } else if (in_pin != nullptr) {
    l_d = db_network->location(in_pin);
  } else {
    // No critical input available; the plain-midpoint fallback handles it.
    return false;
  }

  double c_pin = 0.0;
  if (in_pin != nullptr) {
    sta::LibertyPort* in_port = network->libertyPort(in_pin);
    if (in_port != nullptr) {
      c_pin = resizer_.portCapacitance(in_port, scene);
    }
  }
  // Input-side criticality weight for the Elmore cost scoring (the geometric
  // average below is driven by the sink weights).  The path slack is negative
  // on a violating target; floor keeps the anchor's weight positive.
  const double w_in = std::max(-static_cast<double>(target.slack), 1e-12);

  // --- Output anchors: negative-slack (critical) fanout sinks only ----------
  // C_load sums EVERY fanout pin cap -- the gate-side drive delay depends on
  // the whole net -- but only critical (negative-slack) sinks anchor the gate
  // geometrically.  Pruning the non-critical loads removes the high-fanout
  // dilution where a sink's many non-critical siblings pulled the gate off the
  // critical corridor.
  std::vector<RelocateAnchor> sinks;
  sta::Vertex* drvr_vertex = graph->pinDrvrVertex(drvr_pin);
  if (drvr_vertex != nullptr) {
    sta::VertexOutEdgeIterator edge_iter(drvr_vertex, graph);
    while (edge_iter.hasNext()) {
      sta::Edge* edge = edge_iter.next();
      sta::Vertex* load_vertex = edge->to(graph);
      const sta::Pin* load_pin = load_vertex->pin();
      if (load_pin == nullptr || load_pin == drvr_pin) {
        continue;
      }
      double cap = 0.0;
      sta::LibertyPort* load_port = network->libertyPort(load_pin);
      if (load_port != nullptr) {
        cap = resizer_.portCapacitance(load_port, scene);
      }
      c_load_total += cap;
      const double slack
          = static_cast<double>(sta->slack(load_vertex, min_max));
      if (slack < 0.0) {
        // weight = max(0, -slack); strictly positive for a critical sink.
        sinks.push_back({db_network->location(load_pin),
                         -slack,
                         r_g,
                         cap,
                         /*is_fanout=*/true});
      }
    }
  }

  if (sinks.empty()) {
    // No critical sinks: caller falls back to the plain critical-path midpoint
    // rather than anchoring on non-critical loads.
    return false;
  }

  // Emit the single input anchor first, then the critical sinks.
  anchors.push_back({l_d, w_in, r_up, c_pin, /*is_fanout=*/false});
  for (const RelocateAnchor& s : sinks) {
    anchors.push_back(s);
  }
  return true;
}

bool RelocateGenerator::computeBestLocation(const Target& target,
                                            sta::Pin* drvr_pin,
                                            odb::Point& result) const
{
  std::vector<RelocateAnchor> anchors;
  double c_load_total = 0.0;
  if (!collectAnchors(target, drvr_pin, anchors, c_load_total)) {
    // No usable anchors: fall back to the plain path midpoint.
    return computeCriticalPathLocation(target, drvr_pin, result);
  }

  const sta::Scene* scene = target.activeScene(resizer_);
  double wire_res = 0.0;  // ohms/meter
  double wire_cap = 0.0;  // farads/meter
  resizer_.estimateParasitics()->wireSignalRC(scene, wire_res, wire_cap);

  const double r_g = resizer_.driveResistance(drvr_pin);

  // anchors.front() is the input anchor (is_fanout == false); the remaining
  // anchors are the critical (negative-slack) sinks.  L_D / R_up / C_pin come
  // from that input anchor.
  const RelocateAnchor& input = anchors.front();
  const odb::Point l_d = input.loc;
  const double r_up = input.res;
  const double c_pin = input.cap;

  // RC/criticality shift as a signed wire length:
  //   rc_shift = (R_g - R_up)/(2r) + (C_load - C_pin)/(2c)
  // The gate slides toward the sinks when it is the weaker driver (R_g > R_up)
  // and/or drives the heavier load (C_load > C_pin), and toward the upstream
  // driver otherwise.  The magnitude is independent of which sink we consider;
  // only its direction (toward each sink) and the per-sink half-span clamp
  // differ, so it is computed once here and reapplied per sink below.
  double shift_m = 0.0;
  if (wire_res > 0.0) {
    shift_m += (r_g - r_up) / (2.0 * wire_res);
  }
  if (wire_cap > 0.0) {
    shift_m += (c_load_total - c_pin) / (2.0 * wire_cap);
  }
  // metersToDbu() rejects negative distances, so carry the magnitude and sign
  // separately.
  const double shift_dbu_mag = resizer_.metersToDbu(std::abs(shift_m));
  const double shift_sign = shift_m < 0.0 ? -1.0 : 1.0;

  // Two candidate locations, each a criticality-weighted average over the
  // critical sinks of the per-sink ideal gate position on the L_D -> s_i path
  // (Manhattan wirelength separates x and y, so each axis is optimized
  // independently):
  //   0: plain midpoint(L_D, s_i)                       (no RC shift)
  //   1: midpoint(L_D, s_i) + rc_shift along L_D -> s_i (per-sink Elmore ideal)
  // Averaging the per-sink points -- rather than shifting one lumped centroid
  // -- keeps each sink's own driver->sink corridor and half-span clamp.
  constexpr int kRcShift = 1;
  constexpr int kNumCandidates = 2;
  odb::Point candidates_axis[2][kNumCandidates];
  for (int axis = 0; axis < 2; ++axis) {
    const double a_coord = axisCoord(l_d, axis);
    double w_sum = 0.0;
    double mid_num = 0.0;
    double rc_num = 0.0;
    for (const RelocateAnchor& s : anchors) {
      if (!s.is_fanout) {
        continue;
      }
      const double b_coord = axisCoord(s.loc, axis);
      const double midpoint = (a_coord + b_coord) / 2.0;
      // Cap the shift to half the driver-sink span so the RC-shifted point
      // stays interior to this sink's corridor (a genuine shift, never a jump
      // onto an endpoint pin, which the local Elmore model over-favors but real
      // timing punishes).
      const double half_span = std::abs(b_coord - a_coord) / 2.0;
      const double shift_dbu
          = std::clamp(shift_sign * shift_dbu_mag, -half_span, half_span);
      const double dir = (b_coord > a_coord) ? 1.0 : -1.0;
      const double p_i = midpoint + dir * shift_dbu;
      w_sum += s.weight;
      mid_num += s.weight * midpoint;
      rc_num += s.weight * p_i;
    }
    const double midpoint_avg = w_sum > 0.0 ? mid_num / w_sum : a_coord;
    const double rc_avg = w_sum > 0.0 ? rc_num / w_sum : a_coord;
    const double picks[kNumCandidates] = {midpoint_avg, rc_avg};
    for (int i = 0; i < kNumCandidates; ++i) {
      const int coord = static_cast<int>(std::lround(picks[i]));
      candidates_axis[axis][i]
          = axis == 0 ? odb::Point(coord, 0) : odb::Point(0, coord);
    }
  }

  // Score each combined candidate point by the criticality-weighted Elmore wire
  // delay and keep the cheapest.  L is the true Manhattan distance (in meters);
  // the per-anchor position-dependent delay is
  //   (R*c + r*C)*L + (r*c/2)*L^2.
  auto cost = [&](const odb::Point& p) {
    double total = 0.0;
    for (const RelocateAnchor& a : anchors) {
      const int manh = std::abs(p.getX() - a.loc.getX())
                       + std::abs(p.getY() - a.loc.getY());
      const double len = resizer_.dbuToMeters(manh);
      const double linear = (a.res * wire_cap + wire_res * a.cap) * len;
      const double quad = 0.5 * wire_res * wire_cap * len * len;
      total += a.weight * (linear + quad);
    }
    return total;
  };

  double best_cost = 0.0;
  bool have_best = false;
  int best_i = -1;
  for (int i = 0; i < kNumCandidates; ++i) {
    const odb::Point p(candidates_axis[0][i].getX(),
                       candidates_axis[1][i].getY());
    const double c = cost(p);
    if (!have_best || c < best_cost) {
      best_cost = c;
      result = p;
      have_best = true;
      best_i = i;
    }
  }
  debugPrint(resizer_.logger(),
             RSZ,
             "relocate_move",
             3,
             "RELOCATE {}: anchors={} pick={} loc=({}, {})",
             resizer_.network()->pathName(drvr_pin),
             anchors.size(),
             best_i == kRcShift ? "rc_shift" : "midpoint",
             result.getX(),
             result.getY());
  return have_best;
}

bool RelocateGenerator::computeCriticalPathLocation(const Target& target,
                                                    sta::Pin* drvr_pin,
                                                    odb::Point& result) const
{
  sta::dbNetwork* db_network = resizer_.dbNetwork();

  // FROM: the driver of this gate's most-critical input, i.e. the upstream
  // source that feeds the input pin lying on the worst path through the gate.
  // prevDriverPath() is that upstream driver output pin on the timing path.
  // When the gate is fed directly by a startpoint (no upstream driver on this
  // segment), fall back to the critical input pin location itself.
  const sta::Path* from_path = target.prevDriverPath(resizer_);
  if (from_path == nullptr) {
    from_path = target.inputPath(resizer_);
  }
  if (from_path == nullptr) {
    return false;
  }
  const sta::Pin* from_pin = from_path->pin(resizer_.staState());
  if (from_pin == nullptr) {
    return false;
  }
  const odb::Point from_loc = db_network->location(from_pin);

  // TO: the gate's critical output load, i.e. the sink on the output net that
  // lies on the worst path.  Expanding the endpoint path, the node one step
  // toward the endpoint from the driver stage is exactly that critical sink
  // (for a single-fanout net it is the net's only load).
  if (target.endpoint_path == nullptr || target.path_index < 0) {
    return false;
  }
  sta::PathExpanded expanded(target.endpoint_path, resizer_.staState());
  const size_t load_index = static_cast<size_t>(target.path_index) + 1;
  if (load_index >= expanded.size()) {
    return false;
  }
  const sta::Path* load_path = expanded.path(load_index);
  if (load_path == nullptr) {
    return false;
  }
  const sta::Pin* load_pin = load_path->pin(resizer_.staState());
  if (load_pin == nullptr) {
    return false;
  }
  const odb::Point to_loc = db_network->location(load_pin);

  // Place the gate at the midpoint between where its critical signal comes
  // from and where its critical output goes, shortening the wire along that
  // path.
  result = odb::Point((from_loc.getX() + to_loc.getX()) / 2,
                      (from_loc.getY() + to_loc.getY()) / 2);
  return true;
}

std::vector<std::unique_ptr<MoveCandidate>> RelocateGenerator::generate(
    const Target& target)
{
  std::vector<std::unique_ptr<MoveCandidate>> candidates;

  sta::Pin* drvr_pin = nullptr;
  sta::Instance* drvr_inst = nullptr;
  odb::dbInst* db_inst = nullptr;
  if (!resolveDriver(target, drvr_pin, drvr_inst, db_inst)) {
    return candidates;
  }

  // Do not relocate the same instance more than once per checkpoint.
  if (committer_.hasPendingMoves(MoveType::kRelocate, drvr_inst)) {
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove {}: has pending RelocateMove",
               resizer_.network()->pathName(drvr_pin));
    return candidates;
  }

  odb::Point new_loc;
  if (!computeBestLocation(target, drvr_pin, new_loc)) {
    return candidates;
  }

  candidates.push_back(std::make_unique<RelocateCandidate>(
      resizer_, target, drvr_pin, drvr_inst, db_inst, new_loc));
  return candidates;
}

}  // namespace rsz
