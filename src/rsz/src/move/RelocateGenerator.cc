// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RelocateGenerator.hh"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
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
#include "sta/PortDirection.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

namespace {

// Coordinate accessor for a Manhattan axis (0 = x, 1 = y).
int axisCoord(const odb::Point& p, int axis)
{
  return axis == 0 ? p.getX() : p.getY();
}

// Criticality-weighted mean of anchor coordinates on one axis.  Returns
// fallback when the weight sum is non-positive.
double weightedMean(const std::vector<std::pair<double, double>>& weighted,
                    double fallback)
{
  double num = 0.0;
  double den = 0.0;
  for (const auto& [coord, weight] : weighted) {
    num += weight * coord;
    den += weight;
  }
  return den > 0.0 ? num / den : fallback;
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
                                       sta::Instance* drvr_inst,
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

  // Near-critical filter: an incident pin is an anchor if its slack lies within
  // the setup slack margin of the target path slack.
  const double slack_threshold = target.slack + run_config_.setup_slack_margin;

  // Raw anchors carry slack; criticality weights are assigned once the slack
  // window across all anchors is known.
  struct RawAnchor
  {
    odb::Point loc;
    double slack;
    double res;
    double cap;
    bool is_fanout;
  };
  std::vector<RawAnchor> raw_fanins;
  std::vector<RawAnchor> raw_fanouts;

  // --- Fanout sinks on the output net ---------------------------------------
  // Every load contributes its pin cap to the total output load (which sets the
  // gate-side delay magnitude); each load is also a weighted geometric anchor.
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
      const sta::Slack slack = sta->slack(load_vertex, min_max);
      raw_fanouts.push_back({db_network->location(load_pin),
                             static_cast<double>(slack),
                             r_g,
                             cap,
                             /*is_fanout=*/true});
    }
  }

  // --- Fanin drivers of each input pin --------------------------------------
  // Each near-critical input net contributes its upstream driver as an anchor;
  // the relevant segment R is the upstream driver's drive resistance and the
  // far-end cap is this gate's input pin cap.
  std::unique_ptr<sta::InstancePinIterator> pin_iter(
      network->pinIterator(drvr_inst));
  while (pin_iter->hasNext()) {
    const sta::Pin* in_pin = pin_iter->next();
    if (!network->direction(in_pin)->isInput()
        || network->isRegClkPin(in_pin)) {
      continue;
    }
    sta::PinSet* drivers = network->drivers(in_pin);
    if (drivers == nullptr || drivers->empty()) {
      continue;
    }
    const sta::Pin* up_pin = *drivers->begin();
    if (up_pin == nullptr || up_pin == in_pin) {
      continue;
    }
    sta::Vertex* in_vertex = graph->pinLoadVertex(in_pin);
    if (in_vertex == nullptr) {
      continue;
    }
    const sta::Slack slack = sta->slack(in_vertex, min_max);
    double c_pin = 0.0;
    sta::LibertyPort* in_port = network->libertyPort(in_pin);
    if (in_port != nullptr) {
      c_pin = resizer_.portCapacitance(in_port, scene);
    }
    const double r_up = resizer_.driveResistance(up_pin);
    raw_fanins.push_back({db_network->location(up_pin),
                          static_cast<double>(slack),
                          r_up,
                          c_pin,
                          /*is_fanout=*/false});
  }

  // When no upstream driver anchor was found (e.g. a register/startpoint whose
  // only fanin is its clock), fall back to the worst-path input anchor the
  // plain-midpoint heuristic uses.  Neutral RC data (this gate's own R and the
  // total output load) zeroes the RC shift for this synthetic anchor, so such a
  // gate degrades gracefully to midpoint placement rather than stacking on its
  // load.
  if (raw_fanins.empty()) {
    const sta::Path* from_path = target.prevDriverPath(resizer_);
    if (from_path == nullptr) {
      from_path = target.inputPath(resizer_);
    }
    if (from_path != nullptr) {
      const sta::Pin* from_pin = from_path->pin(resizer_.staState());
      if (from_pin != nullptr) {
        raw_fanins.push_back({db_network->location(from_pin),
                              target.slack,
                              r_g,
                              c_load_total,
                              /*is_fanout=*/false});
      }
    }
  }

  if (raw_fanins.empty() && raw_fanouts.empty()) {
    return false;
  }

  // Decide the included set per side: keep every near-critical anchor, but
  // always keep the single most-critical anchor of each non-empty side so the
  // path endpoints still anchor the gate even at a zero slack margin.
  auto select = [&](std::vector<RawAnchor>& raw) {
    std::vector<RawAnchor> kept;
    const RawAnchor* worst = nullptr;
    for (const RawAnchor& a : raw) {
      if (a.slack <= slack_threshold) {
        kept.push_back(a);
      }
      if (worst == nullptr || a.slack < worst->slack) {
        worst = &a;
      }
    }
    if (kept.empty() && worst != nullptr) {
      kept.push_back(*worst);
    }
    return kept;
  };
  std::vector<RawAnchor> fanins = select(raw_fanins);
  std::vector<RawAnchor> fanouts = select(raw_fanouts);

  // Criticality weight: (slack_threshold - slack) grows with negative slack; a
  // floor proportional to the slack window keeps every included anchor's weight
  // positive and collapses to uniform weights when all slacks are equal (so the
  // weighted driver/sink centroids degrade gracefully to plain centroids).
  double s_min = slack_threshold;
  for (const RawAnchor& a : fanins) {
    s_min = std::min(s_min, a.slack);
  }
  for (const RawAnchor& a : fanouts) {
    s_min = std::min(s_min, a.slack);
  }
  const double window = std::max(slack_threshold - s_min, 0.0);
  const double floor = 0.25 * std::max(window, 1e-12);

  auto append = [&](const std::vector<RawAnchor>& kept) {
    for (const RawAnchor& a : kept) {
      const double weight = std::max(slack_threshold - a.slack, 0.0) + floor;
      anchors.push_back({a.loc, weight, a.res, a.cap, a.is_fanout});
    }
  };
  append(fanins);
  append(fanouts);

  return !anchors.empty();
}

bool RelocateGenerator::computeBestLocation(const Target& target,
                                            sta::Pin* drvr_pin,
                                            sta::Instance* drvr_inst,
                                            odb::Point& result) const
{
  std::vector<RelocateAnchor> anchors;
  double c_load_total = 0.0;
  if (!collectAnchors(target, drvr_pin, drvr_inst, anchors, c_load_total)) {
    // No usable anchors: fall back to the plain path midpoint.
    return computeCriticalPathLocation(target, drvr_pin, result);
  }

  const sta::Scene* scene = target.activeScene(resizer_);
  double wire_res = 0.0;  // ohms/meter
  double wire_cap = 0.0;  // farads/meter
  resizer_.estimateParasitics()->wireSignalRC(scene, wire_res, wire_cap);

  // This gate's drive resistance and the weighted upstream driver resistance /
  // input pin cap feed the RC shift closed form.
  const double r_g = resizer_.driveResistance(drvr_pin);

  // Two candidate locations, computed per axis (Manhattan wirelength separates
  // x and y so each axis is optimized independently):
  //   0: plain midpoint of the criticality-weighted driver / sink centroids.
  //   1: the RC/criticality-shifted point.
  // Both are "central" points on the worst-path corridor; the weighted median
  // and the driver/sink brackets are deliberately not used -- their per-axis
  // choice is an anchor coordinate, which combines across axes into a
  // geometrically poor 2-D point that scores well under the local Elmore model
  // but hurts real timing.
  constexpr int kRcShift = 1;
  constexpr int kNumCandidates = 2;
  odb::Point candidates_axis[2][kNumCandidates];
  for (int axis = 0; axis < 2; ++axis) {
    std::vector<std::pair<double, double>> fanin;  // driver side (coord,weight)
    std::vector<std::pair<double, double>> fanout;  // sink side (coord,weight)
    double fanin_w = 0.0;
    double fanin_r_num = 0.0;
    double fanin_c_num = 0.0;
    int coord_min = axisCoord(anchors.front().loc, axis);
    int coord_max = coord_min;
    for (const RelocateAnchor& a : anchors) {
      const double coord = axisCoord(a.loc, axis);
      coord_min = std::min<int>(coord_min, static_cast<int>(coord));
      coord_max = std::max<int>(coord_max, static_cast<int>(coord));
      if (a.is_fanout) {
        fanout.emplace_back(coord, a.weight);
      } else {
        fanin.emplace_back(coord, a.weight);
        fanin_w += a.weight;
        fanin_r_num += a.weight * a.res;
        fanin_c_num += a.weight * a.cap;
      }
    }

    const double center = (coord_min + coord_max) / 2.0;
    const double a_coord = weightedMean(fanin, center);   // driver centroid A
    const double b_coord = weightedMean(fanout, center);  // sink centroid B
    const double midpoint = (a_coord + b_coord) / 2.0;

    // RC/criticality-shifted midpoint:
    //   p* = (A + B)/2 + (R_g - R_up)/(2r) + (C_load - C_pin)/(2c)
    // The gate slides toward the sinks when it is the weaker driver
    // (R_g > R_up) and/or drives the heavier load (C_load > C_pin), and toward
    // the upstream driver otherwise.  The shift is a signed length applied
    // along the driver -> sink direction on this axis.
    const double r_up = fanin_w > 0.0 ? fanin_r_num / fanin_w : r_g;
    const double c_pin = fanin_w > 0.0 ? fanin_c_num / fanin_w : c_load_total;
    double shift_m = 0.0;
    if (wire_res > 0.0) {
      shift_m += (r_g - r_up) / (2.0 * wire_res);
    }
    if (wire_cap > 0.0) {
      shift_m += (c_load_total - c_pin) / (2.0 * wire_cap);
    }
    // Cap the shift to half the driver-sink span so the RC-shifted point stays
    // interior (a genuine shift toward the sink/driver, never a jump onto an
    // endpoint pin, which the local Elmore model over-favors but real timing
    // punishes).
    const double half_span = std::abs(b_coord - a_coord) / 2.0;
    double shift_dbu = resizer_.metersToDbu(shift_m);
    shift_dbu = std::clamp(shift_dbu, -half_span, half_span);
    const double dir = (b_coord > a_coord) ? 1.0 : -1.0;
    const double rc_shift = midpoint + dir * shift_dbu;

    auto clamp = [&](double v) {
      return std::clamp(
          v, static_cast<double>(coord_min), static_cast<double>(coord_max));
    };
    const double picks[kNumCandidates] = {midpoint, rc_shift};
    for (int i = 0; i < kNumCandidates; ++i) {
      const int coord = static_cast<int>(std::lround(clamp(picks[i])));
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
  if (!computeBestLocation(target, drvr_pin, drvr_inst, new_loc)) {
    return candidates;
  }

  candidates.push_back(std::make_unique<RelocateCandidate>(
      resizer_, target, drvr_pin, drvr_inst, db_inst, new_loc));
  return candidates;
}

}  // namespace rsz
