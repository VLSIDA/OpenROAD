// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "MoveGenerator.hh"

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "DelayEstimator.hh"
#include "SubgraphTimer.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/Search.hh"
#include "utl/Logger.h"

namespace rsz {

bool MoveGenerator::neighborCheckVeto(
    const float on_path_slack,
    const float gain,
    const std::vector<NeighborImpact>& impacts) const
{
  if (impacts.empty()) {
    return false;
  }
  const LocalSlack on_path{on_path_slack, on_path_slack + gain};
  std::vector<LocalSlack> neighbors;
  neighbors.reserve(impacts.size());
  for (const NeighborImpact& impact : impacts) {
    neighbors.push_back(
        {impact.slack_before, impact.slack_before - impact.delay_delta});
  }
  return wnsDegraded(on_path, neighbors);
}

bool MoveGenerator::cachedSlack(sta::Vertex* vertex, float& slack_out) const
{
  if (vertex == nullptr) {
    return false;
  }
  sta::Search* search = resizer_.sta()->search();
  if (!search->arrivalsValid() || !search->requiredsExist()) {
    return false;
  }
  slack_out = sta::delayAsFloat(
      resizer_.sta()->slack(vertex, resizer_.maxAnalysisMode()));
  return true;
}

float MoveGenerator::netDriveResistance(const sta::Net* net) const
{
  if (net == nullptr) {
    return 0.0f;
  }
  sta::Network* network = resizer_.network();
  std::unique_ptr<sta::NetPinIterator> pin_iter(network->pinIterator(net));
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    if (network->direction(pin)->isOutput()) {
      const sta::LibertyPort* port = network->libertyPort(pin);
      return port != nullptr ? port->driveResistance() : 0.0f;
    }
  }
  return 0.0f;
}

float MoveGenerator::driverDelayDelta(const float drive_resistance,
                                      const float delta_cap)
{
  return drive_resistance * delta_cap;
}

std::vector<NeighborImpact> MoveGenerator::faninSlowdownImpacts(
    sta::Instance* inst,
    const sta::LibertyCell* old_cell,
    const sta::LibertyCell* new_cell,
    const sta::MinMax* min_max,
    const sta::Pin* skip_pin) const
{
  std::vector<NeighborImpact> impacts;
  if (inst == nullptr || new_cell == nullptr) {
    return impacts;
  }
  sta::Network* network = resizer_.network();
  std::unique_ptr<sta::InstancePinIterator> pin_iter(
      network->pinIterator(inst));
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    if (!network->direction(pin)->isInput()) {
      continue;
    }
    if (pin == skip_pin) {
      continue;  // on-path input: its cost is part of the move's own gain
    }
    const sta::LibertyPort* in_port = network->libertyPort(pin);
    if (in_port == nullptr) {
      continue;
    }
    const std::string port_name = in_port->name();
    const sta::LibertyPort* new_port = new_cell->findLibertyPort(port_name);
    if (new_port == nullptr) {
      continue;
    }
    const float new_cap = new_port->capacitance(min_max);
    float old_cap = 0.0f;
    if (old_cell != nullptr) {
      const sta::LibertyPort* old_port = old_cell->findLibertyPort(port_name);
      old_cap = old_port != nullptr ? old_port->capacitance(min_max) : 0.0f;
    }
    const float delta_cap = new_cap - old_cap;
    if (delta_cap <= 0.0f) {
      continue;  // load did not grow -> this fanin does not slow down
    }
    const float drive_res = netDriveResistance(network->net(pin));
    if (drive_res <= 0.0f) {
      continue;
    }
    sta::Vertex* load_vertex = resizer_.graph()->pinLoadVertex(pin);
    float slack_before = 0.0f;
    if (!cachedSlack(load_vertex, slack_before)) {
      continue;  // not readable without a recompute: omit (treat as no harm)
    }
    impacts.push_back({slack_before, driverDelayDelta(drive_res, delta_cap)});
  }
  return impacts;
}

namespace {

using SubgraphEvalFn = std::function<
    bool(SubgraphTimer&, LocalSlack&, std::vector<LocalSlack>&)>;

}  // namespace

bool MoveGenerator::subgraphVeto(const Target& target,
                                 sta::Instance* inst,
                                 sta::Pin* drvr_pin,
                                 const char* what,
                                 const RegionSpec& spec,
                                 const SubgraphEvalFn& eval) const
{
  if (!policy_config_.generate_on_main_thread) {
    return false;  // worker-thread generation: permissive skip
  }
  SubgraphTimer timer(resizer_);
  if (!timer.build(target, inst, drvr_pin, spec)) {
    return false;
  }
  LocalSlack on_path;
  std::vector<LocalSlack> neighbors;
  if (!eval(timer, on_path, neighbors)) {
    return false;
  }
  const bool veto = wnsDegraded(on_path, neighbors);
  float neighbor_wns_before = std::numeric_limits<float>::max();
  float neighbor_wns_after = std::numeric_limits<float>::max();
  for (const LocalSlack& n : neighbors) {
    neighbor_wns_before = std::min(neighbor_wns_before, n.before);
    neighbor_wns_after = std::min(neighbor_wns_after, n.after);
  }
  debugPrint(resizer_.logger(),
             utl::RSZ,
             "neighbor_subgraph",
             veto ? 1 : 2,
             "{} {} {}: on-path {:.3e}->{:.3e}, {} neighbors "
             "(wns {:.3e}->{:.3e})",
             veto ? "VETO" : "eval",
             what,
             resizer_.network()->pathName(drvr_pin),
             on_path.before,
             on_path.after,
             neighbors.size(),
             neighbor_wns_before,
             neighbor_wns_after);
  return veto;
}

bool MoveGenerator::subgraphCellSwapVeto(
    const Target& target,
    sta::Instance* inst,
    sta::Pin* drvr_pin,
    const sta::LibertyCell* candidate) const
{
  return subgraphVeto(target,
                      inst,
                      drvr_pin,
                      "cell-swap",
                      RegionSpec{},
                      [candidate](SubgraphTimer& timer,
                                  LocalSlack& on_path,
                                  std::vector<LocalSlack>& neighbors) {
                        return timer.evaluateCellSwap(
                            candidate, on_path, neighbors);
                      });
}

bool MoveGenerator::subgraphPinSwapVeto(const Target& target,
                                        sta::Instance* inst,
                                        sta::Pin* drvr_pin,
                                        const sta::LibertyPort* input_port,
                                        const sta::LibertyPort* swap_port) const
{
  return subgraphVeto(
      target,
      inst,
      drvr_pin,
      "pin-swap",
      RegionSpec{},
      [input_port, swap_port](SubgraphTimer& timer,
                              LocalSlack& on_path,
                              std::vector<LocalSlack>& neighbors) {
        return timer.evaluatePinSwap(input_port, swap_port, on_path, neighbors);
      });
}

bool MoveGenerator::subgraphSplitLoadVeto(const Target& target,
                                          sta::Instance* inst,
                                          sta::Pin* drvr_pin,
                                          const sta::LibertyCell* buffer_cell,
                                          const sta::PinSet& moved_loads) const
{
  return subgraphVeto(
      target,
      inst,
      drvr_pin,
      "split-load",
      RegionSpec{},
      [buffer_cell, &moved_loads](SubgraphTimer& timer,
                                  LocalSlack& on_path,
                                  std::vector<LocalSlack>& neighbors) {
        return timer.evaluateSplitLoad(
            buffer_cell, moved_loads, on_path, neighbors);
      });
}

bool MoveGenerator::subgraphCloneVeto(
    const Target& target,
    sta::Instance* inst,
    sta::Pin* drvr_pin,
    const sta::LibertyCell* clone_cell,
    const std::vector<sta::Pin*>& moved_loads) const
{
  return subgraphVeto(
      target,
      inst,
      drvr_pin,
      "clone",
      // Fanouts of the fanins: the doubled fanin load slows the fanin
      // drivers' OTHER receivers; promote them to full stages so the harm
      // propagates to their loads instead of stopping at their inputs.
      RegionSpec{.fanin_levels = 1,
                 .fanout_levels = 1,
                 .expand_center_fanouts = false,
                 .expand_fanin_fanouts = true},
      [clone_cell, &moved_loads](SubgraphTimer& timer,
                                 LocalSlack& on_path,
                                 std::vector<LocalSlack>& neighbors) {
        return timer.evaluateClone(clone_cell, moved_loads, on_path, neighbors);
      });
}

bool MoveGenerator::estimatedSwapGain(const Target& target,
                                      const sta::LibertyCell* candidate_cell,
                                      float& gain_out) const
{
  const std::optional<ArcDelayState> context
      = DelayEstimator::buildContext(resizer_, target, /*delay_levels=*/1);
  if (!context.has_value()) {
    return false;
  }
  const DelayEstimate delay_est
      = DelayEstimator::estimate(context.value(), candidate_cell);
  if (!delay_est.legal) {
    return false;
  }
  gain_out = delay_est.arrival_impr;
  return true;
}

const sta::LibertyPort* MoveGenerator::findScenePort(
    const sta::LibertyCell* cell,
    const std::string& port_name,
    const int lib_ap) const
{
  if (cell == nullptr) {
    return nullptr;
  }

  const sta::LibertyPort* port = cell->findLibertyPort(port_name);
  return port != nullptr ? port->scenePort(lib_ap) : nullptr;
}

bool MoveGenerator::strongerCellLess(const sta::LibertyCell* lhs,
                                     const sta::LibertyCell* rhs,
                                     const std::string& drvr_port_name,
                                     const int lib_ap) const
{
  const sta::LibertyPort* lhs_port = findScenePort(lhs, drvr_port_name, lib_ap);
  const sta::LibertyPort* rhs_port = findScenePort(rhs, drvr_port_name, lib_ap);
  if ((lhs_port != nullptr) != (rhs_port != nullptr)) {
    return lhs_port != nullptr;
  }
  if (lhs_port == nullptr) {
    return lhs->name() < rhs->name();
  }

  const float lhs_drive = lhs_port->driveResistance();
  const float rhs_drive = rhs_port->driveResistance();
  const sta::ArcDelay lhs_intrinsic
      = lhs_port->intrinsicDelay(resizer_.staState());
  const sta::ArcDelay rhs_intrinsic
      = rhs_port->intrinsicDelay(resizer_.staState());
  const float lhs_capacitance = lhs_port->capacitance();
  const float rhs_capacitance = rhs_port->capacitance();
  return std::tie(rhs_drive, lhs_intrinsic, lhs_capacitance)
         < std::tie(lhs_drive, rhs_intrinsic, rhs_capacitance);
}

}  // namespace rsz
