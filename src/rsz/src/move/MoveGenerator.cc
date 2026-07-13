// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "MoveGenerator.hh"

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "DelayEstimator.hh"
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

namespace rsz {

bool MoveGenerator::neighborCheckVeto(
    const float gain,
    const std::vector<NeighborImpact>& impacts) const
{
  if (gain <= 0.0f || impacts.empty()) {
    return false;
  }
  float worst_given_up = 0.0f;
  for (const NeighborImpact& impact : impacts) {
    const float absorbed = std::max(0.0f, impact.slack_before);
    const float negative_harm = std::max(0.0f, impact.delay_delta - absorbed);
    worst_given_up = std::max(worst_given_up, negative_harm);
  }
  return worst_given_up > 0.0f
         && gain - run_config_.neighbor_check_lambda * worst_given_up <= 0.0f;
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
