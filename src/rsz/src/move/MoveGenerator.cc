// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "MoveGenerator.hh"

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Path.hh"
#include "sta/PortDirection.hh"

namespace rsz {

float MoveGenerator::driverDelayDelta(const float drive_resistance,
                                      const sta::LibertyPort* old_port,
                                      const sta::LibertyPort* new_port,
                                      const sta::MinMax* min_max)
{
  return drive_resistance
         * (new_port->capacitance(min_max) - old_port->capacitance(min_max));
}

float MoveGenerator::driverDelayDelta(const float drive_resistance,
                                      const float delta_cap)
{
  return drive_resistance * delta_cap;
}

float MoveGenerator::driveResistanceAt(const sta::Path* driver_path) const
{
  if (driver_path == nullptr) {
    return 0.0f;
  }
  sta::Pin* pin = driver_path->pin(resizer_.staState());
  const sta::LibertyPort* port
      = pin != nullptr ? resizer_.network()->libertyPort(pin) : nullptr;
  return port != nullptr ? port->driveResistance() : 0.0f;
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
      continue;  // on-path input: evaluated by the score, would self-veto here
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
    if (load_vertex == nullptr) {
      continue;
    }
    // Snapshot the neighbor's current slack so the accept decision is pure.
    const float slack_before = sta::delayAsFloat(
        resizer_.sta()->slack(load_vertex, resizer_.maxAnalysisMode()));
    impacts.push_back({slack_before, driverDelayDelta(drive_res, delta_cap)});
  }
  return impacts;
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
