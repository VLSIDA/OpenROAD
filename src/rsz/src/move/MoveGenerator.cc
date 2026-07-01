// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "MoveGenerator.hh"

#include <string>
#include <tuple>

#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Path.hh"

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
