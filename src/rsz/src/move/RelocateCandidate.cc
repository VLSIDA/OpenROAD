// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RelocateCandidate.hh"

#include <memory>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "db_sta/dbNetwork.hh"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

RelocateCandidate::RelocateCandidate(Resizer& resizer,
                                     const Target& target,
                                     sta::Pin* drvr_pin,
                                     sta::Instance* drvr_inst,
                                     odb::dbInst* db_inst,
                                     const odb::Point& new_loc)
    : MoveCandidate(resizer, target),
      drvr_pin_(drvr_pin),
      drvr_inst_(drvr_inst),
      db_inst_(db_inst),
      new_loc_(new_loc)
{
}

void RelocateCandidate::invalidateConnectedParasitics()
{
  sta::Network* network = resizer_.network();
  std::unique_ptr<sta::InstancePinIterator> pin_iter(
      network->pinIterator(drvr_inst_));
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    sta::Net* net = network->net(pin);
    if (net != nullptr && !network->isPower(net) && !network->isGround(net)) {
      resizer_.estimateParasitics()->parasiticsInvalid(net);
    }
  }
}

MoveResult RelocateCandidate::apply()
{
  int cur_x = 0;
  int cur_y = 0;
  db_inst_->getLocation(cur_x, cur_y);
  if (cur_x == new_loc_.getX() && cur_y == new_loc_.getY()) {
    // Already at the target location; nothing to do.
    debugPrint(resizer_.logger(),
               RSZ,
               "relocate_move",
               2,
               "REJECT RelocateMove {}: already at ({}, {})",
               resizer_.network()->pathName(drvr_pin_),
               cur_x,
               cur_y);
    return rejectedMove();
  }

  // Set the origin (clamped to the core).  odb journals the origin change so
  // the ECO framework can roll it back if timing does not improve.
  resizer_.setLocation(db_inst_, new_loc_);
  resizer_.legalizeCellPos(db_inst_);
  invalidateConnectedParasitics();

  debugPrint(resizer_.logger(),
             RSZ,
             "relocate_move",
             1,
             "ACCEPT RelocateMove {}: ({}, {}) -> ({}, {})",
             resizer_.network()->pathName(drvr_pin_),
             cur_x,
             cur_y,
             new_loc_.getX(),
             new_loc_.getY());
  return {
      .accepted = true,
      .type = MoveType::kRelocate,
      .move_count = 1,
      .touched_instances = {drvr_inst_},
  };
}

}  // namespace rsz
