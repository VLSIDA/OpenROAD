// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace odb {
class dbInst;
}  // namespace odb

namespace sta {
class Instance;
class Pin;
}  // namespace sta

namespace rsz {

// Moves one already-placed gate to a new location along its critical path.
//
// apply() sets the instance origin (journaled through odb ECO so the move is
// reversible), legalizes it to a site when a placement grid exists, and
// invalidates parasitics on every net the gate touches.  The move changes only
// placement -- no cell is resized, inserted, or removed -- so it leaves the
// design area unchanged.
class RelocateCandidate : public MoveCandidate
{
 public:
  RelocateCandidate(Resizer& resizer,
                    const Target& target,
                    sta::Pin* drvr_pin,
                    sta::Instance* drvr_inst,
                    odb::dbInst* db_inst,
                    const odb::Point& new_loc);

  MoveResult apply() override;
  MoveType type() const override { return MoveType::kRelocate; }

 private:
  // Invalidate the estimated parasitics of every signal net connected to
  // drvr_inst_.  Moving the gate changes each net's geometry, and no odb
  // move-inst callback recomputes them.
  void invalidateConnectedParasitics();

  sta::Pin* drvr_pin_;
  sta::Instance* drvr_inst_;
  odb::dbInst* db_inst_;
  odb::Point new_loc_;
};

}  // namespace rsz
