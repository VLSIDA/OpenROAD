// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "RelocateGenerator.hh"

#include <memory>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveCommitter.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "RelocateCandidate.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "odb/geom.h"
#include "rsz/Resizer.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "utl/Logger.h"

namespace rsz {

using utl::RSZ;

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
  if (!computeCriticalPathLocation(target, drvr_pin, new_loc)) {
    return candidates;
  }

  candidates.push_back(std::make_unique<RelocateCandidate>(
      resizer_, target, drvr_pin, drvr_inst, db_inst, new_loc));
  return candidates;
}

}  // namespace rsz
