// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <vector>

#include "MoveCandidate.hh"
#include "MoveGenerator.hh"
#include "OptimizerTypes.hh"
#include "odb/geom.h"
#include "rsz/Resizer.hh"

namespace sta {
class Instance;
class Pin;
}  // namespace sta

namespace rsz {

// Relocates the target gate along its critical path.
//
// The framework hands this generator the standard path-driver critical
// candidate (the same driver pin/path CloneGenerator and SizeUpGenerator
// receive).  The generator's only logic is to place that gate at the midpoint
// between where its critical signal comes FROM (the driver of its most-critical
// input) and where its critical output goes TO (the load on the worst path
// through it), shortening the wire on that path.  A single RelocateCandidate is
// produced per target.  Single-threaded (reads live pin locations and expands
// the timing path on the main thread).
class RelocateGenerator : public MoveGenerator
{
 public:
  explicit RelocateGenerator(const GeneratorContext& context);

  MoveType type() const override { return MoveType::kRelocate; }
  std::vector<std::unique_ptr<MoveCandidate>> generate(
      const Target& target) override;

 private:
  // Resolve the movable standard-cell driver for this target.  Returns false
  // (with a debug trace) when the target is a don't-touch, non-logic, fixed, or
  // otherwise unmovable instance.
  bool resolveDriver(const Target& target,
                     sta::Pin*& drvr_pin,
                     sta::Instance*& drvr_inst,
                     odb::dbInst*& db_inst) const;

  // Midpoint of the most-critical input driver's location and the critical
  // output-load location, both read from the timing path the framework
  // provides.  Returns false when either endpoint cannot be located.
  bool computeCriticalPathLocation(const Target& target,
                                   sta::Pin* drvr_pin,
                                   odb::Point& result) const;
};

}  // namespace rsz
