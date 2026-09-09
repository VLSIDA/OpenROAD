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

// Relocates the target gate to a timing/RC-aware location on its critical
// path.
//
// The framework hands this generator the standard path-driver critical
// candidate (the same driver pin/path CloneGenerator and SizeUpGenerator
// receive).  Rather than the plain geometric midpoint of one input driver and
// one output load, the generator anchors the gate on ALL near-critical fanin
// drivers and ALL fanout sinks, each weighted by timing criticality, and uses
// an Elmore-RC estimate that accounts for drive-strength and load asymmetry.
//
// Per the Elmore model (Manhattan wirelength separates x and y so each axis is
// optimized independently) minimizing the criticality-weighted sum of wire
// delays over the gate position p shifts the plain midpoint toward the sinks
// when this gate is the weaker driver / more heavily loaded, and toward the
// upstream driver otherwise:
//
//   p* = (A + B)/2 + (R_g - R_up)/(2r) + (C_load - C_pin)/(2c)
//
// where A/B are the (weighted) driver/sink coordinates, R_up/R_g the upstream
// and this-gate drive resistances, C_pin/C_load the input pin and total output
// load caps, and r/c the wire resistance/capacitance per unit length.
//
// The generator computes a small set of candidate locations (RC-shifted point,
// criticality-weighted median, criticality-weighted mean, and the driver/sink
// brackets), scores each with the criticality-weighted Elmore estimate, and
// emits the single best as a RelocateCandidate.  (The setup legacy policy
// commits the first legal candidate rather than ranking by estimate(), so the
// best-location choice is made here; real timing then gates the move at the
// endpoint-pass journal level.)  Single-threaded (reads live pin locations and
// timing on the main thread).
class RelocateGenerator : public MoveGenerator
{
 public:
  explicit RelocateGenerator(const GeneratorContext& context);

  MoveType type() const override { return MoveType::kRelocate; }
  std::vector<std::unique_ptr<MoveCandidate>> generate(
      const Target& target) override;

 private:
  // One incident-net anchor pulling the gate toward its coordinate.
  //   loc       : pin location this anchor pulls toward.
  //   weight    : timing criticality weight (> 0 for included anchors).
  //   res       : driving resistance on this net segment (ohm) -- the upstream
  //               driver resistance R_up for a fanin, this gate's drive
  //               resistance R_g for a fanout.
  //   cap        : far-end lumped capacitance on this segment (F) -- this
  //               gate's input pin cap C_pin for a fanin, the sink pin cap for
  //               a fanout.
  //   is_fanout : true when the gate drives this anchor (output net), false
  //               when the anchor drives the gate (input net).
  struct RelocateAnchor
  {
    odb::Point loc;
    double weight;
    double res;
    double cap;
    bool is_fanout;
  };

  // Resolve the movable standard-cell driver for this target.  Returns false
  // (with a debug trace) when the target is a don't-touch, non-logic, fixed, or
  // otherwise unmovable instance.
  bool resolveDriver(const Target& target,
                     sta::Pin*& drvr_pin,
                     sta::Instance*& drvr_inst,
                     odb::dbInst*& db_inst) const;

  // Collect the near-critical fanin-driver and fanout-sink anchors for the
  // target gate and their per-anchor RC data, plus the total output load cap
  // (used by the RC-shift closed form).  Returns false when neither a usable
  // fanin nor fanout anchor can be located.
  bool collectAnchors(const Target& target,
                      sta::Pin* drvr_pin,
                      sta::Instance* drvr_inst,
                      std::vector<RelocateAnchor>& anchors,
                      double& c_load_total) const;

  // Rank a small candidate set (RC-shifted point, weighted median/mean, driver
  // and sink brackets, plain midpoint) by the criticality-weighted Elmore wire
  // delay and return the best location.  Falls back to the plain two-point
  // midpoint when anchors are insufficient.  Returns false when no location can
  // be computed.
  bool computeBestLocation(const Target& target,
                           sta::Pin* drvr_pin,
                           sta::Instance* drvr_inst,
                           odb::Point& result) const;

  // Plain midpoint of the most-critical input driver and the critical output
  // load, read from the timing path the framework provides.  Used as the
  // fallback when the richer anchor set is unavailable.  Returns false when
  // either endpoint cannot be located.
  bool computeCriticalPathLocation(const Target& target,
                                   sta::Pin* drvr_pin,
                                   odb::Point& result) const;
};

}  // namespace rsz
