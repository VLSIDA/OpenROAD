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
// receive).  The gate has one input anchor -- the driver of its critical input
// net (location L_D, upstream drive resistance R_up, gate input pin cap C_pin)
// -- and a set of fanout sinks.  Only the CRITICAL (negative-slack) sinks
// anchor the gate; the non-critical loads are pruned so a sink's many
// non-critical siblings cannot dilute the placement off the critical corridor.
// (The whole fanout still contributes to the total output load C_load, which
// sets the gate-side drive-delay magnitude.)
//
// For each critical sink s_i the Elmore-ideal gate position on the
// L_D -> gate -> s_i path is, per axis (Manhattan wirelength separates x and y
// so each axis is optimized independently),
//
//   p_i = midpoint(L_D, L_i) + rc_shift   along the L_D -> L_i direction
//   rc_shift = (R_g - R_up)/(2r) + (C_load - C_pin)/(2c)
//
// where R_g is this gate's drive resistance and r/c the wire resistance/
// capacitance per unit length.  The gate slides toward the sinks when it is the
// weaker driver (R_g > R_up) and/or drives the heavier load (C_load > C_pin),
// and toward the upstream driver otherwise.  The final target is the
// criticality-weighted average P = (Sum_i w_i p_i) / (Sum_i w_i) with
// w_i = max(0, -slack_i).
//
// The generator scores the weighted-average point and the plain (unshifted)
// weighted midpoint with the criticality-weighted Elmore estimate and emits the
// cheaper as a RelocateCandidate.  When the gate has no critical sink it falls
// back to the plain two-point critical-path midpoint.  (The setup legacy policy
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

  // Collect the input-driver anchor and the critical (negative-slack) fanout
  // sink anchors for the target gate and their per-anchor RC data, plus the
  // total output load cap (used by the RC-shift closed form).  Returns false
  // when the gate has no critical sink (the caller then falls back to the plain
  // critical-path midpoint).
  bool collectAnchors(const Target& target,
                      sta::Pin* drvr_pin,
                      std::vector<RelocateAnchor>& anchors,
                      double& c_load_total) const;

  // Wire-delay accounting for one relocation candidate, used by the round-1
  // acceptance guards in generate().  All delays are in seconds.
  //   cur_wire_delay  : unweighted incident Elmore wire delay at the gate's
  //                     current location (sum over the input + critical-sink
  //                     anchors).
  //   best_wire_delay : the same, criticality-weighted, at the chosen location
  //                     (the value minimized by computeBestLocation); paired
  //                     with cur_wire_delay_weighted for the relative-gain
  //                     test.
  //   cur_wire_delay_weighted : criticality-weighted incident wire delay at the
  //                     current location.
  //   gate_delay      : the gate's own drive+intrinsic delay driving the whole
  //                     output load, the "cell" term for the wire-fraction
  //                     test.
  //   have_rc         : false when the plain-midpoint fallback was used (no RC
  //                     data); the guards are then skipped.
  struct RelocatePlan
  {
    double cur_wire_delay{0.0};
    double best_wire_delay{0.0};
    double cur_wire_delay_weighted{0.0};
    double gate_delay{0.0};
    bool have_rc{false};
  };

  // Compute the relocation target as the criticality-weighted average, over the
  // critical sinks, of each sink's Elmore-ideal gate position on the
  // driver -> gate -> sink path, and pick the cheaper of the plain-midpoint and
  // RC-shifted variants by the criticality-weighted Elmore wire delay.  Falls
  // back to the plain two-point midpoint when no critical sink exists.  Returns
  // false when no location can be computed.  Fills `plan` with the wire/gate
  // delay accounting used by the acceptance guards.
  bool computeBestLocation(const Target& target,
                           sta::Pin* drvr_pin,
                           odb::Point& result,
                           RelocatePlan& plan) const;

  // Single-cell star wirelength (Manhattan): sum over every signal net incident
  // to `db_inst` of the distance from `gate_loc` to each other pin on the net.
  // A move that increases this sum lengthens the moved cell's own nets and is
  // unlikely to survive routing, so generate() rejects such relocations even
  // when the local Elmore estimate improves (WNS-survival heuristic).
  double incidentStarWirelength(odb::dbInst* db_inst,
                                const odb::Point& gate_loc) const;

  // Plain midpoint of the most-critical input driver and the critical output
  // load, read from the timing path the framework provides.  Used as the
  // fallback when the richer anchor set is unavailable.  Returns false when
  // either endpoint cannot be located.
  bool computeCriticalPathLocation(const Target& target,
                                   sta::Pin* drvr_pin,
                                   odb::Point& result) const;
};

}  // namespace rsz
