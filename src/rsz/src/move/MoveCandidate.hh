// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <vector>

#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace sta {
class LibertyCell;
}  // namespace sta

namespace rsz {

// === Move candidate interface ==============================================

// Base class for one concrete ECO proposal (swap this cell to LVT, insert a
// buffer at that pin, etc.).
//
// Lifecycle:
//   constructed by a MoveGenerator -> estimate() scores the move without
//   mutating design state -> policy compares estimates and picks a winner
//   -> MoveCommitter::commit(candidate) opens an ECO journal, calls apply()
//   once, and commits or rolls back depending on MoveResult.accepted.
//
// Separation of concerns:
//   estimate() must be pure w.r.t. OpenDB/STA so MT policies can call it
//   from worker threads using only prepared Target data.  apply() runs under
//   an open ECO journal on the main thread and is the only place that may
//   mutate the database or perform live STA checks that are not worker-safe.
//
class MoveCandidate
{
 public:
  // === Candidate lifecycle ==================================================
  virtual ~MoveCandidate() = default;

  // Provide a dummy implementation for legacy policy
  virtual Estimate estimate() { return {.legal = true, .score = 1.0f}; }
  virtual MoveResult apply() = 0;
  virtual MoveType type() const = 0;

  // The neighbor-cost guardband: the minimum net improvement (or extra slack
  // headroom) a move must leave to be accepted (env RSZ_SLACK_GUARDBAND,
  // default 0).  Public so generators that gate via their own slack estimate
  // (e.g. unbuffer's estimatedSlackOK) can apply the same guardband.
  static float slackGuardband();

 protected:
  MoveCandidate(Resizer& resizer, const Target& target);
  const Target& target() const { return target_; }
  MoveResult rejectedMove() const;

  // === Shared accept/reject decision (all moves) ============================
  //
  // The single, unified accept rule: a move is legal iff its predicted net
  // arrival improvement -- already net of neighbor cost -- clears the
  // guardband.  EVERY move funnels its prediction through here so the criterion
  // is identical across move classes; only the way each move predicts its net
  // improvement differs (cell-swap via the DelayEstimator, pin-swap via the
  // arc + fanin-cap model, topology via its own incremental model).
  // score = the predicted net improvement.
  Estimate acceptByImprovement(float delta_improvement) const;

  // Cell-swap prediction helper.  Builds a DelayEstimator context spanning the
  // target stage plus `delay_levels` fanin and fanout stages, scores swapping
  // in `candidate_cell`, and applies acceptByImprovement.  Because the context
  // includes the neighbor stages, the arrival improvement is already net of the
  // extra load this move places on its fanin drivers (or the drive it removes
  // downstream).  Never mutates the database or rebuilds the STA search, so it
  // is safe in the legacy repair loop (which caches Path pointers) and MT.
  Estimate estimatorEvaluate(const sta::LibertyCell* candidate_cell,
                             int delay_levels);

  // Default fanin/fanout depth captured by estimatorEvaluate.
  static constexpr int kDefaultDelayLevels = 1;

  // === Shared feasibility ("don't give up more than you gain") =============
  //
  // The improvement score answers "does this help the target?".  Feasibility
  // answers the orthogonal question "how much timing did we sacrifice on OTHER
  // neighbors to get it?".  A move can be improving yet a poor trade (e.g.
  // sizeUp helps its path but overloads its off-path fanin drivers; swapPins
  // speeds the critical pin but sinks the other pin's net).
  //
  // Each move's generator declares the neighbors it perturbs as NeighborImpact
  // {slack_before, delay_delta}.  Feasibility is a SCALE-FREE ratio: the
  // NEGATIVE slack the worst neighbor is pushed into, over the gain.  Only
  // slack driven below zero counts -- a neighbor with slack to spare absorbs
  // the delay and is not harmed -- so the "given up" per neighbor is
  //   max(0, delay_delta - max(0, slack_before)).
  // 10% means we newly violated by a tenth of what we gained (fine); 100% means
  // we gave back the whole gain (worthless).  A move is rejected when the ratio
  // exceeds RSZ_FEAS_RATIO.  A ratio is used precisely so no absolute,
  // per-design/tech threshold needs tuning.  Opt-in via RSZ_MOVE_FEASIBILITY.
  // slack_before is snapshotted in the generator, so estimate() does pure
  // arithmetic -- no live STA -- keeping it snapshot-pure and MT-safe.
  static bool neighborFeasibilityEnabled();
  static float feasibilityRatio();
  // Turn `estimate` infeasible when the feasibility gate is on and the neighbor
  // slack sacrificed exceeds feasibilityRatio() of the gain.  No-op when off.
  Estimate applyFeasibility(Estimate estimate,
                            const std::vector<NeighborImpact>& impacts) const;

  // === Candidate identity ===================================================
  Resizer& resizer_;
  const Target& target_;  // From the active poilicy target vector
};

}  // namespace rsz
