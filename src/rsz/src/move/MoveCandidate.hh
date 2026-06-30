// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

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

 protected:
  MoveCandidate(Resizer& resizer, const Target& target);
  const Target& target() const { return target_; }
  MoveResult rejectedMove() const;

  // === Shared accept/reject evaluation (cell-swap moves) ====================
  //
  // Consistent, neighbor-aware estimate shared by the cell-swap moves (size-up,
  // size-down-fanout, vt-swap).  It builds a DelayEstimator context that spans
  // the target stage plus `delay_levels` fanin and fanout stages, then scores
  // swapping in `candidate_cell`.  Because the context includes the neighbor
  // stages, the returned arrival improvement is already net of the extra load
  // this move places on its fanin drivers (or the drive it removes downstream),
  // so a move that helps the target but hurts a neighbor more is not legal.
  //
  // Unlike a live-STA measurement this never mutates the database or rebuilds
  // the STA search, so it is safe to call from the legacy repair loop (which
  // caches Path pointers) and from MT worker threads.
  //
  // The move is legal iff the estimator deems it valid AND the net arrival
  // improvement exceeds `min_improvement` (the guardband, so marginal moves
  // that leave no post-route headroom are rejected).  score = arrival
  // improvement.
  Estimate estimatorEvaluate(const sta::LibertyCell* candidate_cell,
                             int delay_levels,
                             float min_improvement);

  // Configured neighbor-cost guardband: the minimum net arrival improvement a
  // move must show to be accepted (env RSZ_SLACK_GUARDBAND, default 0).
  static float slackGuardband();

  // Default fanin/fanout depth captured by estimatorEvaluate.
  static constexpr int kDefaultDelayLevels = 1;

  // === Candidate identity ===================================================
  Resizer& resizer_;
  const Target& target_;  // From the active poilicy target vector
};

}  // namespace rsz
