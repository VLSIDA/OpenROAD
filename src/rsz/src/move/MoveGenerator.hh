// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace sta {
class Instance;
class LibertyCell;
class LibertyPort;
class MinMax;
class Net;
class Pin;
class Vertex;
}  // namespace sta

namespace rsz {

class Resizer;
class MoveCommitter;

// === Generator input bundles ===============================================

// Deliver necessary information from policy to MoveGenerator.
//
// Passing references through GeneratorContext replaces the pre-refactor
// pattern where each generator held a direct pointer to Resizer and reached
// into global state for options.  This keeps unit-level coupling explicit:
//   - resizer       : the shared rsz::Resizer (OpenDB + STA access)
//   - committer     : the run-scoped MoveCommitter (journal + accounting)
//   - run_config    : frozen user options (skip flags, margins, sequence)
//   - policy_config : policy-selected tunables (max_candidate_generation, ...)
// All four references outlive every generator, so generators may store
// references directly and do not need to copy or own any of these.
struct GeneratorContext
{
  Resizer& resizer;
  MoveCommitter& committer;  // For move conflict check in pending moves.
  const OptimizerRunConfig& run_config;
  const OptimizationPolicyConfig& policy_config;
};

// === Move generator interface ==============================================

// Base class for one move type (Buffer, Clone, SizeUp, VtSwap, ...).
//
// Responsibilities:
//   1. prepareRequirements() -- declare which per-target PrepareCacheMask bits
//      this type reads.  The policy computes flags per target so
//      OptimizationPolicy only prepares data for generators that can consume
//      that target.  The default says "I need nothing"; override in types that
//      read prepared Target fields.
//   2. isApplicable() -- caller-facing target filter.  The base method checks
//      requiredViews(); derived generators call it first, then add
//      move-specific legality checks.
//   3. generate() -- expand one Target into zero or more concrete
//      MoveCandidate objects.  Candidates are owned by unique_ptr and
//      returned up to the policy which decides which to estimate/commit.
//
// Thread-safety: generate()/isApplicable() run on worker threads for MT
// policies.  Implementations should read prepared Target fields and avoid STA
// analysis state that requires the main thread.
class MoveGenerator
{
 public:
  // === Construction and identity ===========================================
  explicit MoveGenerator(const GeneratorContext& context)
      : resizer_(context.resizer),
        committer_(context.committer),
        run_config_(context.run_config),
        policy_config_(context.policy_config)
  {
  }
  virtual ~MoveGenerator() = default;

  virtual MoveType type() const = 0;
  virtual const char* name() const { return moveName(type()); }

  // === Target preparation and generation ===================================
  virtual bool isApplicable(const Target& target) const
  {
    const TargetViewMask views = requiredViews();
    return ((views & kPathDriverView) != 0 && target.canBePathDriver())
           || ((views & kInstanceView) != 0 && target.canBeInstance());
  }

  virtual PrepareCacheMask prepareRequirements() const
  {
    return kNoPrepareCache;
  }
  virtual std::vector<std::unique_ptr<MoveCandidate>> generate(
      const Target& target)
      = 0;

 protected:
  // === Target-view requirements ============================================

  // Since kPathDriver is the most common, set it as the default required view.
  // If a derived MoveGenerator requires other views, override this.
  virtual TargetViewMask requiredViews() const { return kPathDriverView; }

  // === Shared Liberty-cell ordering helpers ================================
  const sta::LibertyPort* findScenePort(const sta::LibertyCell* cell,
                                        const std::string& port_name,
                                        int lib_ap) const;
  bool strongerCellLess(const sta::LibertyCell* lhs,
                        const sta::LibertyCell* rhs,
                        const std::string& drvr_port_name,
                        int lib_ap) const;

  // === Optional neighbor feasibility check (-neighbor_check) ===============
  // All of the following are called ONLY when run_config_.neighbor_check is
  // set; the default path collects no neighbor data and is unchanged.

  bool neighborCheckEnabled() const { return run_config_.neighbor_check; }

  // Veto decision: WNS-degradation rule (wnsDegraded in OptimizerTypes.hh).
  // The local region is the on-path target -- {slack, slack + gain} under
  // the frozen-boundary assumption -- plus one {slack_before,
  // slack_before - delay_delta} entry per perturbed neighbor.  Reject when a
  // neighbor becomes the region's governing worst slack and degrades it;
  // when the on-path pin governs, the ordinary accept machinery decides.
  bool neighborCheckVeto(float on_path_slack,
                         float gain,
                         const std::vector<NeighborImpact>& impacts) const;

  // Snapshot a vertex's slack ONLY if it is a pure cache read (arrivals valid
  // and requireds already computed).  Returns false otherwise -- so a
  // neighbor read never triggers a findRequired()/findAllArrivals() recompute
  // (which would both free the repair loop's cached Paths -- a
  // use-after-free -- and cost a full timing pass during generation).
  bool cachedSlack(sta::Vertex* vertex, float& slack_out) const;

  // Drive resistance of the cell driving `net` (0 if none/undriven).
  float netDriveResistance(const sta::Net* net) const;

  // Lumped-RC delay change of a driver with the given drive resistance when
  // its load changes by delta_cap (positive => the driver slows).
  static float driverDelayDelta(float drive_resistance, float delta_cap);

  // Impacts on the fanin nets of `inst` when its input-pin loads grow.
  // Resize: pass old_cell and new_cell (per-pin delta = new - old).  Added
  // duplicate (clone): pass old_cell == nullptr (delta = new_cell's full
  // input cap).  skip_pin excludes the ON-PATH input pin: its slack already
  // equals the endpoint being repaired, so charging it would self-veto every
  // move, and the on-path stage cost is part of the move's own gain.
  std::vector<NeighborImpact> faninSlowdownImpacts(
      sta::Instance* inst,
      const sta::LibertyCell* old_cell,
      const sta::LibertyCell* new_cell,
      const sta::MinMax* min_max,
      const sta::Pin* skip_pin = nullptr) const;

  // Predicted arrival improvement of swapping the target's cell to
  // `candidate_cell`, via a DelayEstimator context spanning the target stage
  // plus one fanin/fanout stage (so the gain is net of the extra fanin load).
  // Returns false when no context/estimate is available.
  bool estimatedSwapGain(const Target& target,
                         const sta::LibertyCell* candidate_cell,
                         float& gain_out) const;

  // === Shared generator dependencies =======================================
  Resizer& resizer_;
  MoveCommitter& committer_;
  const OptimizerRunConfig& run_config_;
  const OptimizationPolicyConfig& policy_config_;
};

}  // namespace rsz
