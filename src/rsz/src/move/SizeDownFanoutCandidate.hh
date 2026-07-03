// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <utility>
#include <vector>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/NetworkClass.hh"

namespace sta {
class LibertyCell;
}  // namespace sta

namespace rsz {

// Candidate that replaces one non-critical load instance with a smaller cell.
//
// The generator pre-screens loads on the driver's net and picks a weaker
// cell that reduces area/leakage while staying within the available slack
// budget. apply() performs the cell swap via Resizer::replaceCell.
class SizeDownFanoutCandidate : public MoveCandidate
{
 public:
  // === Construction =========================================================
  SizeDownFanoutCandidate(Resizer& resizer,
                          const Target& target,
                          sta::Pin* drvr_pin,
                          sta::Instance* inst,
                          sta::Pin* load_pin,
                          sta::LibertyCell* current_cell,
                          sta::LibertyCell* replacement,
                          sta::Slack slack,
                          float delta_improvement,
                          std::vector<NeighborImpact> impacts = {});

  // === MoveCandidate API ====================================================
  // Shrinking a non-critical fanout load removes input cap from the critical
  // driver's net; the generator predicts the resulting driver-delay relief
  // (gain) and funnels it through the shared accept rule + soft veto.  The
  // perturbed neighbor is the shrunk gate itself: its own stage delay grows,
  // charged against its slack budget.  (The generator also pre-screens the swap
  // against that budget when selecting the cell.)
  Estimate estimate() override;
  MoveResult apply() override;
  MoveType type() const override { return MoveType::kSizeDownFanout; }

 private:
  // === Candidate state ======================================================
  sta::Pin* drvr_pin_{nullptr};
  sta::Instance* inst_{nullptr};
  sta::Pin* load_pin_{nullptr};
  sta::LibertyCell* current_cell_{nullptr};
  sta::LibertyCell* replacement_{nullptr};
  sta::Slack slack_{0.0};
  float delta_improvement_{0.0f};
  // The shrunk gate's own stage-delay increase vs its slack budget.
  std::vector<NeighborImpact> impacts_;
};

}  // namespace rsz
