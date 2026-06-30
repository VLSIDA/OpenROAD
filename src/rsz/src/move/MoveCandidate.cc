// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "MoveCandidate.hh"

#include <cstdlib>
#include <optional>

#include "DelayEstimator.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace rsz {

MoveCandidate::MoveCandidate(Resizer& resizer, const Target& target)
    : resizer_(resizer), target_(target)
{
}

MoveResult MoveCandidate::rejectedMove() const
{
  return {
      .accepted = false,
      .type = type(),
      .touched_instances = {},
  };
}

float MoveCandidate::slackGuardband()
{
  const char* env = std::getenv("RSZ_SLACK_GUARDBAND");
  if (env == nullptr) {
    return 0.0f;
  }
  return static_cast<float>(std::atof(env));
}

Estimate MoveCandidate::estimatorEvaluate(
    const sta::LibertyCell* candidate_cell,
    const int delay_levels,
    const float min_improvement)
{
  // Build a context spanning the target stage plus delay_levels fanin/fanout
  // stages so the estimate is net of neighbor cost.  Read-only w.r.t. STA, so
  // it does not dangle the Path pointers the legacy repair loop caches.
  FailReason fail_reason = FailReason::kNone;
  const std::optional<ArcDelayState> context = DelayEstimator::buildContext(
      resizer_, target_, delay_levels, &fail_reason);
  if (!context.has_value()) {
    return {.legal = false, .score = 0.0f};
  }

  const DelayEstimate delay_est
      = DelayEstimator::estimate(context.value(), candidate_cell);
  // Keep the raw score even for rejected moves so policy ranking stays
  // consistent.
  const bool legal
      = delay_est.legal && delay_est.arrival_impr > min_improvement;
  return {.legal = legal, .score = delay_est.arrival_impr};
}

}  // namespace rsz
