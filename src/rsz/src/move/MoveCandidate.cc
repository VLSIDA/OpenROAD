// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "MoveCandidate.hh"

#include <algorithm>
#include <cstdlib>
#include <optional>
#include <vector>

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

Estimate MoveCandidate::acceptByImprovement(const float delta_improvement) const
{
  // The one shared criterion: clear the guardband.  score carries the raw
  // improvement so policy ranking stays consistent even for rejected moves.
  return {.legal = delta_improvement > slackGuardband(),
          .score = delta_improvement};
}

Estimate MoveCandidate::applyFeasibility(
    Estimate estimate,
    const std::vector<NeighborImpact>& impacts) const
{
  if (!estimate.legal || impacts.empty()) {
    return estimate;
  }
  // Scale-free soft veto: reject when the WORST single neighbor is pushed too
  // far into violation relative to the gain.  Only NEGATIVE slack counts -- a
  // neighbor with slack to spare absorbs the extra delay and is not harmed; the
  // "given up" is the part of delay_delta that drives its slack below zero:
  //   max(0, delay_delta - max(0, slack_before)).
  // Per-neighbor (not summed): each sits on a different path.  gain
  // (estimate.score) is the predicted improvement to the critical net.  All
  // terms are snapshots/analytic -- no live STA.
  const float gain = estimate.score;
  if (gain <= 0.0f) {
    return estimate;  // acceptByImprovement already required gain > guardband
  }
  float worst_given_up = 0.0f;
  for (const NeighborImpact& impact : impacts) {
    const float absorbed = std::max(0.0f, impact.slack_before);
    const float negative_harm = std::max(0.0f, impact.delay_delta - absorbed);
    worst_given_up = std::max(worst_given_up, negative_harm);
  }
  // Accept metric: NET slack delta, net = gain - lambda * worst_harm; veto
  // when net <= 0.  A difference (not a harm/gain ratio) so a tiny gain cannot
  // blow the metric up -- tiny gain + real harm is net-negative (correctly
  // rejected), tiny gain + tiny harm is ~neutral (kept).  Record the raw
  // (gain, harm) pair even when the veto is disabled so a veto-off run
  // captures the joint distribution for offline metric tuning.
  resizer_.recordMoveFeasibility(type(), gain, worst_given_up);
  const float net = gain - resizer_.feasibilityLambda(type()) * worst_given_up;
  estimate.feasibility_net = net;
  if (resizer_.moveFeasibilityVetoActive() && worst_given_up > 0.0f
      && net <= 0.0f) {
    estimate.legal = false;
    estimate.feasibility_vetoed = true;
  }
  return estimate;
}

Estimate MoveCandidate::estimatorEvaluate(
    const sta::LibertyCell* candidate_cell,
    const int delay_levels)
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
  if (!delay_est.legal) {
    return {.legal = false, .score = delay_est.arrival_impr};
  }
  return acceptByImprovement(delay_est.arrival_impr);
}

}  // namespace rsz
