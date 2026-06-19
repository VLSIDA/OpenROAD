// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "SetupLegacyPolicy.hh"

namespace rsz {

// Coarse / fine setup-repair phases for coarse->fine multi-pass experiments.
//
// Both reuse the legacy worst-path target selection (SetupLegacyPolicy) and
// only override the move sequence:
//   - COARSE: large area moves that need legalization (sizeup, clone, buffer,
//             split).
//   - FINE  : little/no area or legalization (vt_swap, pin swap, size_down,
//             unbuffer, reroute).
// Compose passes with e.g. `repair_timing -phases "COARSE FINE COARSE FINE"`.

class SetupCoarsePolicy : public SetupLegacyPolicy
{
 public:
  using SetupLegacyPolicy::SetupLegacyPolicy;

  const char* name() const override { return "SetupCoarsePolicy"; }

 protected:
  void buildMainMoveSequence(bool log_sequence) override;
  const char* phaseName() const override { return "COARSE"; }
  const char* phaseSummaryTitle() const override
  {
    return "COARSE Phase Summary";
  }
  const char* phaseEndpointProfilerTitle() const override
  {
    return "COARSE Phase Endpoint Profiler";
  }
};

class SetupFinePolicy : public SetupLegacyPolicy
{
 public:
  using SetupLegacyPolicy::SetupLegacyPolicy;

  const char* name() const override { return "SetupFinePolicy"; }

 protected:
  void buildMainMoveSequence(bool log_sequence) override;
  const char* phaseName() const override { return "FINE"; }
  const char* phaseSummaryTitle() const override
  {
    return "FINE Phase Summary";
  }
  const char* phaseEndpointProfilerTitle() const override
  {
    return "FINE Phase Endpoint Profiler";
  }
};

}  // namespace rsz
