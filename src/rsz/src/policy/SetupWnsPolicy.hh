// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include "MoveCommitter.hh"
#include "OptimizerTypes.hh"
#include "RepairSetupContext.hh"
#include "RepairTargetCollector.hh"
#include "SetupLegacyBase.hh"

namespace rsz {

class SetupWnsPolicy : public SetupLegacyBase
{
 public:
  SetupWnsPolicy(Resizer& resizer,
                 MoveCommitter& committer,
                 RepairSetupContext& setup_context,
                 const OptimizerRunConfig& config,
                 bool use_cone,
                 ConeDirection cone_direction = ConeDirection::kFanin)
      : SetupLegacyBase(resizer, committer, setup_context, config),
        use_cone_(use_cone),
        cone_direction_(cone_direction)
  {
  }

  const char* name() const override { return "SetupWnsPolicy"; }
  void iterate() override;

 private:
  void repairSetupWns(float setup_slack_margin,
                      int max_passes_per_endpoint,
                      int max_repairs_per_pass,
                      bool verbose,
                      bool use_cone_collection,
                      rsz::ViolatorSortType sort_type);

  // Phase name for progress and debug reporting.
  const char* phaseName() const;

  bool use_cone_{false};
  ConeDirection cone_direction_{ConeDirection::kFanin};
};

}  // namespace rsz
