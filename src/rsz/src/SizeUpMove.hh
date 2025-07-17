// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include <cmath>

#include "BaseMove.hh"

namespace rsz {

class SizeUpMove : public BaseMove
{
 public:
  using BaseMove::BaseMove;

  bool doMove(const Path* drvr_path,
              Slack drvr_slack,
              float setup_slack_margin) override;

  const char* name() override { return "SizeUpMove"; }
};

}  // namespace rsz
