// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include <cmath>

#include "BaseMove.hh"

namespace rsz {

class SizeUpMove : public BaseMove
{
 public:
  using BaseMove::BaseMove;

  bool doMove(const Pin* drvr_pin, float setup_slack_margin) override;

  const char* name() override { return "SizeUpMove"; }

 private:
  LibertyCell* upsizeCell(LibertyPort* in_port,
                          LibertyPort* drvr_port,
                          float load_cap,
                          float prev_drive,
                          const DcalcAnalysisPt* dcalc_ap);
};

// Upsize cells to match drive strength with previous stage
class SizeUpMatchMove : public BaseMove
{
 public:
  using BaseMove::BaseMove;

  bool doMove(const Pin* drvr_pin, float setup_slack_margin) override;

  const char* name() override { return "SizeUpMoveMatch"; }
};

}  // namespace rsz
