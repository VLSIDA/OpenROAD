// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include "BaseMove.hh"

namespace rsz {

using sta::ArcDelay;
using sta::Instance;
using sta::InstancePinIterator;
using sta::LoadPinIndexMap;
using sta::NetConnectedPinIterator;
using sta::Path;
using sta::PathExpanded;
using sta::Slack;
using sta::Slew;

class UnbufferMove : public BaseMove
{
 public:
  using BaseMove::BaseMove;
  ~UnbufferMove() override = default;

  bool doMove(const Pin* drvr_pin,
              float setup_slack_margin) override;

  const char* name() override { return "UnbufferMove"; }

  bool removeBufferIfPossible(Instance* buffer, bool honorDontTouchFixed);
  bool canRemoveBuffer(Instance* buffer, bool honorDontTouchFixed);
  void removeBuffer(Instance* buffer);

 private:
  bool bufferBetweenPorts(Instance* buffer);

  static constexpr int buffer_removal_max_fanout_ = 10;
};

}  // namespace rsz
