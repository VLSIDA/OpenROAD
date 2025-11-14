// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "BaseMove.hh"
#include "ViolatorCollector.hh"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/Sta.hh"
#include "utl/Logger.h"

namespace rsz {

using sta::Network;
using sta::Pin;
using sta::Slack;
using sta::Vertex;
using sta::VertexSet;
using std::pair;
using std::set;
using std::vector;
using utl::RSZ;

// Class to simulate doing repair setup move combinations
class RepairSimulator
{
 public:
  // Constructor
  RepairSimulator(Resizer* resizer, ViolatorCollector* violator_collector)
      : resizer_(resizer)
  {
    logger_ = resizer_->logger_;
    network_ = resizer_->network_;
    violator_collector_ = violator_collector;
  }

  void init(float slack_margin);
  std::pair<const Pin*, BaseMove*> simulate(const Pin* endpoint,
                                            const std::vector<const Pin*>& pins,
                                            const std::vector<BaseMove*>& moves,
                                            int depth,
                                            float setup_slack_margin);

 private:
  class SimulationTreeNode
  {
   public:
    SimulationTreeNode(const Pin* pin, BaseMove* move, int depth)
        : pin_(pin), move_(move), depth_(depth), slack_(0.0)
    {
    }

    ~SimulationTreeNode()
    {
      for (auto* child : children_) {
        delete child;
      }
    }

    const Pin* pin_;
    BaseMove* move_;
    int depth_;
    Slack slack_;
    std::vector<SimulationTreeNode*> children_;
  };

  void simulateDFS(const Pin* endpoint,
                   SimulationTreeNode* node,
                   const std::vector<const Pin*>& pins,
                   const std::vector<BaseMove*>& moves,
                   int max_depth,
                   float setup_slack_margin);
  void trickleUpBestSlack(SimulationTreeNode* node);
  Resizer* resizer_;
  Logger* logger_;
  Network* network_;
  ViolatorCollector* violator_collector_;
};

}  // namespace rsz
