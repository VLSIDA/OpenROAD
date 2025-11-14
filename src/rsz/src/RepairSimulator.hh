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
  RepairSimulator(Resizer* resizer, ViolatorCollector* violator_collector)
      : resizer_(resizer)
  {
    logger_ = resizer_->logger_;
    network_ = resizer_->network_;
    violator_collector_ = violator_collector;
  }

  ~RepairSimulator() { clear(); }

  void init(const Pin* endpoint,
            std::vector<const Pin*>& pins,
            std::vector<BaseMove*>& moves,
            int depth,
            float setup_slack_margin);
  void clear();
  void simulate();
  std::pair<const Pin*, BaseMove*> getBestImmediateMove();
  void commitMove(const Pin* pin, const BaseMove* move);

 private:
  class SimulationTreeNode
  {
   public:
    SimulationTreeNode(const Pin* pin, BaseMove* move, int depth)
        : pin_(pin), move_(move), depth_(depth)
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
    Slack slack_{0.0};
    std::vector<SimulationTreeNode*> children_;
  };

  void simulateDFS(SimulationTreeNode* node);
  void trickleUpBestSlack(SimulationTreeNode* node);

  const Pin* endpoint_;
  const std::vector<const Pin*>* pins_;
  const std::vector<BaseMove*>* moves_;
  int max_depth_;
  float setup_slack_margin_;

  SimulationTreeNode* root_;

  Resizer* resizer_;
  Logger* logger_;
  Network* network_;
  ViolatorCollector* violator_collector_;
};

}  // namespace rsz
