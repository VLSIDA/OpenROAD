// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "BaseMove.hh"
#include "ViolatorCollector.hh"
#include "dbBlock.h"
#include "dbJournal.h"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "utl/Logger.h"

namespace rsz {

using odb::_dbBlock;
using odb::dbJournal;
using sta::Sta;
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
    sta_ = resizer_->sta_;
    network_ = resizer_->network_;
    violator_collector_ = violator_collector;
  }

  ~RepairSimulator() { clear(); }

  void init(const Pin* endpoint,
            const std::vector<const Pin*>& pins,
            const std::vector<BaseMove*>& moves,
            int level,
            float setup_slack_margin);
  void clear();
  void simulate();
  std::pair<const Pin*, BaseMove*> getBestImmediateMove();
  void commitMove(const Pin* pin, BaseMove* move);

  const Pin* getEndpoint() { return endpoint_; }

 private:
  class SimulationTreeNode
  {
   public:
    SimulationTreeNode(Resizer* resizer, const Pin* pin, BaseMove* move, int level)
        : pin_(pin), move_(move), level_(level)
    {
      std::string pin_name = pin ? resizer->network_->pathName(pin) : "null";
      std::string move_name = move ? move->name() : "null";
      name_ = fmt::format("SimNode({}, {}, L{})", pin_name, move_name, level);
    }

    ~SimulationTreeNode()
    {
      delete eco_;
      for (auto* child : children_) {
        delete child;
      }
    }

    const Pin* pin_;
    BaseMove* move_;
    int level_;
    std::string name_;
    bool is_simulated_{false};
    bool odb_eco_active_{false};
    Slack slack_{0.0};
    // Isolated ECO journal of this node (doesn't include other nodes' ECOs)
    dbJournal* eco_{nullptr};
    std::map<Instance*, int> tracked_changes_;
    std::vector<SimulationTreeNode*> children_;
  };

  bool doMove(SimulationTreeNode* node);
  void undoMove(SimulationTreeNode* node);
  void simulateDFS(SimulationTreeNode* node);
  SimulationTreeNode* getBestPossibleNodeDFS(SimulationTreeNode* node);
  void decrementLevel(SimulationTreeNode* node);

  // Prevents use-after-free error
  void addDestroyedPin(const Pin* pin, BaseMove* move);
  void removeDestroyedPin(const Pin* pin, BaseMove* move);
  bool isPinDestroyed(const Pin* pin);

  // Skips already rejected moves
  void addRejectedMove(const Pin* pin, BaseMove* move);
  bool isMoveRejected(const Pin* pin, BaseMove* move);

  const Pin* endpoint_;
  const std::vector<const Pin*>* pins_;
  const std::vector<BaseMove*>* moves_;
  int max_level_;
  float setup_slack_margin_;

  SimulationTreeNode* root_{nullptr};
  std::unordered_set<const Pin*> destroyed_pins_;
  std::map<const Pin*, std::unordered_set<BaseMove*>> rejected_moves_;

  Resizer* resizer_;
  Logger* logger_;
  Sta* sta_;
  Network* network_;
  ViolatorCollector* violator_collector_;
};

}  // namespace rsz
