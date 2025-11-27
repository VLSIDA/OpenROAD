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
using sta::Network;
using sta::Pin;
using sta::Slack;
using sta::Sta;
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
  enum class SearchMode
  {
    DFS,
    BFS,
    MCTS
  };

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
            SearchMode mode,
            int time_limit,
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
    SimulationTreeNode(Resizer* resizer,
                       const Pin* pin,
                       BaseMove* move,
                       int level)
        : pin_(pin), move_(move), level_(level)
    {
      pin_name_ = pin ? resizer->network_->pathName(pin) : "null";
      move_name_ = move ? move->name() : "null";
    }

    ~SimulationTreeNode()
    {
      delete eco_;
      for (auto* child : children_) {
        delete child;
      }
    }

    std::string name()
    {
      return fmt::format("SimNode({}, {}, L{})", pin_name_, move_name_, level_);
    }

    const Pin* pin_;
    BaseMove* move_;
    int level_;
    std::string pin_name_;
    std::string move_name_;
    bool simulation_started_{false};
    bool simulation_finished_{false};
    bool simulation_aborted_{false};
    bool odb_eco_active_{false};
    // Slack of path endpoint
    Slack slack_{0.0};
    // MCTS variables
    int mcts_visits_{0};
    Slack mcts_best_slack_{-sta::INF};
    // Isolated ECO journal of this node (doesn't include other nodes' ECOs)
    dbJournal* eco_{nullptr};
    // Track BaseMove::addMove() changes to properly revert them
    std::map<Instance*, int> tracked_changes_;
    // Children of this node
    std::vector<SimulationTreeNode*> children_;
    // Children of this node haven't been simulated yet
    std::queue<SimulationTreeNode*> children_pending_;
  };

  bool doMove(SimulationTreeNode* node);
  void undoMove(SimulationTreeNode* node);
  bool simulateBFS(SimulationTreeNode* node);
  bool simulateDFS(SimulationTreeNode* node);
  bool simulateMCTS(SimulationTreeNode* node);
  SimulationTreeNode* getBestPossibleNodeDFS(SimulationTreeNode* node);
  void decrementLevel(SimulationTreeNode* node);
  std::queue<SimulationTreeNode*> createChildren(SimulationTreeNode* parent);

  // Prevents use-after-free error
  void addDestroyedPin(const Pin* pin, BaseMove* move);
  void removeDestroyedPin(const Pin* pin, BaseMove* move);
  bool isPinDestroyed(const Pin* pin);

  // Measure simulation and time progress
  bool hasTimeLimitPassed();

  const Pin* endpoint_;
  const std::vector<const Pin*>* pins_;
  const std::vector<BaseMove*>* moves_;
  int max_level_;
  SearchMode mode_;
  int time_limit_;
  float setup_slack_margin_;

  SimulationTreeNode* root_{nullptr};
  std::unordered_set<const Pin*> destroyed_pins_;
  std::map<const Pin*, std::unordered_set<BaseMove*>> rejected_moves_;

  std::chrono::time_point<std::chrono::steady_clock> start_time_;

  Resizer* resizer_;
  Logger* logger_;
  Sta* sta_;
  Network* network_;
  ViolatorCollector* violator_collector_;
};

}  // namespace rsz
