// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cmath>
#include <deque>
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

// Class to search for the best repair move combinations
class RepairSearch
{
 public:
  enum class SearchMode
  {
    DFS,
    BFS,
    MCTS
  };

  RepairSearch(Resizer* resizer, ViolatorCollector* violator_collector)
      : resizer_(resizer)
  {
    logger_ = resizer_->logger_;
    sta_ = resizer_->sta_;
    network_ = resizer_->network_;
    violator_collector_ = violator_collector;
  }

  ~RepairSearch() { clear(); }

  void init(const Pin* endpoint,
            const std::vector<const Pin*>& pins,
            const std::vector<BaseMove*>& moves,
            int level,
            SearchMode mode,
            int time_limit,
            float setup_slack_margin);
  void clear();
  void search();
  std::pair<const Pin*, BaseMove*> getBestImmediateMove();
  void commitMove(const Pin* pin, BaseMove* move);

  const Pin* getEndpoint() { return endpoint_; }

 private:
  class SearchTreeNode
  {
   public:
    SearchTreeNode(Resizer* resizer, const Pin* pin, BaseMove* move, int level)
        : pin_(pin), move_(move), level_(level)
    {
      pin_name_ = pin ? resizer->network_->pathName(pin) : "null";
      move_name_ = move ? move->name() : "null";
    }

    ~SearchTreeNode()
    {
      delete eco_;
      delete move_info_;
      for (auto* child : children_) {
        delete child;
      }
    }

    std::string name()
    {
      return fmt::format("STNode({}, {}, L{})", pin_name_, move_name_, level_);
    }

    const Pin* pin_;
    BaseMove* move_;
    int level_;
    std::string pin_name_;
    std::string move_name_;
    bool search_started_{false};
    bool search_finished_{false};
    bool search_aborted_{false};
    bool odb_eco_active_{false};
    // Slack of path endpoint
    Slack slack_{0.0};
    // MCTS variables
    int mcts_visits_{0};
    Slack mcts_best_slack_{-sta::INF};
    // Isolated ECO journal of this node (doesn't include other nodes' ECOs)
    dbJournal* eco_{nullptr};
    // Keep the move info object to handle the quirks of BaseMove
    MoveInfo* move_info_{nullptr};
    // Children of this node
    std::vector<SearchTreeNode*> children_;
    // Children of this node haven't been searched yet
    std::deque<SearchTreeNode*> children_pending_;
  };

  bool doMove(SearchTreeNode* node);
  void undoMove(SearchTreeNode* node);
  bool searchBFS(SearchTreeNode* node);
  bool searchDFS(SearchTreeNode* node);
  bool searchMCTS(SearchTreeNode* node);
  SearchTreeNode* getBestPossibleNodeDFS(SearchTreeNode* node);
  void decrementLevel(SearchTreeNode* node);
  std::deque<SearchTreeNode*> createChildren(SearchTreeNode* parent);

  // Prevents use-after-free error
  void addDestroyedPin(const Pin* pin, BaseMove* move);
  void removeDestroyedPin(const Pin* pin, BaseMove* move);
  bool isPinDestroyed(const Pin* pin);

  // Measure time elapsed to stop the search
  bool hasTimeLimitPassed();

  const Pin* endpoint_;
  const std::vector<const Pin*>* pins_;
  const std::vector<BaseMove*>* moves_;
  int max_level_;
  SearchMode mode_;
  int time_limit_;
  float setup_slack_margin_;

  SearchTreeNode* root_{nullptr};
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
