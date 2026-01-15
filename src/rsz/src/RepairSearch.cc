// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "RepairSearch.hh"

#include <cassert>
#include <cmath>
#include <numbers>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "BaseMove.hh"
#include "odb/db.h"
#include "rsz/Resizer.hh"
#include "utl/Logger.h"

namespace rsz {

using std::string;

using sta::fuzzyLess;
using sta::Pin;
using utl::RSZ;

void RepairSearch::init(const Pin* endpoint,
                        const std::vector<const Pin*>& pins,
                        const std::vector<BaseMove*>& moves,
                        int level,
                        SearchMode mode,
                        int time_limit,
                        float setup_slack_margin)
{
  endpoint_ = endpoint;
  pins_ = &pins;
  moves_ = &moves;
  max_level_ = level;
  mode_ = mode;
  time_limit_ = time_limit;
  setup_slack_margin_ = setup_slack_margin;
  root_ = new SearchTreeNode(resizer_, nullptr, nullptr, 0);
  root_->slack_ = violator_collector_->getCurrentEndpointSlack();
}

void RepairSearch::clear()
{
  endpoint_ = nullptr;
  pins_ = nullptr;
  moves_ = nullptr;
  delete root_;
  root_ = nullptr;
  destroyed_pins_.clear();
  rejected_moves_.clear();
}

// Run the actual move search
void RepairSearch::search()
{
  debugPrint(logger_,
             RSZ,
             "repair_search",
             1,
             "Started repair search for endpoint: {} with slack {}",
             network_->pathName(endpoint_),
             delayAsString(root_->slack_, sta_, 3));

  start_time_ = std::chrono::steady_clock::now();

  // Expand the search tree
  bool finished;
  switch (mode_) {
    case SearchMode::BFS:
      finished = searchBFS(root_);
      break;
    case SearchMode::DFS:
      finished = searchDFS(root_);
      break;
    case SearchMode::MCTS:
      finished = searchMCTS(root_);
      break;
    default:
      logger_->error(RSZ, 156, "Unexpected repair search mode");
      return;
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_secs
      = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time_)
            .count();
  if (finished) {
    debugPrint(logger_,
               RSZ,
               "repair_search",
               1,
               "Finished repair search in {} secs",
               elapsed_secs);
  } else {
    debugPrint(logger_,
               RSZ,
               "repair_search",
               1,
               "Finished repair search early after {} secs due to time limit",
               elapsed_secs);
  }
}

bool RepairSearch::searchBFS(SearchTreeNode* node)
{
  // Queue stores pairs of (node, path_from_root_to_node)
  std::queue<std::pair<SearchTreeNode*, std::vector<SearchTreeNode*>>> queue;
  std::vector<SearchTreeNode*> root_path;
  root_path.push_back(node);
  queue.emplace(node, root_path);
  // Breadth-first search
  while (!queue.empty()) {
    auto current_pair = queue.front();  // (node, path_from_root_to_node)
    queue.pop();
    SearchTreeNode* current = current_pair.first;
    std::vector<SearchTreeNode*> path = current_pair.second;
    // Skip if reached max level
    if (current->level_ >= max_level_) {
      continue;
    }
    current->search_aborted_ = false;
    // If this node has finished exploring all its children, add them the queue
    if (current->search_finished_) {
      assert(current->search_started_);
      for (auto child : current->children_) {
        std::vector<SearchTreeNode*> child_path = path;
        child_path.push_back(child);
        queue.emplace(child, child_path);
      }
      if (current->search_finished_) {
        continue;
      }
      current->search_aborted_ = false;
    }
    // Continue if no pending children are left
    if (current->search_started_ && current->children_pending_.empty()) {
      current->search_finished_ = true;
      continue;
    }
    // Apply all moves along the path from root to current (excluding root)
    for (auto i = 1; i < path.size(); i++) {
      doMove(path[i]);
    }
    // Create all children as pending
    if (!current->search_started_) {
      current->children_pending_ = createChildren(current);
    }
    current->search_started_ = true;
    // Explore all children
    while (!current->children_pending_.empty()) {
      auto child = current->children_pending_.front();
      current->children_pending_.pop_front();
      if (!doMove(child)) {
        delete child;
        continue;
      }
      current->children_.push_back(child);
      // Create path for child and add to queue
      std::vector<SearchTreeNode*> child_path = path;
      child_path.push_back(child);
      queue.emplace(child, child_path);
      undoMove(child);
      // Early termination if we've found a good enough solution
      if (!fuzzyLess(child->slack_, setup_slack_margin_)) {
        debugPrint(logger_,
                   RSZ,
                   "repair_search",
                   2,
                   "BFS found optimal solution at level {}",
                   child->level_);
        // Undo all moves back to root before returning
        for (int i = path.size() - 1; i >= 1; i--) {
          undoMove(path[i]);
        }
        return true;
      }
      if (hasTimeLimitPassed()) {
        current->search_aborted_ = true;
        break;
      }
    }
    // Undo all moves back to root
    for (int i = path.size() - 1; i >= 1; i--) {
      undoMove(path[i]);
    }
    if (current->search_aborted_) {
      return false;
    }
    current->search_finished_ = true;
  }
  return true;
}

bool RepairSearch::searchDFS(SearchTreeNode* node)
{
  // Skip if reached the max level
  if (node->level_ >= max_level_) {
    return true;
  }
  node->search_aborted_ = false;
  // If this node has explored at least some of its children, run them first
  if (node->search_started_) {
    for (SearchTreeNode* child : node->children_) {
      doMove(child);
      bool finished = searchDFS(child);
      undoMove(child);
      if (!finished) {
        node->search_aborted_ = true;
        return false;
      }
    }
    if (node->search_finished_) {
      return true;
    }
    node->search_aborted_ = false;
  }
  // Create all children as pending
  if (!node->search_started_) {
    node->children_pending_ = createChildren(node);
  }
  node->search_started_ = true;
  // Process all children
  while (!node->children_pending_.empty()) {
    auto child = node->children_pending_.front();
    node->children_pending_.pop_front();
    if (!doMove(child)) {
      delete child;
      continue;
    }
    node->children_.push_back(child);
    bool finished = searchDFS(child);
    undoMove(child);
    if (!finished) {
      node->search_aborted_ = true;
      return false;
    }
    // Early termination if we found a good enough solution
    if (!fuzzyLess(child->slack_, setup_slack_margin_)) {
      debugPrint(logger_,
                 RSZ,
                 "repair_search",
                 2,
                 "DFS found optimal solution at level {}",
                 child->level_);
      node->search_finished_ = true;
      return true;
    }
  }
  node->search_finished_ = true;
  return true;
}

bool RepairSearch::searchMCTS(RepairSearch::SearchTreeNode* node)
{
  // Adaptive iteration count based on search space size
  const int branching_factor = pins_->size() * moves_->size();
  const int base_iterations = branching_factor * max_level_;
  const int num_iterations
      = std::min(10000, std::max(100, base_iterations * max_level_));
  const float exploration_constant = std::numbers::sqrt2;

  debugPrint(logger_,
             RSZ,
             "repair_search",
             2,
             "MCTS running with max {} iterations (branching={}, depth={})",
             num_iterations,
             branching_factor,
             max_level_);

  for (int iter = 0; iter < num_iterations; iter++) {
    std::vector<SearchTreeNode*> path;
    SearchTreeNode* current = node;
    path.push_back(current);
    // Phase 1: Selection - traverse tree using UCB1
    while (current->level_ < max_level_ && current->search_finished_
           && !current->children_.empty()) {
      SearchTreeNode* best_child = nullptr;
      float best_ucb = -sta::INF;
      for (SearchTreeNode* child : current->children_) {
        float ucb;
        if (child->mcts_visits_ == 0) {
          ucb = sta::INF;  // Prioritize unvisited nodes
        } else {
          float exploitation = child->mcts_best_slack_;
          float exploration = exploration_constant
                              * std::sqrt(std::log(current->mcts_visits_)
                                          / child->mcts_visits_);
          ucb = exploitation + exploration;
        }
        if (ucb > best_ucb) {
          best_ucb = ucb;
          best_child = child;
        }
      }
      if (best_child == nullptr) {
        break;
      }
      current = best_child;
      doMove(current);
      path.push_back(current);
    }
    // Phase 2: Expansion - add one child if not fully expanded
    if (current->level_ < max_level_) {
      if (!current->search_started_) {
        current->children_pending_ = createChildren(current);
        current->search_started_ = true;
      }
      if (!current->children_pending_.empty()) {
        auto child = current->children_pending_.front();
        current->children_pending_.pop_front();
        debugPrint(logger_,
                   RSZ,
                   "repair_search",
                   5,
                   "MCTS expanding {} -> {}",
                   current->name(),
                   child->name());
        if (doMove(child)) {
          current->children_.push_back(child);
          path.push_back(child);
          current = child;
        } else {
          delete child;
        }
      } else {
        current->search_finished_ = true;
      }
    }
    // Phase 3: Evaluation - get slack at current node
    Slack new_path_slack = current->slack_;
    // Phase 4: Backpropagation - update statistics
    for (SearchTreeNode* node : path) {
      node->mcts_visits_++;
      if (node->mcts_visits_ == 1
          || fuzzyLess(node->mcts_best_slack_, new_path_slack)) {
        node->mcts_best_slack_ = new_path_slack;
      }
    }
    // Undo all moves back to root
    for (int i = path.size() - 1; i >= 1; i--) {
      undoMove(path[i]);
    }
    if (hasTimeLimitPassed()) {
      return false;
    }
    // Early termination if we've found a good enough solution
    if (!fuzzyLess(new_path_slack, setup_slack_margin_)) {
      debugPrint(logger_,
                 RSZ,
                 "repair_search",
                 2,
                 "MCTS found optimal solution at iteration {}",
                 iter);
      break;
    }
  }
  return true;
}

// Return the best immediate move of the search tree
std::pair<const Pin*, BaseMove*> RepairSearch::getBestImmediateMove()
{
  // Recursively search for the best immediate move among root's children
  debugPrint(logger_, RSZ, "repair_search", 4, "Searched moves:");
  SearchTreeNode* best_descendant = nullptr;
  SearchTreeNode* best_child = nullptr;
  Slack best_slack = -sta::INF;
  for (SearchTreeNode* child : root_->children_) {
    SearchTreeNode* descendant_of_child = getBestPossibleNodeDFS(child);
    if (fuzzyLess(best_slack, descendant_of_child->slack_)) {
      best_descendant = descendant_of_child;
      best_child = child;
      best_slack = descendant_of_child->slack_;
    }
  }
  std::pair<const Pin*, BaseMove*> result = {nullptr, nullptr};
  if (best_child && fuzzyLess(root_->slack_, best_slack)) {
    result = {best_child->pin_, best_child->move_};
    debugPrint(logger_,
               RSZ,
               "repair_search",
               1,
               "Found the best immediate move: {} for {}. "
               "Potential slack {} -> {} -{}-> {}",
               best_child->move_->name(),
               network_->pathName(best_child->pin_),
               delayAsString(root_->slack_, sta_, 3),
               delayAsString(best_child->slack_, sta_, 3),
               best_descendant->level_ - root_->level_ - 1,
               delayAsString(best_slack, sta_, 3));
  } else {
    debugPrint(logger_,
               RSZ,
               "repair_search",
               1,
               "Couldn't find a good immediate move!");
  }
  return result;
}

// Best immediate move DFS helper
RepairSearch::SearchTreeNode* RepairSearch::getBestPossibleNodeDFS(
    RepairSearch::SearchTreeNode* node)
{
  debugPrint(logger_,
             RSZ,
             "repair_search",
             4,
             std::string(node->level_ * 2, ' ') + "{} for {} with slack {}",
             node->move_->name(),
             network_->pathName(node->pin_),
             delayAsString(node->slack_, sta_, 3));
  if (node->level_ >= max_level_) {
    return node;
  }
  SearchTreeNode* best_descendant = nullptr;
  Slack best_slack = -sta::INF;
  for (SearchTreeNode* child : node->children_) {
    SearchTreeNode* descendant = getBestPossibleNodeDFS(child);
    if (fuzzyLess(best_slack, descendant->slack_)) {
      best_descendant = descendant;
      best_slack = descendant->slack_;
    }
  }
  if (best_descendant == nullptr) {
    return node;
  }
  return best_descendant;
}

bool RepairSearch::doMove(SearchTreeNode* node)
{
  // We have already searched this node
  if (node->eco_) {
    debugPrint(logger_, RSZ, "repair_search", 6, "Redoing {}", node->name());
    addDestroyedPin(node->pin_, node->move_);
    if (((_dbBlock*) resizer_->block_)->journal_) {
      odb::dbDatabase::endEco(resizer_->block_);
    }
    node->eco_->redo();
    for (auto& [inst, count] : node->move_info_->changes) {
      node->move_->countMove(inst, count);
    }
    return true;
  }
  // This node needs to be explored from scratch
  debugPrint(logger_, RSZ, "repair_search", 6, "Doing {}", node->name());
  addDestroyedPin(node->pin_, node->move_);
  node->odb_eco_active_ = true;
  // FIXME: Update STA before the move for now.
  //        We wouldn't do this if global router was dbJournal aware.
  odb::dbDatabase::beginEco(resizer_->block_);
  resizer_->estimate_parasitics_->updateParasitics();
  sta_->findRequireds();
  odb::dbDatabase::endEco(resizer_->block_);
  // Run the move
  odb::dbDatabase::beginEco(resizer_->block_);
  bool accepted = node->move_->doMove(node->pin_, setup_slack_margin_);
  odb::dbDatabase::endEco(resizer_->block_);
  if (accepted) {
    // Save move ECO
    node->eco_ = new dbJournal(resizer_->block_);
    _dbBlock* block = (_dbBlock*) resizer_->block_;
    node->eco_->append(block->journal_stack_.top());
    // FIXME: Update STA after the move for now.
    //        We wouldn't do this if global router was dbJournal aware.
    odb::dbDatabase::beginEco(resizer_->block_);
    resizer_->estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    odb::dbDatabase::endEco(resizer_->block_);
    node->slack_ = violator_collector_->getCurrentEndpointSlack();
    node->move_info_ = node->move_->pending_move_info_.back();
    node->move_->pending_move_info_.pop_back();
  } else {
    debugPrint(logger_, RSZ, "repair_search", 6, "Rejected {}", node->name());
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo move ECO
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo pre-move STA update
    node->odb_eco_active_ = false;
    removeDestroyedPin(node->pin_, node->move_);
  }
  return accepted;
}

void RepairSearch::undoMove(SearchTreeNode* node)
{
  debugPrint(logger_, RSZ, "repair_search", 6, "Undoing {}", node->name());
  if (node->odb_eco_active_) {
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo post-move STA update
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo move ECO
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo pre-move STA update
    node->odb_eco_active_ = false;
  } else {
    node->eco_->undo();
  }
  removeDestroyedPin(node->pin_, node->move_);
  for (auto& [inst, count] : node->move_info_->changes) {
    node->move_->uncountMove(inst, count);
  }
}

void RepairSearch::commitMove(const Pin* pin, BaseMove* move)
{
  // Find the child node that corresponds to the given move
  bool found = false;
  for (auto it = root_->children_.begin(); it != root_->children_.end(); it++) {
    SearchTreeNode* node = *it;
    if (node->pin_ == pin && node->move_ == move) {
      root_->children_.erase(it);
      delete root_;
      root_ = node;
      found = true;
      break;
    }
  }
  assert(found);
  debugPrint(logger_, RSZ, "repair_search", 3, "Committing {}", root_->name());
  // Redo this node's ECO
  doMove(root_);
  root_->move_->pending_move_info_.push_back(root_->move_info_);
  for (auto& [inst, count] : root_->move_info_->changes) {
    root_->move_->countMove(inst, count);
  }
  // There should always be an active ECO journal originating from RepairSetup
  _dbBlock* block = (_dbBlock*) resizer_->block_;
  if (block->journal_) {
    block->journal_->append(root_->eco_);
  } else {
    block->journal_stack_.top()->append(root_->eco_);
  }
  delete root_->eco_;
  root_->eco_ = nullptr;
  root_->move_info_ = nullptr;
  decrementLevel(root_);
}

// Recursively decrement each node's level
void RepairSearch::decrementLevel(SearchTreeNode* node)
{
  node->level_--;
  for (SearchTreeNode* child : node->children_) {
    decrementLevel(child);
  }
  for (SearchTreeNode* child : node->children_pending_) {
    decrementLevel(child);
  }
}

std::deque<RepairSearch::SearchTreeNode*> RepairSearch::createChildren(
    RepairSearch::SearchTreeNode* parent)
{
  std::deque<RepairSearch::SearchTreeNode*> children;
  for (const Pin* pin : *pins_) {
    if (isPinDestroyed(pin)) {
      continue;
    }
    for (BaseMove* move : *moves_) {
      auto child = new SearchTreeNode(resizer_, pin, move, parent->level_ + 1);
      children.emplace_back(child);
    }
  }
  return children;
}

void RepairSearch::addDestroyedPin(const Pin* pin, BaseMove* move)
{
  if (move == (BaseMove*) resizer_->unbuffer_move_.get()) {
    destroyed_pins_.insert(pin);
  }
}

void RepairSearch::removeDestroyedPin(const Pin* pin, BaseMove* move)
{
  if (move == (BaseMove*) resizer_->unbuffer_move_.get()) {
    destroyed_pins_.erase(pin);
  }
}

bool RepairSearch::isPinDestroyed(const Pin* pin)
{
  return destroyed_pins_.find(pin) != destroyed_pins_.end();
}

bool RepairSearch::hasTimeLimitPassed()
{
  if (time_limit_ < 0) {
    return false;
  }
  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_secs
      = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time_)
            .count();
  return elapsed_secs >= time_limit_;
}

}  // namespace rsz
