// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "RepairSimulator.hh"

#include <cmath>
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

void RepairSimulator::init(const Pin* endpoint,
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
  root_ = new SimulationTreeNode(resizer_, nullptr, nullptr, 0);
  root_->slack_ = violator_collector_->getCurrentEndpointSlack();
}

void RepairSimulator::clear()
{
  endpoint_ = nullptr;
  pins_ = nullptr;
  moves_ = nullptr;
  delete root_;
  root_ = nullptr;
  destroyed_pins_.clear();
  rejected_moves_.clear();
}

// Run the actual simulation
void RepairSimulator::simulate()
{
  debugPrint(logger_,
             RSZ,
             "repair_simulator",
             1,
             "Started repair simulation for endpoint: {} with slack {}",
             network_->pathName(endpoint_),
             delayAsString(root_->slack_, sta_, 3));

  start_time_ = std::chrono::steady_clock::now();

  // Expand the simulation tree
  bool finished;
  switch (mode_) {
    case SearchMode::BFS:
      finished = simulateBFS(root_);
      break;
    case SearchMode::DFS:
      finished = simulateDFS(root_);
      break;
    case SearchMode::MCTS:
      finished = simulateMCTS(root_);
      break;
    default:
      logger_->error(RSZ, 156, "Search mode unknown");
      return;
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_secs
      = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time_)
            .count();
  if (finished) {
    debugPrint(logger_,
               RSZ,
               "repair_simulator",
               1,
               "Finished repair simulation in {} secs",
               elapsed_secs);
  } else {
    debugPrint(
        logger_,
        RSZ,
        "repair_simulator",
        1,
        "Finished repair simulation early after {} secs due to time limit",
        elapsed_secs);
  }
}

// BFS simulation helper
bool RepairSimulator::simulateBFS(SimulationTreeNode* node)
{
  // Queue stores pairs of (node, path_from_root_to_node)
  std::queue<std::pair<SimulationTreeNode*, std::vector<SimulationTreeNode*>>>
      queue;
  std::vector<SimulationTreeNode*> root_path;
  root_path.push_back(node);
  queue.emplace(node, root_path);
  // Breadth-first search
  while (!queue.empty()) {
    std::pair<SimulationTreeNode*, std::vector<SimulationTreeNode*>>
        current_pair = queue.front();
    queue.pop();
    SimulationTreeNode* current = current_pair.first;
    std::vector<SimulationTreeNode*> path = current_pair.second;
    // Skip if reached max level
    if (current->level_ >= max_level_) {
      continue;
    }
    // If this node has simulated at least some of its all its children, add
    // them to the queue
    if (current->simulation_finished_ || current->simulation_aborted_) {
      assert(current->simulation_started_);
      for (SimulationTreeNode* child : current->children_) {
        std::vector<SimulationTreeNode*> child_path = path;
        child_path.push_back(child);
        queue.emplace(child, child_path);
      }
      if (current->simulation_finished_) {
        continue;
      }
      current->simulation_aborted_ = false;
    }
    // Continue if no pending children are left
    if (current->simulation_started_ && current->children_pending_.empty()) {
      current->simulation_finished_ = true;
      continue;
    }
    // Apply all moves along the path from root to current (excluding root)
    for (size_t i = 1; i < path.size(); i++) {
      doMove(path[i]);
    }
    // Create all children as pending
    if (!current->simulation_started_) {
      current->children_pending_ = createChildren(current);
    }
    current->simulation_started_ = true;
    // Process all children
    while (!current->children_pending_.empty()) {
      auto child = current->children_pending_.front();
      current->children_pending_.pop();
      if (!doMove(child)) {
        delete child;
        continue;
      }
      current->children_.push_back(child);
      // Create path for child and add to queue
      std::vector<SimulationTreeNode*> child_path = path;
      child_path.push_back(child);
      queue.emplace(child, child_path);
      undoMove(child);
      // Early termination if we found a good enough solution
      if (!fuzzyLess(child->slack_, setup_slack_margin_)) {
        debugPrint(logger_,
                   RSZ,
                   "repair_simulator",
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
        current->simulation_aborted_ = true;
        break;
      }
    }
    // Undo all moves back to root
    for (int i = path.size() - 1; i >= 1; i--) {
      undoMove(path[i]);
    }
    if (current->simulation_aborted_) {
      return false;
    }
    current->simulation_finished_ = true;
  }
  return true;
}

// DFS simulation helper
bool RepairSimulator::simulateDFS(SimulationTreeNode* node)
{
  // Skip if reached the max level
  if (node->level_ >= max_level_) {
    return true;
  }
  // If this node has simulated at least some of its all its children, run them
  // first
  if (node->simulation_finished_ || node->simulation_aborted_) {
    assert(node->simulation_started_);
    for (SimulationTreeNode* child : node->children_) {
      doMove(child);
      bool finished = simulateDFS(child);
      undoMove(child);
      if (!finished) {
        node->simulation_aborted_ = true;
        return false;
      }
    }
    if (node->simulation_finished_) {
      return true;
    }
    node->simulation_aborted_ = false;
  }
  // Create all children as pending
  if (!node->simulation_started_) {
    node->children_pending_ = createChildren(node);
  }
  node->simulation_started_ = true;
  // Process all children
  while (!node->children_pending_.empty()) {
    auto child = node->children_pending_.front();
    node->children_pending_.pop();
    if (!doMove(child)) {
      delete child;
      continue;
    }
    node->children_.push_back(child);
    bool finished = simulateDFS(child);
    undoMove(child);
    if (!finished) {
      node->simulation_aborted_ = true;
      return false;
    }
    // Early termination if we found a good enough solution
    if (!fuzzyLess(child->slack_, setup_slack_margin_)) {
      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 2,
                 "DFS found optimal solution at level {}",
                 child->level_);
      node->simulation_finished_ = true;
      return true;
    }
  }
  node->simulation_finished_ = true;
  return true;
}

// MCTS simulation helper
bool RepairSimulator::simulateMCTS(RepairSimulator::SimulationTreeNode* node)
{
  // Adaptive iteration count based on search space size
  const int branching_factor = pins_->size() * moves_->size();
  const int base_iterations = branching_factor * max_level_;
  const int num_iterations = std::min(10000, std::max(100, base_iterations * 2));
  const float exploration_constant = 1.414;  // sqrt(2)

  debugPrint(logger_,
             RSZ,
             "repair_simulator",
             2,
             "MCTS running with max {} iterations (branching={}, depth={})",
             num_iterations,
             branching_factor,
             max_level_);

  for (int iter = 0; iter < num_iterations; iter++) {
    std::vector<SimulationTreeNode*> path;
    SimulationTreeNode* current = node;
    path.push_back(current);
    // Phase 1: Selection - traverse tree using UCB1
    while (current->level_ < max_level_ && current->simulation_finished_
           && !current->children_.empty()) {
      SimulationTreeNode* best_child = nullptr;
      float best_ucb = -sta::INF;
      for (SimulationTreeNode* child : current->children_) {
        float ucb;
        if (child->mcts_visits_ == 0) {
          ucb = sta::INF;  // Prioritize unvisited nodes
        } else {
          float exploitation = child->mcts_best_slack_;
          float exploration
              = exploration_constant
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
      if (!current->simulation_started_) {
        current->children_pending_ = createChildren(current);
        current->simulation_started_ = true;
      }
      if (!current->children_pending_.empty()) {
        auto child = current->children_pending_.front();
        current->children_pending_.pop();
        if (doMove(child)) {
          current->children_.push_back(child);
          path.push_back(child);
          current = child;
        } else {
          delete child;
        }
      } else {
        current->simulation_finished_ = true;
      }
    }
    // Phase 3: Evaluation - get slack at current node
    Slack new_path_slack = current->slack_;
    // Phase 4: Backpropagation - update statistics
    for (SimulationTreeNode* node : path) {
      node->mcts_visits_++;
      if (node->mcts_visits_ == 1 || fuzzyLess(node->mcts_best_slack_, new_path_slack)) {
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
    // Early termination if we found a good enough solution
    if (!fuzzyLess(new_path_slack, setup_slack_margin_)) {
      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 2,
                 "MCTS found optimal solution at iteration {}",
                 iter);
      break;
    }
  }
  return true;
}

// Return the best immediate move of the simulation tree
std::pair<const Pin*, BaseMove*> RepairSimulator::getBestImmediateMove()
{
  // Recursively search for the best immediate move among root's children
  debugPrint(logger_, RSZ, "repair_simulator", 4, "Simulated moves:");
  SimulationTreeNode* best_descendant = nullptr;
  SimulationTreeNode* best_child = nullptr;
  Slack best_slack = -sta::INF;
  for (SimulationTreeNode* child : root_->children_) {
    SimulationTreeNode* descendant_of_child = getBestPossibleNodeDFS(child);
    if (fuzzyLess(best_slack, descendant_of_child->slack_)) {
      best_descendant = descendant_of_child;
      best_child = child;
      best_slack = descendant_of_child->slack_;
    }
  }
  std::pair<const Pin*, BaseMove*> result = {nullptr, nullptr};
  if (best_child != nullptr && fuzzyLess(root_->slack_, best_slack)) {
    result = {best_child->pin_, best_child->move_};
    debugPrint(logger_,
               RSZ,
               "repair_simulator",
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
               "repair_simulator",
               1,
               "Couldn't find a good immediate move!");
  }
  return result;
}

// Best immediate move DFS helper
RepairSimulator::SimulationTreeNode* RepairSimulator::getBestPossibleNodeDFS(
    RepairSimulator::SimulationTreeNode* node)
{
  debugPrint(logger_,
             RSZ,
             "repair_simulator",
             4,
             std::string(node->level_ * 2, ' ') + "{} for {} with slack {}",
             node->move_->name(),
             network_->pathName(node->pin_),
             delayAsString(node->slack_, sta_, 3));
  if (node->level_ >= max_level_) {
    return node;
  }
  SimulationTreeNode* best_descendant = nullptr;
  Slack best_slack = -sta::INF;
  for (SimulationTreeNode* child : node->children_) {
    SimulationTreeNode* descendant = getBestPossibleNodeDFS(child);
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

bool RepairSimulator::doMove(SimulationTreeNode* node)
{
  // We have already simulated this node
  if (node->eco_) {
    debugPrint(logger_, RSZ, "repair_simulator", 5, "Redoing {}", node->name());
    addDestroyedPin(node->pin_, node->move_);
    if (((_dbBlock*) resizer_->block_)->_journal) {
      odb::dbDatabase::endEco(resizer_->block_);
    }
    node->eco_->redo();
    node->move_->addMove(node->pin_, node->tracked_changes_);
    return true;
  }
  // This node needs to be simulated from scratch
  debugPrint(logger_, RSZ, "repair_simulator", 5, "Doing {}", node->name());
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
    node->eco_->append(block->_journal_stack.top());
    // FIXME: Update STA after the move for now.
    //        We wouldn't do this if global router was dbJournal aware.
    odb::dbDatabase::beginEco(resizer_->block_);
    resizer_->estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    odb::dbDatabase::endEco(resizer_->block_);
    node->slack_ = violator_collector_->getCurrentEndpointSlack();
    // Track changes to prevent BaseMove::addMove() from corrupting
    node->tracked_changes_ = node->move_->tracking_stack_.back().second;
  } else {
    debugPrint(
        logger_, RSZ, "repair_simulator", 5, "Rejected {}", node->name());
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo move ECO
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo pre-move STA update ECO
    node->odb_eco_active_ = false;
    removeDestroyedPin(node->pin_, node->move_);
  }
  return accepted;
}

void RepairSimulator::undoMove(SimulationTreeNode* node)
{
  debugPrint(logger_, RSZ, "repair_simulator", 5, "Undoing {}", node->name());
  if (node->odb_eco_active_) {
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo post-move STA update ECO
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo move ECO
    odb::dbDatabase::undoEco(resizer_->block_);  // Undo pre-move STA update ECO
    node->odb_eco_active_ = false;
  } else {
    node->eco_->undo();
  }
  removeDestroyedPin(node->pin_, node->move_);
  node->move_->removeMove();
}

void RepairSimulator::commitMove(const Pin* pin, BaseMove* move)
{
  // Find the child node that corresponds to the given move
  bool found = false;
  for (auto it = root_->children_.begin(); it != root_->children_.end(); it++) {
    SimulationTreeNode* node = *it;
    if (node->pin_ == pin && node->move_ == move) {
      root_->children_.erase(it);
      delete root_;
      root_ = node;
      found = true;
      break;
    }
  }
  ZASSERT(found);
  debugPrint(
      logger_, RSZ, "repair_simulator", 3, "Committing {}", root_->name());
  // Redo this node's ECO
  doMove(root_);
  // There should always be an active ECO journal originating from RepairSetup
  _dbBlock* block = (_dbBlock*) resizer_->block_;
  if (block->_journal) {
    block->_journal->append(root_->eco_);
  } else {
    block->_journal_stack.top()->append(root_->eco_);
  }
  delete root_->eco_;
  root_->eco_ = nullptr;
  decrementLevel(root_);
}

// Recursively decrement each node's level
void RepairSimulator::decrementLevel(SimulationTreeNode* node)
{
  node->level_--;
  for (SimulationTreeNode* child : node->children_) {
    decrementLevel(child);
  }
}

std::queue<RepairSimulator::SimulationTreeNode*>
RepairSimulator::createChildren(RepairSimulator::SimulationTreeNode* parent)
{
  std::queue<RepairSimulator::SimulationTreeNode*> children;
  for (const Pin* pin : *pins_) {
    if (isPinDestroyed(pin)) {
      continue;
    }
    for (BaseMove* move : *moves_) {
      auto child
          = new SimulationTreeNode(resizer_, pin, move, parent->level_ + 1);
      children.emplace(child);
    }
  }
  return children;
}

void RepairSimulator::addDestroyedPin(const Pin* pin, BaseMove* move)
{
  if (move == (BaseMove*) resizer_->unbuffer_move_.get()) {
    destroyed_pins_.insert(pin);
  }
}

void RepairSimulator::removeDestroyedPin(const Pin* pin, BaseMove* move)
{
  if (move == (BaseMove*) resizer_->unbuffer_move_.get()) {
    destroyed_pins_.erase(pin);
  }
}

bool RepairSimulator::isPinDestroyed(const Pin* pin)
{
  return destroyed_pins_.find(pin) != destroyed_pins_.end();
}

bool RepairSimulator::hasTimeLimitPassed()
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
