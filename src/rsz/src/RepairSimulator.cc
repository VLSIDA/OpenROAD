// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "RepairSimulator.hh"

#include <string>

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
                           float setup_slack_margin)
{
  endpoint_ = endpoint;
  pins_ = &pins;
  moves_ = &moves;
  max_level_ = level;
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

  // Expand the simulation tree
  simulateDFS(root_);

  debugPrint(logger_, RSZ, "repair_simulator", 1, "Finished repair simulation");
}

// Recursive DFS simulation helper
void RepairSimulator::simulateDFS(SimulationTreeNode* node)
{
  // Skip if reached the max level
  if (node->level_ >= max_level_) {
    return;
  }
  // Continue with the children if we've already simulated this node
  if (node->is_simulated_) {
    for (SimulationTreeNode* child : node->children_) {
      doMove(child);
      simulateDFS(child);
      undoMove(child);
    }
    return;
  }
  // Simulate the children of this node
  for (const Pin* pin : *pins_) {
    if (isPinDestroyed(pin)) {
      continue;
    }
    for (BaseMove* move : *moves_) {
      if (isMoveRejected(pin, move)) {
        continue;
      }
      SimulationTreeNode* child
          = new SimulationTreeNode(resizer_, pin, move, node->level_ + 1);
      if (!doMove(child)) {
        delete child;
        //addRejectedMove(pin, move);
        continue;
      }
      node->children_.push_back(child);
      simulateDFS(child);
      undoMove(child);
    }
  }
  node->is_simulated_ = true;
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
    SimulationTreeNode* node)
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
    node->eco_->redo();
    node->move_->addMove(node->pin_, node->tracked_changes_);
    resizer_->estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    return true;
  }
  // This node needs to be simulated from scratch
  debugPrint(logger_, RSZ, "repair_simulator", 5, "Doing {}", node->name());
  addDestroyedPin(node->pin_, node->move_);
  node->odb_eco_active_ = true;
  odb::dbDatabase::beginEco(resizer_->block_);
  bool accepted = node->move_->doMove(node->pin_, setup_slack_margin_);
  odb::dbDatabase::endEco(resizer_->block_);
  if (accepted) {
    node->eco_ = new dbJournal(resizer_->block_);
    _dbBlock* block = (_dbBlock*) resizer_->block_;
    node->eco_->append(block->_journal_stack.top());
    resizer_->estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    node->slack_ = violator_collector_->getCurrentEndpointSlack();
    node->tracked_changes_ = node->move_->tracking_stack_.back().second;
  } else {
    debugPrint(
        logger_, RSZ, "repair_simulator", 5, "Rejected {}", node->name());
    odb::dbDatabase::undoEco(resizer_->block_);
    node->odb_eco_active_ = false;
    removeDestroyedPin(node->pin_, node->move_);
  }
  return accepted;
}

void RepairSimulator::undoMove(SimulationTreeNode* node)
{
  debugPrint(logger_, RSZ, "repair_simulator", 5, "Undoing {}", node->name());
  if (node->odb_eco_active_) {
    odb::dbDatabase::undoEco(resizer_->block_);
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
  assert(found);
  debugPrint(
      logger_, RSZ, "repair_simulator", 3, "Committing {}", root_->name());
  // Redo the node journal
  doMove(root_);
  odb::dbDatabase::beginEco(resizer_->block_);
  _dbBlock* block = (_dbBlock*) resizer_->block_;
  // There should always be an active journal ECO originating from RepairSetup
  if (block->_journal) {
    block->_journal->append(root_->eco_);
  } else {
    block->_journal_stack.top()->append(root_->eco_);
  }
  delete root_->eco_;
  root_->eco_ = nullptr;
  decrementLevel(root_);
  Slack new_endpoint_slack = violator_collector_->getCurrentEndpointSlack();
  if (fuzzyLess(new_endpoint_slack, root_->slack_)) {
    logger_->warn(
        RSZ,
        168,
        "Endpoint's actual slack is worse than simulated slack: {} < {}",
        delayAsString(new_endpoint_slack, sta_, 3),
        delayAsString(root_->slack_, sta_, 3));
  }
}

// Recursively decrement each node's level
void RepairSimulator::decrementLevel(SimulationTreeNode* node)
{
  node->level_--;
  for (SimulationTreeNode* child : node->children_) {
    decrementLevel(child);
  }
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

void RepairSimulator::addRejectedMove(const Pin* pin, BaseMove* move)
{
  rejected_moves_[pin].insert(move);
}

bool RepairSimulator::isMoveRejected(const Pin* pin, BaseMove* move)
{
  return rejected_moves_[pin].find(move) != rejected_moves_[pin].end();
}

}  // namespace rsz
