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
  root_ = new SimulationTreeNode(nullptr, nullptr, 0);
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
             "Started repair simulation for endpoint: {}",
             network_->pathName(endpoint_));

  std::string pins_str;
  for (const Pin* pin : *pins_) {
    pins_str += " ";
    pins_str += network_->pathName(pin);
  }
  debugPrint(logger_, RSZ, "repair_simulator", 2, "Pins:{}", pins_str);

  std::string moves_str;
  for (BaseMove* move : *moves_) {
    moves_str += " ";
    moves_str += move->name();
  }
  debugPrint(logger_, RSZ, "repair_simulator", 2, "Moves:{}", moves_str);

  // Generate the simulation tree
  simulateDFS(root_);

  // Get the possible slack of root's children
  for (SimulationTreeNode* child : root_->children_) {
    trickleUpBestSlack(child);
  }

  debugPrint(logger_, RSZ, "repair_simulator", 1, "Finished repair simulation");
}

// Return the best immediate move of the simulation tree
std::pair<const Pin*, BaseMove*> RepairSimulator::getBestImmediateMove()
{
  // Find the best immediate move from root's children
  SimulationTreeNode* best_child = nullptr;
  Slack best_slack = -sta::INF;
  for (SimulationTreeNode* child : root_->children_) {
    if (fuzzyLess(best_slack, child->slack_)) {
      best_slack = child->slack_;
      best_child = child;
    }
  }
  std::pair<const Pin*, BaseMove*> result = {nullptr, nullptr};
  if (best_child != nullptr) {
    result = {best_child->pin_, best_child->move_};
  }
  return result;
}

void RepairSimulator::commitMove(const Pin* pin, BaseMove* move)
{
  debugPrint(logger_,
             RSZ,
             "repair_simulator",
             3,
             "Committing {} for {} on level 1",
             move->name(),
             network_->pathName(pin));

  // Find the child node that corresponds to the given move
  for (auto it = root_->children_.begin(); it != root_->children_.end(); it++) {
    SimulationTreeNode* node = *it;
    if (node->pin_ == pin && node->move_ == move) {
      root_->children_.erase(it);
      delete root_;
      root_ = node;
      break;
    }
  }
  // Redo the node journal
  addDestroyedPin(root_->pin_, root_->move_);
  root_->eco_->redo();
  delete root_->eco_;
  root_->eco_ = nullptr;
  decrementLevel(root_);
}

// Recursive DFS simulation helper
void RepairSimulator::simulateDFS(SimulationTreeNode* node)
{
  // Skip if reached the max level
  if (node->level_ >= max_level_) {
    return;
  }

  // Continue with the children if we've already simulated this node
  if (!node->children_.empty()) {
    for (SimulationTreeNode* child : node->children_) {
      const char* pin_name = network_->pathName(child->pin_);
      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 3,
                 "Redoing {} for {} on level {}",
                 child->move_->name(),
                 pin_name,
                 child->level_);
      addDestroyedPin(child->pin_, child->move_);
      child->eco_->redo();
      simulateDFS(child);
      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 3,
                 "Undoing {} for {} on level {}",
                 child->move_->name(),
                 pin_name,
                 child->level_);
      child->eco_->undo();
      removeDestroyedPin(child->pin_, child->move_);
    }
    return;
  }

  for (const Pin* pin : *pins_) {
    if (isPinDestroyed(pin)) {
      continue;
    }
    const char* pin_name = network_->pathName(pin);
    for (BaseMove* move : *moves_) {
      if (isMoveRejected(pin, move)) {
        continue;
      }
      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 3,
                 "Doing {} for {} on level {}",
                 move->name(),
                 pin_name,
                 node->level_ + 1);

      SimulationTreeNode* child
          = new SimulationTreeNode(pin, move, node->level_ + 1);

      odb::dbDatabase::beginEco(resizer_->block_);
      addDestroyedPin(pin, move);
      if (!move->doMove(pin, setup_slack_margin_)) {
        debugPrint(logger_,
                   RSZ,
                   "repair_simulator",
                   3,
                   "Rejected {} for {} on level {}",
                   move->name(),
                   pin_name,
                   node->level_ + 1);
        odb::dbDatabase::undoEco(resizer_->block_);
        removeDestroyedPin(pin, move);
        delete child;
        continue;
      }
      child->eco_ = new dbJournal(resizer_->block_);
      _dbBlock* block = (_dbBlock*) resizer_->block_;
      child->eco_->append(block->_journal);
      child->slack_ = violator_collector_->getPathSlackByIndex(endpoint_, 0);
      node->children_.push_back(child);
      simulateDFS(child);

      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 3,
                 "Undoing {} for {} on level {}",
                 move->name(),
                 pin_name,
                 node->level_ + 1);

      odb::dbDatabase::undoEco(resizer_->block_);
      removeDestroyedPin(pin, move);
    }
  }
}

// Trickle the best slacks up the tree
void RepairSimulator::trickleUpBestSlack(SimulationTreeNode* node)
{
  if (node->children_.empty()) {
    return;
  }
  for (SimulationTreeNode* child : node->children_) {
    trickleUpBestSlack(child);
  }
  Slack best_slack = -sta::INF;
  for (SimulationTreeNode* child : node->children_) {
    if (fuzzyLess(best_slack, child->slack_)) {
      best_slack = child->slack_;
    }
  }
  node->slack_ = best_slack;
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
