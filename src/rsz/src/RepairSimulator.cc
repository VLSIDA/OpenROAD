// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "RepairSimulator.hh"

#include <string>

#include "BaseMove.hh"
#include "rsz/Resizer.hh"
#include "utl/Logger.h"

namespace rsz {

using std::string;

using sta::fuzzyLess;
using sta::Pin;
using utl::RSZ;

void RepairSimulator::init(const Pin* endpoint,
                           std::vector<const Pin*>& pins,
                           std::vector<BaseMove*>& moves,
                           int depth,
                           float setup_slack_margin)
{
  endpoint_ = endpoint;
  pins_ = &pins;
  moves_ = &moves;
  max_depth_ = depth;
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
}

// Run the actual simulation
void RepairSimulator::simulate()
{
  debugPrint(logger_, RSZ, "repair_simulator", 1, "Started repair simulation");

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

// Recursive DFS simulation helper
void RepairSimulator::simulateDFS(SimulationTreeNode* node)
{
  if (node->depth_ >= max_depth_) {
    return;
  }

  for (const Pin* pin : *pins_) {
    for (BaseMove* move : *moves_) {
      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 2,
                 "Doing {} on {}",
                 move->name(),
                 network_->pathName(pin));

      SimulationTreeNode* child
          = new SimulationTreeNode(pin, move, node->depth_ + 1);

      odb::dbDatabase::beginEco(resizer_->block_);
      if (!move->doMove(pin, setup_slack_margin_)) {
        odb::dbDatabase::undoEco(resizer_->block_);
        delete child;
        debugPrint(logger_,
                   RSZ,
                   "repair_simulator",
                   2,
                   "Rejected {} on {}",
                   move->name(),
                   network_->pathName(pin));
        continue;
      }
      child->slack_ = violator_collector_->getPathSlackByIndex(endpoint_, 0);
      node->children_.push_back(child);
      simulateDFS(child);
      odb::dbDatabase::undoEco(resizer_->block_);

      debugPrint(logger_,
                 RSZ,
                 "repair_simulator",
                 2,
                 "Undoing {} on {}",
                 move->name(),
                 network_->pathName(pin));
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

}  // namespace rsz
