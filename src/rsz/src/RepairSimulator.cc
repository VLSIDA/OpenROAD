// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "RepairSimulator.hh"

#include <string>

#include "BaseMove.hh"
#include "rsz/Resizer.hh"
#include "utl/Logger.h"

namespace rsz {

using std::string;
using utl::RSZ;

using sta::fuzzyLess;
using sta::Pin;

// Run the actual simulation
std::pair<const Pin*, BaseMove*> RepairSimulator::simulate(
    const Pin* endpoint,
    const std::vector<const Pin*>& pins,
    const std::vector<BaseMove*>& moves,
    int depth,
    float setup_slack_margin)
{
  debugPrint(logger_, RSZ, "repair_simulator", 1, "Started repair simulation");

  // Generate the simulation tree
  SimulationTreeNode* root = new SimulationTreeNode(nullptr, nullptr, 0);
  simulateDFS(endpoint, root, pins, moves, depth, setup_slack_margin);

  // Get the possible slack of root's children
  for (SimulationTreeNode* child : root->children_) {
    trickleUpBestSlack(child);
  }

  // Find the best immediate move from root's children
  SimulationTreeNode* best_child = nullptr;
  Slack best_slack = -sta::INF;
  for (SimulationTreeNode* child : root->children_) {
    if (fuzzyLess(best_slack, child->slack_)) {
      best_slack = child->slack_;
      best_child = child;
    }
  }

  std::pair<const Pin*, BaseMove*> result = {nullptr, nullptr};
  if (best_child != nullptr) {
    result = {best_child->pin_, best_child->move_};
  }

  debugPrint(logger_, RSZ, "repair_simulator", 1, "Finished repair simulation");

  delete root;
  return result;
}

// Recursive DFS simulation helper
void RepairSimulator::simulateDFS(const Pin* endpoint,
                                  SimulationTreeNode* node,
                                  const std::vector<const Pin*>& pins,
                                  const std::vector<BaseMove*>& moves,
                                  int max_depth,
                                  float setup_slack_margin)
{
  if (node->depth_ >= max_depth) {
    return;
  }

  for (const Pin* pin : pins) {
    for (BaseMove* move : moves) {
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
      if (!move->doMove(pin, setup_slack_margin)) {
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
      child->slack_ = violator_collector_->getPathSlackByIndex(endpoint, 0);
      node->children_.push_back(child);
      simulateDFS(endpoint, child, pins, moves, max_depth, setup_slack_margin);
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
