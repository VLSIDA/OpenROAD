// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "RepairSetup.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "BaseMove.hh"
#include "BufferMove.hh"
#include "CloneMove.hh"
#include "MoveTracker.hh"
#include "Rebuffer.hh"
#include "SizeDownMove.hh"
#include "db_sta/dbSta.hh"
#include "sta/Delay.hh"
#include "sta/NetworkClass.hh"
#include "sta/SearchClass.hh"
// This includes SizeUpMatchMove
#include "SizeUpMove.hh"
#include "SplitLoadMove.hh"
#include "SwapPinsMove.hh"
#include "UnbufferMove.hh"
#include "VTSwapMove.hh"
#include "ViolatorCollector.hh"
#include "odb/db.h"
#include "rsz/Resizer.hh"
#include "sta/Corner.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/Fuzzy.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/InputDrive.hh"
#include "sta/Liberty.hh"
#include "sta/Parasitics.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "sta/PortDirection.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/Sta.hh"
#include "sta/TimingArc.hh"
#include "sta/Units.hh"
#include "utl/Logger.h"
#include "utl/mem_stats.h"

namespace rsz {

using std::max;
using std::pair;
using std::string;
using std::vector;
using utl::RSZ;

using sta::Edge;
using sta::fuzzyEqual;
using sta::fuzzyGreater;
using sta::fuzzyGreaterEqual;
using sta::fuzzyLess;
using sta::GraphDelayCalc;
using sta::InstancePinIterator;
using sta::NetConnectedPinIterator;
using sta::PathEndSeq;
using sta::PathExpanded;
using sta::Slew;
using sta::VertexInEdgeIterator;
using sta::VertexOutEdgeIterator;

RepairSetup::RepairSetup(Resizer* resizer) : resizer_(resizer)
{
}

RepairSetup::~RepairSetup() = default;

void RepairSetup::init()
{
  logger_ = resizer_->logger_;
  dbStaState::init(resizer_->sta_);
  db_network_ = resizer_->db_network_;
  estimate_parasitics_ = resizer_->estimate_parasitics_;
  initial_design_area_ = resizer_->computeDesignArea();

  violator_collector_ = std::make_unique<ViolatorCollector>(resizer_);

  // Only create MoveTracker if debug level is enabled
  if (logger_->debugCheck(RSZ, "move_tracker", 1)) {
    move_tracker_ = std::make_unique<MoveTracker>(logger_, sta_);
  }
}

void RepairSetup::setupMoveSequence(const std::vector<MoveType>& sequence,
                                    const bool skip_pin_swap,
                                    const bool skip_gate_cloning,
                                    const bool skip_size_down,
                                    const bool skip_buffering,
                                    const bool skip_buffer_removal,
                                    const bool skip_vt_swap)
{
  if (!sequence.empty()) {
    move_sequence.clear();
    for (MoveType move : sequence) {
      switch (move) {
        case MoveType::BUFFER:
          if (!skip_buffering) {
            move_sequence.push_back(resizer_->buffer_move_.get());
          }
          break;
        case MoveType::UNBUFFER:
          if (!skip_buffer_removal) {
            move_sequence.push_back(resizer_->unbuffer_move_.get());
          }
          break;
        case MoveType::SWAP:
          if (!skip_pin_swap) {
            move_sequence.push_back(resizer_->swap_pins_move_.get());
          }
          break;
        case MoveType::SIZE:
          move_sequence.push_back(resizer_->size_up_move_.get());
          if (!skip_size_down) {
            move_sequence.push_back(resizer_->size_down_move_.get());
          }
          break;
        case MoveType::SIZEUP:
          move_sequence.push_back(resizer_->size_up_move_.get());
          break;
        case MoveType::SIZEDOWN:
          if (!skip_size_down) {
            move_sequence.push_back(resizer_->size_down_move_.get());
          }
          break;
        case MoveType::CLONE:
          if (!skip_gate_cloning) {
            move_sequence.push_back(resizer_->clone_move_.get());
          }
          break;
        case MoveType::SPLIT:
          if (!skip_buffering) {
            move_sequence.push_back(resizer_->split_load_move_.get());
          }
          break;
        case MoveType::VTSWAP_SPEED:
          if (!skip_vt_swap
              && resizer_->lib_data_->sorted_vt_categories.size() > 1) {
            move_sequence.push_back(resizer_->vt_swap_speed_move_.get());
          }
          break;
        case MoveType::SIZEUP_MATCH:
          move_sequence.push_back(resizer_->size_up_match_move_.get());
          break;
      }
    }
  } else {
    move_sequence.clear();
    if (!skip_buffer_removal) {
      move_sequence.push_back(resizer_->unbuffer_move_.get());
    }
    if (!skip_vt_swap && resizer_->lib_data_->sorted_vt_categories.size() > 1) {
      move_sequence.push_back(resizer_->vt_swap_speed_move_.get());
    }
    // Always  have sizing
    move_sequence.push_back(resizer_->size_up_move_.get());
    // Disabled by default for now
    if (!skip_size_down) {
      // move_sequence.push_back(resizer_->size_down_move_.get());
    }
    if (!skip_pin_swap) {
      move_sequence.push_back(resizer_->swap_pins_move_.get());
    }
    if (!skip_buffering) {
      move_sequence.push_back(resizer_->buffer_move_.get());
    }
    if (!skip_gate_cloning) {
      move_sequence.push_back(resizer_->clone_move_.get());
    }
    if (!skip_buffering) {
      move_sequence.push_back(resizer_->split_load_move_.get());
    }
  }

  string repair_moves = "Repair move sequence: ";
  for (auto move : move_sequence) {
    move->init();
    repair_moves += move->name() + string(" ");
  }
  logger_->info(RSZ, 100, repair_moves);
}

bool RepairSetup::repairSetup(const float setup_slack_margin,
                              const double repair_tns_end_percent,
                              const int max_passes,
                              const int max_iterations,
                              const int max_repairs_per_pass,
                              const bool verbose,
                              const std::vector<MoveType>& sequence,
                              const bool skip_pin_swap,
                              const bool skip_gate_cloning,
                              const bool skip_size_down,
                              const bool skip_buffering,
                              const bool skip_buffer_removal,
                              const bool skip_last_gasp,
                              const bool skip_vt_swap,
                              const bool skip_crit_vt_swap)
{
  auto start_time = std::chrono::steady_clock::now();
  bool repaired = false;
  init();
  resizer_->rebuffer_->init();
  // IMPROVE ME: rebuffering always looks at cmd corner
  resizer_->rebuffer_->initOnCorner(sta_->cmdCorner());
  max_repairs_per_pass_ = max_repairs_per_pass;
  resizer_->buffer_moved_into_core_ = false;

  violator_collector_->init(setup_slack_margin);
  violator_collector_->setMaxPassesPerEndpoint(max_passes);

  setupMoveSequence(sequence,
                    skip_pin_swap,
                    skip_gate_cloning,
                    skip_size_down,
                    skip_buffering,
                    skip_buffer_removal,
                    skip_vt_swap);

  // ViolatorCollector has already collected and sorted violating endpoints
  const VertexSet* endpoints = sta_->endpoints();
  int num_viols = violator_collector_->getTotalViolations();

  if (num_viols > 0) {
    debugPrint(logger_,
               RSZ,
               "repair_setup",
               1,
               "Violating endpoints {}/{} {}%",
               num_viols,
               endpoints->size(),
               int(num_viols / double(endpoints->size()) * 100));
    logger_->info(
        RSZ, 94, "Found {} endpoints with setup violations.", num_viols);
  } else {
    // nothing to repair
    logger_->metric("design__instance__count__setup_buffer", 0);
    logger_->info(RSZ, 98, "No setup violations found");
    return false;
  }

  float initial_tns = sta_->totalNegativeSlack(max_);
  float prev_tns = initial_tns;
  logger_->info(RSZ,
                99,
                "Repairing {} out of {} ({:0.2f}%) violating endpoints...",
                violator_collector_->getMaxEndpointCount(),
                num_viols,
                repair_tns_end_percent * 100.0);

  // Ensure that max cap and max fanout violations don't get worse
  sta_->checkCapacitanceLimitPreamble();
  sta_->checkSlewLimitPreamble();
  sta_->checkFanoutLimitPreamble();

  est::IncrementalParasiticsGuard guard(estimate_parasitics_);
  int opto_iteration = 0;

  // Run WNS Phase: WNS-Focused Repair
  repairSetup_WNS(setup_slack_margin,
                  max_passes,
                  max_repairs_per_pass,
                  verbose,
                  opto_iteration,
                  initial_tns,
                  prev_tns);

  if (move_tracker_) {
    move_tracker_->printMoveSummary("WNS Phase Summary");
    move_tracker_->printEndpointSummary("WNS Phase Endpoint Profiler");
    move_tracker_->clear();
  }

  // Run TNS Phase: TNS-Focused Repair
  repairSetup_TNS(setup_slack_margin,
                  max_passes,
                  max_repairs_per_pass,
                  verbose,
                  opto_iteration,
                  initial_tns,
                  prev_tns);

  if (move_tracker_) {
    move_tracker_->printMoveSummary("TNS Phase Summary");
    move_tracker_->printEndpointSummary("TNS Phase Endpoint Profiler");
    move_tracker_->clear();
  }

  if (!skip_crit_vt_swap && !skip_vt_swap
      && resizer_->lib_data_->sorted_vt_categories.size() > 1) {
    // Swap most critical cells to fastest VT

    OptoParams params(setup_slack_margin,
                      verbose,
                      skip_pin_swap,
                      skip_gate_cloning,
                      skip_size_down,
                      skip_buffering,
                      skip_buffer_removal,
                      skip_vt_swap);
    if (swapVTCritCells(params, num_viols)) {
      estimate_parasitics_->updateParasitics();
      sta_->findRequireds();
    }
    if (move_tracker_) {
      move_tracker_->printMoveSummary("VT Swap Phase Summary");
      move_tracker_->clear();
    }
  }

  // Final WNS Phase: Last optimization pass focusing on worst slack
  if (!skip_last_gasp) {
    logger_->info(RSZ, 210, "Final WNS Phase: Last optimization pass...");
    printProgressHeader();

    repairSetup_WNS(setup_slack_margin,
                    max_passes,
                    max_repairs_per_pass,
                    verbose,
                    opto_iteration,
                    initial_tns,
                    prev_tns,
                    ViolatorSortType::SORT_BY_LOAD_DELAY);

    printProgressFooter();
    if (move_tracker_) {
      move_tracker_->printMoveSummary("Final WNS Phase Summary");
      move_tracker_->printEndpointSummary("Final WNS Phase Endpoint Profiler");
      move_tracker_->clear();
    }
  }

  // Print comprehensive optimization reports (only if tracking is enabled)
  if (move_tracker_) {
    // Pass all critical pins to MoveTracker for missed opportunities analysis
    const auto& critical_pins = violator_collector_->getViolatingPins();
    move_tracker_->trackCriticalPins(critical_pins);

    logger_->info(RSZ, 211, "");
    logger_->info(RSZ, 212, "=== Optimization Analysis Reports ===");
    move_tracker_->printSlackDistribution("Pin Slack Distribution");
    move_tracker_->printSuccessReport("Successful Optimizations Report");
    move_tracker_->printFailureReport("Unsuccessful Optimizations Report");
    move_tracker_->printMissedOpportunitiesReport(
        "Missed Opportunities Report");
    logger_->info(RSZ, 213, "");
  }

  int buffer_moves_ = resizer_->buffer_move_->numCommittedMoves();
  int size_up_moves_ = resizer_->size_up_move_->numCommittedMoves();
  int size_down_moves_ = resizer_->size_down_move_->numCommittedMoves();
  int swap_pins_moves_ = resizer_->swap_pins_move_->numCommittedMoves();
  int clone_moves_ = resizer_->clone_move_->numCommittedMoves();
  int split_load_moves_ = resizer_->split_load_move_->numCommittedMoves();
  int unbuffer_moves_ = resizer_->unbuffer_move_->numCommittedMoves();
  int vt_swap_moves_ = resizer_->vt_swap_speed_move_->numCommittedMoves();
  int size_up_match_moves_ = resizer_->size_up_match_move_->numCommittedMoves();

  if (unbuffer_moves_ > 0) {
    repaired = true;
    logger_->info(RSZ, 59, "Removed {} buffers.", unbuffer_moves_);
  }
  if (buffer_moves_ > 0 || split_load_moves_ > 0) {
    repaired = true;
    if (split_load_moves_ == 0) {
      logger_->info(RSZ, 40, "Inserted {} buffers.", buffer_moves_);
    } else {
      logger_->info(RSZ,
                    45,
                    "Inserted {} buffers, {} to split loads.",
                    buffer_moves_ + split_load_moves_,
                    split_load_moves_);
    }
  }
  logger_->metric("design__instance__count__setup_buffer",
                  buffer_moves_ + split_load_moves_);
  if (size_up_moves_ + size_down_moves_ + size_up_match_moves_ + vt_swap_moves_
      > 0) {
    repaired = true;
    logger_->info(RSZ,
                  51,
                  "Resized {} instances: {} up, {} up match, {} down, {} VT",
                  size_up_moves_ + size_up_match_moves_ + size_down_moves_
                      + vt_swap_moves_,
                  size_up_moves_,
                  size_up_match_moves_,
                  size_down_moves_,
                  vt_swap_moves_);
  }
  if (swap_pins_moves_ > 0) {
    repaired = true;
    logger_->info(RSZ, 43, "Swapped pins on {} instances.", swap_pins_moves_);
  }
  if (clone_moves_ > 0) {
    repaired = true;
    logger_->info(RSZ, 49, "Cloned {} instances.", clone_moves_);
  }
  const Slack worst_slack = sta_->worstSlack(max_);
  if (fuzzyLess(worst_slack, setup_slack_margin)) {
    repaired = true;
    logger_->warn(RSZ, 62, "Unable to repair all setup violations.");
  }
  if (resizer_->overMaxArea()) {
    logger_->error(RSZ, 25, "max utilization reached.");
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                          end_time - start_time)
                          .count();
  debugPrint(logger_,
             RSZ,
             "repair_setup",
             1,
             "Total repair_timing time: {:.2f} seconds",
             elapsed_time / 1000.0);

  return repaired;
}

// For testing.
void RepairSetup::repairSetup(const Pin* end_pin)
{
  init();
  max_repairs_per_pass_ = 1;

  Vertex* vertex = graph_->pinLoadVertex(end_pin);
  const Slack slack = sta_->vertexSlack(vertex, max_);
  Path* path = sta_->vertexWorstSlackPath(vertex, max_);

  move_sequence.clear();
  move_sequence = {resizer_->unbuffer_move_.get(),
                   resizer_->vt_swap_speed_move_.get(),
                   resizer_->size_down_move_.get(),
                   resizer_->size_up_move_.get(),
                   resizer_->swap_pins_move_.get(),
                   resizer_->buffer_move_.get(),
                   resizer_->clone_move_.get(),
                   resizer_->split_load_move_.get()};

  {
    est::IncrementalParasiticsGuard guard(estimate_parasitics_);
    repairPath(path, slack, 0.0);
  }

  int unbuffer_moves_ = resizer_->unbuffer_move_->numCommittedMoves();
  if (unbuffer_moves_ > 0) {
    logger_->info(RSZ, 61, "Removed {} buffers.", unbuffer_moves_);
  }
  int buffer_moves_ = resizer_->buffer_move_->numCommittedMoves();
  int split_load_moves_ = resizer_->split_load_move_->numMoves();
  if (buffer_moves_ + split_load_moves_ > 0) {
    logger_->info(
        RSZ, 30, "Inserted {} buffers.", buffer_moves_ + split_load_moves_);
  }
  int size_up_moves_ = resizer_->size_up_move_->numMoves();
  int size_down_moves_ = resizer_->size_down_move_->numMoves();
  if (size_up_moves_ + size_down_moves_ > 0) {
    logger_->info(RSZ,
                  38,
                  "Resized {} instances, {} sized up, {} sized down.",
                  size_up_moves_ + size_down_moves_,
                  size_up_moves_,
                  size_down_moves_);
  }
  int swap_pins_moves_ = resizer_->swap_pins_move_->numMoves();
  if (swap_pins_moves_ > 0) {
    logger_->info(RSZ, 44, "Swapped pins on {} instances.", swap_pins_moves_);
  }
}

int RepairSetup::fanout(Vertex* vertex)
{
  int fanout = 0;
  VertexOutEdgeIterator edge_iter(vertex, graph_);
  while (edge_iter.hasNext()) {
    Edge* edge = edge_iter.next();
    // Disregard output->output timing arcs
    if (edge->isWire()) {
      fanout++;
    }
  }
  return fanout;
}

void RepairSetup::hitDebugCheckpoint()
{
  logger_->info(RSZ, 9999, "Hit checkpoint");
}

bool RepairSetup::repairPins(
    const std::vector<const Pin*>& pins,
    const float setup_slack_margin,
    const std::map<const Pin*, std::set<BaseMove*>>* rejected_moves,
    std::vector<std::pair<const Pin*, BaseMove*>>* chosen_moves)
{
  int changed = 0;

  if (pins.size() == 0) {
    return false;
  }
  int repairs_per_pass
      = violator_collector_->repairsPerPass(max_repairs_per_pass_);
  if (fallback_) {
    repairs_per_pass = 1;
  }
  debugPrint(
      logger_,
      RSZ,
      "repair_setup",
      3,
      "Path slack: {}, repairs: {}, load_delays: {}",
      delayAsString(violator_collector_->getCurrentEndpointSlack(), sta_, 3),
      repairs_per_pass,
      pins.size());
  for (const auto& drvr_pin : pins) {
    if (changed >= repairs_per_pass) {
      break;
    }
    // Track this pin as a violator before attempting moves
    Vertex* drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
    LibertyPort* drvr_port = network_->libertyPort(drvr_pin);
    LibertyCell* drvr_cell = drvr_port ? drvr_port->libertyCell() : nullptr;

    // Get detailed information for MoveTracker reporting
    string gate_type = drvr_cell ? drvr_cell->name() : "unknown";
    float load_delay = 0.0;
    float intrinsic_delay = 0.0;

    // Try to get delay data from ViolatorCollector cache first
    if (!violator_collector_->getPinData(
            drvr_pin, load_delay, intrinsic_delay)) {
      // Calculate delays using ViolatorCollector's getEffortDelays method
      auto [load, intrinsic] = violator_collector_->getEffortDelays(drvr_pin);
      load_delay = load;
      intrinsic_delay = intrinsic;
    }

    Slack pin_slack = sta_->vertexSlack(drvr_vertex, max_);
    Slack endpoint_slack = violator_collector_->getCurrentEndpointSlack();

    if (move_tracker_) {
      move_tracker_->trackViolatorWithInfo(drvr_pin,
                                           gate_type,
                                           load_delay,
                                           intrinsic_delay,
                                           pin_slack,
                                           endpoint_slack);
    }
    const int fanout = this->fanout(drvr_vertex);
    debugPrint(logger_,
               RSZ,
               "repair_setup",
               3,
               "{} {} fanout = {} ",
               network_->pathName(drvr_pin),
               drvr_cell ? drvr_cell->name() : "none",
               fanout);

    for (BaseMove* move : move_sequence) {
      // Skip this move if it's been tried and rejected for this pin
      if (rejected_moves) {
        auto it = rejected_moves->find(drvr_pin);
        if (it != rejected_moves->end() && it->second.count(move) > 0) {
          debugPrint(logger_,
                     RSZ,
                     "repair_setup",
                     4,
                     "Skipping {} for {} (previously rejected)",
                     move->name(),
                     network_->pathName(drvr_pin));
          continue;
        }
      }

      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 4,
                 "Considering {} for {}",
                 move->name(),
                 network_->pathName(drvr_pin));

      if (move_tracker_) {
        move_tracker_->trackMove(
            drvr_pin, string(move->name()), rsz::MoveStateType::ATTEMPT);
      }
      if (move->doMove(drvr_pin, setup_slack_margin)) {
        // Record this successful move
        if (chosen_moves) {
          chosen_moves->push_back({drvr_pin, move});
        }

        if (move == resizer_->unbuffer_move_.get()) {
          // Only allow one unbuffer move per pass to
          // prevent the use-after-free error of multiple buffer removals.
          changed += repairs_per_pass;
        } else {
          changed++;
        }
        // Move on to the next gate
        break;
      }

      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 4,
                 "Move {} failed for {}",
                 move->name(),
                 network_->pathName(drvr_pin));
    }
  }
  return changed > 0;
}

bool RepairSetup::repairPath(Path* path,
                             const Slack path_slack,
                             const float setup_slack_margin)
{
  PathExpanded expanded(path, sta_);
  int changed = 0;

  if (expanded.size() > 1) {
    const int path_length = expanded.size();
    vector<pair<int, Delay>> load_delays;
    const int start_index = expanded.startIndex();
    const DcalcAnalysisPt* dcalc_ap = path->dcalcAnalysisPt(sta_);
    const int lib_ap = dcalc_ap->libertyIndex();
    // Find load delay for each gate in the path.
    for (int i = start_index; i < path_length; i++) {
      const Path* path = expanded.path(i);
      Vertex* path_vertex = path->vertex(sta_);
      const Pin* path_pin = path->pin(sta_);
      if (i > 0 && path_vertex->isDriver(network_)
          && !network_->isTopLevelPort(path_pin)) {
        const TimingArc* prev_arc = path->prevArc(sta_);
        const TimingArc* corner_arc = prev_arc->cornerArc(lib_ap);
        Edge* prev_edge = path->prevEdge(sta_);
        const Delay load_delay
            = graph_->arcDelay(prev_edge, prev_arc, dcalc_ap->index())
              // Remove intrinsic delay to find load dependent delay.
              - corner_arc->intrinsicDelay();
        load_delays.emplace_back(i, load_delay);
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   3,
                   "{} load_delay = {} intrinsic_delay = {}",
                   path_vertex->name(network_),
                   delayAsString(load_delay, sta_, 3),
                   delayAsString(corner_arc->intrinsicDelay(), sta_, 3));
      }
    }

    sort(
        load_delays.begin(),
        load_delays.end(),
        [](pair<int, Delay> pair1, pair<int, Delay> pair2) {
          return pair1.second > pair2.second
                 || (pair1.second == pair2.second && pair1.first > pair2.first);
        });
    // Attack gates with largest load delays first.
    int repairs_per_pass = 1;
    if (max_viol_ - min_viol_ != 0.0) {
      repairs_per_pass
          += std::round((max_repairs_per_pass_ - 1) * (-path_slack - min_viol_)
                        / (max_viol_ - min_viol_));
    }
    if (fallback_) {
      repairs_per_pass = 1;
    }
    debugPrint(logger_,
               RSZ,
               "repair_setup",
               3,
               "Path slack: {}, repairs: {}, load_delays: {}",
               delayAsString(path_slack, sta_, 3),
               repairs_per_pass,
               load_delays.size());
    for (const auto& [drvr_index, ignored] : load_delays) {
      if (changed >= repairs_per_pass) {
        break;
      }
      const Path* drvr_path = expanded.path(drvr_index);
      Vertex* drvr_vertex = drvr_path->vertex(sta_);
      const Pin* drvr_pin = drvr_vertex->pin();
      LibertyPort* drvr_port = network_->libertyPort(drvr_pin);
      LibertyCell* drvr_cell = drvr_port ? drvr_port->libertyCell() : nullptr;
      const int fanout = this->fanout(drvr_vertex);
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 3,
                 "{} {} fanout = {} drvr_index = {}",
                 network_->pathName(drvr_pin),
                 drvr_cell ? drvr_cell->name() : "none",
                 fanout,
                 drvr_index);

      for (BaseMove* move : move_sequence) {
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   1,
                   "Considering {} for {}",
                   move->name(),
                   network_->pathName(drvr_pin));

        if (move->doMove(drvr_pin, setup_slack_margin)) {
          if (move == resizer_->unbuffer_move_.get()) {
            // Only allow one unbuffer move per pass to
            // prevent the use-after-free error of multiple buffer removals.
            changed += repairs_per_pass;
          } else {
            changed++;
          }
          // Move on to the next gate
          break;
        }
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   2,
                   "Move {} failed for {}",
                   move->name(),
                   network_->pathName(drvr_pin));
      }
    }
  }
  return changed > 0;
}

void RepairSetup::printProgress(const int iteration,
                                const bool force,
                                const bool end,
                                const bool last_gasp,
                                const bool phase2) const
{
  const bool start = iteration == 0;

  if (start && !end) {
    logger_->report(
        "   Iter   | Removed | Resized | Inserted | Cloned |  Pin  |"
        "   Area   |    WNS   |   TNS      |  Viol  | Worst");
    logger_->report(
        "   *TNS    | Buffers |  Gates  | Buffers  |  Gates | Swaps |"
        "          |          |            | Endpts | Endpt");
    logger_->report(
        "-----------------------------------------------------------"
        "---------------------------------------------------");
  }

  if (iteration % print_interval_ == 0 || force || end) {
    Slack wns;
    Vertex* worst_vertex;
    sta_->worstSlack(max_, wns, worst_vertex);
    const Slack tns = sta_->totalNegativeSlack(max_);

    std::string itr_field
        = fmt::format("{}{}", iteration, (phase2 || last_gasp ? "*" : ""));
    if (end) {
      itr_field = "final";
    }

    const double design_area = resizer_->computeDesignArea();
    const double area_growth = design_area - initial_design_area_;
    double area_growth_percent = std::numeric_limits<double>::infinity();
    if (std::abs(initial_design_area_) > 0.0) {
      area_growth_percent = area_growth / initial_design_area_ * 100.0;
    }

    // This actually prints both committed and pending moves, so the moves could
    // could go down if a pass is rejected and restored by the ECO.
    logger_->report(
        "{: >9s} | {: >7d} | {: >7d} | {: >8d} | {: >6d} | {: >5d} "
        "| {: >+7.1f}% | {: >8s} | {: >10s} | {: >6d} | {}",
        itr_field,
        resizer_->unbuffer_move_->numMoves(),
        resizer_->size_up_move_->numMoves()
            + resizer_->size_down_move_->numMoves()
            + resizer_->size_up_match_move_->numMoves()
            + resizer_->vt_swap_speed_move_->numMoves(),
        resizer_->buffer_move_->numMoves()
            + resizer_->split_load_move_->numMoves(),
        resizer_->clone_move_->numMoves(),
        resizer_->swap_pins_move_->numMoves(),
        area_growth_percent,
        delayAsString(wns, sta_, 3),
        delayAsString(tns, sta_, 1),
        max(0, violator_collector_->getNumViolatingEndpoints()),
        worst_vertex != nullptr ? worst_vertex->name(network_) : "");

    debugPrint(logger_, RSZ, "memory", 1, "RSS = {}", utl::getCurrentRSS());
  }

  if (end) {
    logger_->report(
        "-----------------------------------------------------------"
        "---------------------------------------------------");
  }
}

void RepairSetup::printProgressHeader() const
{
  logger_->report(
      "   Iter   | Removed | Resized | Inserted | Cloned |  Pin  |"
      "   Area   |    WNS   |   TNS      |  Viol  | Worst");
  logger_->report(
      "   *TNS    | Buffers |  Gates  | Buffers  |  Gates | Swaps |"
      "          |          |            | Endpts | Endpt");
  logger_->report(
      "-----------------------------------------------------------"
      "---------------------------------------------------");
}

void RepairSetup::printProgressFooter() const
{
  logger_->report(
      "-----------------------------------------------------------"
      "---------------------------------------------------");
}

// Terminate progress if incremental fix rate within an opto interval falls
// below the threshold.   Bump up the threshold after each large opto
// interval.
bool RepairSetup::terminateProgress(const int iteration,
                                    const float initial_tns,
                                    float& prev_tns,
                                    float& fix_rate_threshold,
                                    // for debug only
                                    const int endpt_index,
                                    const int num_endpts)
{
  if (iteration % opto_large_interval_ == 0) {
    fix_rate_threshold *= 2.0;
  }
  if (iteration % opto_small_interval_ == 0) {
    float curr_tns = sta_->totalNegativeSlack(max_);
    float inc_fix_rate = (prev_tns - curr_tns) / initial_tns;
    prev_tns = curr_tns;
    if (iteration > 1000  // allow for some initial fixing for 1000 iterations
        && inc_fix_rate < fix_rate_threshold) {
      // clang-format off
      debugPrint(logger_, RSZ, "repair_setup", 1, "bailing out at iter {}"
                 " because incr fix rate {:0.2f}% is < {:0.2f}% [endpt {}/{}]",
                 iteration, inc_fix_rate*100, fix_rate_threshold*100,
                 endpt_index, num_endpts);
      // clang-format on
      return true;
    }
  }
  return false;
}

// Perform some last fixing based on sizing only.
// This is a greedy opto that does not degrade WNS or TNS.
void RepairSetup::repairSetupLastGasp(const OptoParams& params)
{
  move_sequence.clear();
  if (!params.skip_vt_swap) {
    move_sequence.push_back(resizer_->vt_swap_speed_move_.get());
  }
  move_sequence.push_back(resizer_->size_up_match_move_.get());
  move_sequence.push_back(resizer_->size_up_move_.get());
  if (!params.skip_pin_swap) {
    move_sequence.push_back(resizer_->swap_pins_move_.get());
  }

  // Re-collect violating endpoints for last gasp (init also resets passes)
  violator_collector_->init(params.setup_slack_margin);

  float curr_tns = sta_->totalNegativeSlack(max_);
  if (fuzzyGreaterEqual(curr_tns, 0)) {
    // clang-format off
    debugPrint(logger_, RSZ, "repair_setup", 1, "last gasp is bailing out "
               "because TNS is {:0.2f}", curr_tns);
    // clang-format on
    return;
  }

  int max_end_count = violator_collector_->getMaxEndpointCount();
  if (max_end_count == 0) {
    // clang-format off
    debugPrint(logger_, RSZ, "repair_setup", 1, "last gasp is bailing out "
               "because there are no violating endpoints");
    // clang-format on
    return;
  }
  // clang-format off
  debugPrint(logger_, RSZ, "repair_setup", 1, "{} violating endpoints remain",
             max_end_count);
  // clang-format on
  int opto_iteration = params.iteration;

  // Print last gasp phase header and table
  logger_->info(RSZ, 209, "Last Gasp: Greedy sizing optimization...");
  printProgressHeader();
  printProgress(opto_iteration, false, false, true);

  float prev_tns = curr_tns;
  violator_collector_->setToEndpoint(0);
  Slack curr_worst_slack = violator_collector_->getCurrentEndpointSlack();
  Slack prev_worst_slack = curr_worst_slack;
  int tns_no_progress_count = 0;
  float fix_rate_threshold = inc_fix_rate_threshold_;
  constexpr int max_tns_no_progress = 4;
  while (violator_collector_->hasMoreEndpoints()) {
    fallback_ = false;
    Slack end_slack = violator_collector_->getCurrentEndpointSlack();
    int end_index = violator_collector_->getCurrentEndpointIndex() + 1;
    Slack worst_slack;
    Vertex* worst_vertex;
    sta_->worstSlack(max_, worst_slack, worst_vertex);
    int pass = violator_collector_->getCurrentPass() + 1;
    while (true) {
      resizer_->journalBegin();
      opto_iteration++;
      if (terminateProgress(opto_iteration,
                            params.initial_tns,
                            prev_tns,
                            fix_rate_threshold,
                            end_index,
                            max_end_count)) {
        tns_no_progress_count++;
        if (tns_no_progress_count >= max_tns_no_progress) {
          resizer_->journalEnd();
          break;
        }
      }
      if (opto_iteration % opto_small_interval_ == 0) {
        tns_no_progress_count = 0;
      }
      if (params.verbose || opto_iteration == 1) {
        printProgress(opto_iteration, false, false, true);
      }
      if (end_slack > params.setup_slack_margin) {
        resizer_->journalEnd();
        break;
      }

      vector<const Pin*> viol_pins = violator_collector_->collectViolators(
          3, 100, ViolatorSortType::SORT_BY_LOAD_DELAY);

      const bool changed = repairPins(viol_pins, params.setup_slack_margin);
      pass = violator_collector_->getCurrentPass();

      if (!changed) {
        if (pass != 1) {
          resizer_->journalRestore();
        } else {
          resizer_->journalEnd();
        }
        break;
      }
      estimate_parasitics_->updateParasitics();
      sta_->findRequireds();
      sta_->worstSlack(max_, curr_worst_slack, worst_vertex);
      curr_tns = sta_->totalNegativeSlack(max_);

      // Accept only moves that improve both WNS and TNS
      if (fuzzyGreaterEqual(curr_worst_slack, prev_worst_slack)
          && fuzzyGreaterEqual(curr_tns, prev_tns)) {
        // clang-format off
        debugPrint(logger_, RSZ, "repair_setup", 1, "sizing move accepted for "
                   "endpoint {} pass {} because WNS improved to {:0.3f} and "
                   "TNS improved to {:0.3f}",
                   end_index, pass, curr_worst_slack, curr_tns);
        // clang-format on
        prev_worst_slack = curr_worst_slack;
        prev_tns = curr_tns;
        resizer_->journalEnd();
      } else {
        fallback_ = true;
        resizer_->journalRestore();
        break;
      }

      if (resizer_->overMaxArea()) {
        resizer_->journalEnd();
        break;
      }
    }  // while pass <= max_last_gasp_passes_
    if (params.verbose || opto_iteration == 1) {
      printProgress(opto_iteration, true, false, true);
    }
    if (tns_no_progress_count >= max_tns_no_progress) {
      // clang-format off
      debugPrint(logger_, RSZ, "repair_setup", 1, "bailing out of last gasp fixing "
                 "due to no TNS progress for {} opto cycles", tns_no_progress_count);
      // clang-format on
      break;
    }
    violator_collector_->advanceToNextEndpoint();
  }  // while hasMoreEndpoints

  // Print last gasp phase completion
  printProgress(opto_iteration, true, false, true);
  printProgressFooter();
}

// WNS Phase: WNS-Focused Repair
// Focus on the worst slack endpoint at any time, switching dynamically when a
// different endpoint becomes worst.
void RepairSetup::repairSetup_WNS(const float setup_slack_margin,
                                  const int max_passes_per_endpoint,
                                  const int max_repairs_per_pass,
                                  const bool verbose,
                                  int& opto_iteration,
                                  const float initial_tns,
                                  float& prev_tns,
                                  const ViolatorSortType sort_type)
{
  constexpr int digits = 3;

  // Initialize WNS Phase tracking
  wns_no_progress_count_ = 0;
  rejected_pin_moves_current_endpoint_.clear();

  // WNS Phase: Track passes for endpoints that have been the worst (WNS)
  // endpoint
  std::map<const Pin*, int> wns_endpoint_pass_counts_;

  // Get initial worst endpoint
  Slack worst_slack;
  Vertex* worst_endpoint;
  sta_->worstSlack(max_, worst_slack, worst_endpoint);

  if (!worst_endpoint) {
    debugPrint(
        logger_, RSZ, "repair_setup", 1, "WNS Phase: No worst endpoint found");
    return;
  }

  // Print phase header and table
  debugPrint(logger_,
             RSZ,
             "repair_setup",
             1,
             "WNS Phase: Focusing on worst slack path...");
  printProgress(opto_iteration, false, false, false);

  // Capture initial slack distribution after printing initial progress
  // This captures the same state shown in iteration 0 of the progress table
  if (move_tracker_) {
    move_tracker_->captureInitialSlackDistribution();
  }

  // Initialize the violator collector with the actual worst endpoint
  Vertex* current_endpoint = worst_endpoint;
  violator_collector_->useWorstEndpoint(worst_endpoint);

  // Track initial endpoint in MoveTracker
  if (move_tracker_) {
    move_tracker_->setCurrentEndpoint(current_endpoint->pin());
  }

  overall_no_progress_count_ = 0;
  float fix_rate_threshold = inc_fix_rate_threshold_;
  constexpr int max_no_progress = 4;
  const int phase1_start_iteration = opto_iteration;

  // Main WNS Phase loop
  while (true) {
    // Get current worst endpoint
    sta_->worstSlack(max_, worst_slack, worst_endpoint);

    if (!worst_endpoint) {
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 1,
                 "WNS Phase: No worst endpoint, exiting");
      break;
    }

    // Check if we should switch to a different endpoint
    if (current_endpoint != worst_endpoint) {
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 1,
                 "WNS Phase: Switching from {} to {}",
                 current_endpoint->name(network_),
                 worst_endpoint->name(network_));
      current_endpoint = worst_endpoint;
      violator_collector_->useWorstEndpoint(worst_endpoint);
      rejected_pin_moves_current_endpoint_
          .clear();  // Clear rejected moves for new endpoint

      // Track new endpoint in MoveTracker
      const Pin* endpoint_pin = current_endpoint->pin();
      if (move_tracker_) {
        move_tracker_->setCurrentEndpoint(endpoint_pin);
      }

      // Print progress when switching to a new endpoint
      printProgress(opto_iteration, true, false, false, false);
    }

    const Pin* endpoint_pin = current_endpoint->pin();

    // Ensure this endpoint is tracked (initialize to 0 if new)
    if (wns_endpoint_pass_counts_.find(endpoint_pin)
        == wns_endpoint_pass_counts_.end()) {
      wns_endpoint_pass_counts_[endpoint_pin] = 0;
    }

    // Check if current WNS endpoint has exceeded its pass limit
    if (wns_endpoint_pass_counts_[endpoint_pin] >= max_passes_per_endpoint) {
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 1,
                 "WNS Phase: WNS endpoint {} exceeded pass limit {}, exiting",
                 network_->pathName(endpoint_pin),
                 max_passes_per_endpoint);
      break;
    }

    // Check if we've exceeded the maximum iterations for WNS Phase
    // This grows as we see more unique endpoints.
    const int phase1_iterations = opto_iteration - phase1_start_iteration;
    const int num_wns_endpoints_seen = wns_endpoint_pass_counts_.size();
    const int max_phase1_iterations
        = num_wns_endpoints_seen * max_passes_per_endpoint;
    if (phase1_iterations >= max_phase1_iterations) {
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 1,
                 "WNS Phase: Reached maximum iterations {} ({} endpoints × {} "
                 "passes), exiting",
                 max_phase1_iterations,
                 num_wns_endpoints_seen,
                 max_passes_per_endpoint);
      break;
    }

    // Check WNS termination
    if (worst_slack >= setup_slack_margin) {
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 1,
                 "WNS Phase: WNS {} meets slack margin {}, done",
                 delayAsString(worst_slack, sta_, digits),
                 delayAsString(setup_slack_margin, sta_, digits));
      printProgress(opto_iteration, true, false, false);
      break;
    }

    // Check TNS termination
    if (terminateProgress(
            opto_iteration, initial_tns, prev_tns, fix_rate_threshold, 0, 1)) {
      overall_no_progress_count_++;
      if (overall_no_progress_count_ >= max_no_progress) {
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   1,
                   "WNS Phase: No TNS progress for {} cycles, exiting",
                   overall_no_progress_count_);
        break;
      }
    }

    if (opto_iteration % opto_small_interval_ == 0) {
      overall_no_progress_count_ = 0;
    }

    // Collect violators from worst path only
    fallback_ = false;
    Slack prev_end_slack = violator_collector_->getCurrentEndpointSlack();
    Slack prev_worst_slack = worst_slack;
    float prev_tns_local = sta_->totalNegativeSlack(max_);

    resizer_->journalBegin();
    opto_iteration++;

    if (verbose || opto_iteration % print_interval_ == 0) {
      printProgress(opto_iteration, false, false, false);
    }

    // Dynamically increase numPathsPerEndpoint from 1 to 3 as we make less
    // progress. When we're struggling, looking at more paths can help find
    // opportunities. Map [0, max_no_progress-1] evenly to [1, 3]
    constexpr int maxPaths = 3;
    const int numPathsPerEndpoint = std::min(
        maxPaths,
        1 + (overall_no_progress_count_ * maxPaths / max_no_progress));
    if (numPathsPerEndpoint > 1) {
      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 2,
                 "WNS Phase: Increasing numPathsPerEndpoint to {} due to {} no "
                 "progress cycles",
                 numPathsPerEndpoint,
                 overall_no_progress_count_);
    }

    vector<const Pin*> viol_pins
        = violator_collector_->collectViolators(numPathsPerEndpoint, -1, sort_type);

    // Track which (pin, move) combinations succeeded
    vector<std::pair<const Pin*, BaseMove*>> chosen_moves;

    const bool changed = repairPins(viol_pins,
                                    setup_slack_margin,
                                    &rejected_pin_moves_current_endpoint_,
                                    &chosen_moves);

    if (!changed) {
      resizer_->journalEnd();
      wns_endpoint_pass_counts_[endpoint_pin]++;

      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 1,
                 "WNS Phase: No changes possible - all moves on critical path "
                 "have been tried, exiting");
      break;
    }

    // Update parasitics and timing
    estimate_parasitics_->updateParasitics();
    sta_->findRequireds();

    Slack end_slack = violator_collector_->getCurrentEndpointSlack();
    Slack new_wns;
    Vertex* new_worst_vertex;
    sta_->worstSlack(max_, new_wns, new_worst_vertex);
    float new_tns = sta_->totalNegativeSlack(max_);

    // Accept if WNS improved OR (WNS same AND (endpoint improved OR TNS improved))
    const bool wns_improved = fuzzyGreater(new_wns, prev_worst_slack);
    const bool wns_same = fuzzyEqual(new_wns, prev_worst_slack);
    const bool endpoint_improved = fuzzyGreater(end_slack, prev_end_slack);
    const bool tns_improved = fuzzyGreater(new_tns, prev_tns_local);
    const bool better = wns_improved
                        || (wns_same && (endpoint_improved || tns_improved));

    debugPrint(
        logger_,
        RSZ,
        "repair_setup",
        2,
        "WNS Phase: end_slack={} WNS={} prev_WNS={} TNS={} prev_TNS={} {}",
        delayAsString(end_slack, sta_, digits),
        delayAsString(new_wns, sta_, digits),
        delayAsString(prev_worst_slack, sta_, digits),
        delayAsString(new_tns, sta_, digits),
        delayAsString(prev_tns_local, sta_, digits),
        better ? "ACCEPT" : "REJECT");

    wns_endpoint_pass_counts_[endpoint_pin]++;

    if (better) {
      if (move_tracker_) {
        move_tracker_->commitMoves();
      }
      resizer_->journalEnd();
    } else {
      if (move_tracker_) {
        move_tracker_->rejectMoves();
      }
      resizer_->journalRestore();
      fallback_ = true;

      // Mark the chosen (pin, move) combinations as rejected since overall
      // change was rejected
      for (const auto& [pin, move] : chosen_moves) {
        rejected_pin_moves_current_endpoint_[pin].insert(move);
      }

      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 2,
                 "WNS Phase: Marked {} (pin,move) pairs as rejected",
                 chosen_moves.size());
    }

    if (resizer_->overMaxArea()) {
      debugPrint(
          logger_, RSZ, "repair_setup", 1, "WNS Phase: Over max area, exiting");
      break;
    }
  }

  // Print phase completion (no footer - continue to TNS Phase in same table)
  printProgress(opto_iteration, true, false, false);

  Slack final_wns;
  Vertex* final_worst;
  sta_->worstSlack(max_, final_wns, final_worst);
  float final_tns = sta_->totalNegativeSlack(max_);

  debugPrint(logger_,
             RSZ,
             "repair_setup",
             1,
             "WNS Phase complete. WNS: {}, TNS: {}",
             delayAsString(final_wns, sta_, digits),
             delayAsString(final_tns, sta_, 1));
}

// TNS Phase: TNS-Focused Repair
// Work on all violating endpoints sequentially, trying to improve each one.
void RepairSetup::repairSetup_TNS(const float setup_slack_margin,
                                  const int max_passes_per_endpoint,
                                  const int max_repairs_per_pass,
                                  const bool verbose,
                                  int& opto_iteration,
                                  const float initial_tns,
                                  float& prev_tns,
                                  const ViolatorSortType sort_type)
{
  constexpr int digits = 3;

  // Initialize TNS Phase tracking
  overall_no_progress_count_ = 0;

  // Reset endpoint tracking from WNS Phase
  violator_collector_->resetEndpointPasses();

  // Print phase header (no table header - continue in same table from WNS
  // Phase)
  debugPrint(logger_,
             RSZ,
             "repair_setup",
             1,
             "TNS Phase: Focusing on all violating endpoints...");

  // Mark the start of TNS Phase in the table with an asterisk
  printProgress(opto_iteration, false, false, false, true);

  // Track global WNS at start of TNS phase - must not degrade below this
  Slack initial_tns_phase_wns;
  Vertex* initial_tns_phase_worst;
  sta_->worstSlack(max_, initial_tns_phase_wns, initial_tns_phase_worst);

  // Track previous pass WNS for comparison
  Slack prev_global_wns = initial_tns_phase_wns;

  // Collect all violating endpoints once at the start
  violator_collector_->collectViolatingEndpoints();
  int max_endpoint_count = violator_collector_->getMaxEndpointCount();

  if (max_endpoint_count == 0) {
    debugPrint(logger_,
               RSZ,
               "repair_setup",
               1,
               "TNS Phase: No violating endpoints, exiting");
    return;
  }

  debugPrint(
      logger_,
      RSZ,
      "repair_setup",
      1,
      "TNS Phase: Processing {} violating endpoints (skipping worst endpoint)",
      max_endpoint_count);

  // Process each endpoint sequentially, starting from index 1 to skip worst
  // endpoint
  for (int endpoint_index = 1; endpoint_index < max_endpoint_count;
       endpoint_index++) {
    violator_collector_->setToEndpoint(endpoint_index);
    Vertex* endpoint = violator_collector_->getCurrentEndpoint();

    if (!endpoint) {
      continue;
    }

    const Pin* endpoint_pin = endpoint->pin();
    Slack endpoint_slack = sta_->vertexSlack(endpoint, max_);

    if (endpoint_slack >= setup_slack_margin) {
      debugPrint(
          logger_,
          RSZ,
          "repair_setup",
          2,
          "TNS Phase: Endpoint {} (index {}) already meets timing, skipping",
          network_->pathName(endpoint_pin),
          endpoint_index);
      continue;
    }

    // Track this endpoint in MoveTracker
    if (move_tracker_) {
      move_tracker_->setCurrentEndpoint(endpoint_pin);
    }

    debugPrint(logger_,
               RSZ,
               "repair_setup",
               1,
               "TNS Phase: Working on endpoint {} (index {}), slack = {}",
               network_->pathName(endpoint_pin),
               endpoint_index,
               delayAsString(endpoint_slack, sta_, digits));

    printProgress(opto_iteration, true, false, false, true);

    // Try multiple passes on this endpoint
    int pass = 1;
    int decreasing_slack_passes = 0;
    Slack prev_endpoint_slack = endpoint_slack;
    resizer_->journalBegin();

    while (pass <= max_passes_per_endpoint) {
      // Collect violating pins from this endpoint's critical path
      vector<const Pin*> viol_pins
          = violator_collector_->collectViolators(1, -1, sort_type);

      if (viol_pins.empty()) {
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   2,
                   "TNS Phase: No violating pins for endpoint {}, pass {}",
                   network_->pathName(endpoint_pin),
                   pass);
        break;
      }

      const bool changed = repairPins(viol_pins, setup_slack_margin);

      if (!changed) {
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   2,
                   "TNS Phase: No changes for endpoint {}, pass {}",
                   network_->pathName(endpoint_pin),
                   pass);
        resizer_->journalEnd();
        break;
      }

      // Update parasitics and timing
      estimate_parasitics_->updateParasitics();
      sta_->findRequireds();

      endpoint_slack = sta_->vertexSlack(endpoint, max_);
      Slack global_wns;
      Vertex* global_wns_vertex;
      sta_->worstSlack(max_, global_wns, global_wns_vertex);

      // Accept if WNS improves OR (WNS same AND endpoint improves)
      // BUT never accept if WNS gets worse than initial TNS phase WNS
      const bool wns_improved = fuzzyGreater(global_wns, prev_global_wns);
      const bool wns_same = fuzzyEqual(global_wns, prev_global_wns);
      const bool endpoint_improved
          = fuzzyGreater(endpoint_slack, prev_endpoint_slack);
      const bool wns_not_worse_than_initial
          = fuzzyGreaterEqual(global_wns, initial_tns_phase_wns);
      const bool better = (wns_improved || (wns_same && endpoint_improved))
                          && wns_not_worse_than_initial;

      debugPrint(logger_,
                 RSZ,
                 "repair_setup",
                 2,
                 "TNS Phase: pass {} slack {} -> {}, WNS {} -> {}, {}",
                 pass,
                 delayAsString(prev_endpoint_slack, sta_, digits),
                 delayAsString(endpoint_slack, sta_, digits),
                 delayAsString(prev_global_wns, sta_, digits),
                 delayAsString(global_wns, sta_, digits),
                 better ? "ACCEPT" : "REJECT");

      if (better) {
        if (endpoint_slack > setup_slack_margin) {
          debugPrint(logger_,
                     RSZ,
                     "repair_setup",
                     1,
                     "TNS Phase: Endpoint {} meets timing, done",
                     network_->pathName(endpoint_pin));
          resizer_->journalEnd();
          break;
        }
        prev_endpoint_slack = endpoint_slack;
        prev_global_wns = global_wns;
        decreasing_slack_passes = 0;
        resizer_->journalEnd();
        resizer_->journalBegin();
      } else {
        // Allow slack to decrease to escape local minima
        decreasing_slack_passes++;
        if (decreasing_slack_passes > decreasing_slack_max_passes_) {
          debugPrint(logger_,
                     RSZ,
                     "repair_setup",
                     2,
                     "TNS Phase: Decreasing slack for {} passes, stopping",
                     decreasing_slack_passes);
          resizer_->journalRestore();
          break;
        }
      }

      if (resizer_->overMaxArea()) {
        debugPrint(logger_,
                   RSZ,
                   "repair_setup",
                   1,
                   "TNS Phase: Over max area, exiting");
        resizer_->journalEnd();
        printProgress(opto_iteration, true, true, false, true);
        printProgressFooter();
        return;
      }

      pass++;
    }

    resizer_->journalEnd();
  }

  // Print phase completion
  printProgress(opto_iteration, true, true, false, true);

  Slack final_wns;
  Vertex* final_worst;
  sta_->worstSlack(max_, final_wns, final_worst);
  float final_tns = sta_->totalNegativeSlack(max_);

  debugPrint(logger_,
             RSZ,
             "repair_setup",
             1,
             "TNS Phase complete. WNS: {}, TNS: {}",
             delayAsString(final_wns, sta_, digits),
             delayAsString(final_tns, sta_, 1));
}

// Perform VT swap on remaining critical cells as a last resort
bool RepairSetup::swapVTCritCells(const OptoParams& params, int& num_viols)
{
  bool changed = false;

  // Start with sorted violating endpoints
  const VertexSet* endpoints = sta_->endpoints();
  vector<pair<Vertex*, Slack>> violating_ends;
  for (Vertex* end : *endpoints) {
    const Slack end_slack = sta_->vertexSlack(end, max_);
    if (end_slack < params.setup_slack_margin) {
      violating_ends.emplace_back(end, end_slack);
    }
  }
  std::stable_sort(violating_ends.begin(),
                   violating_ends.end(),
                   [](const auto& end_slack1, const auto& end_slack2) {
                     return end_slack1.second < end_slack2.second;
                   });

  // Collect 50 critical instances from worst 100 violating endpoints
  // 50 x 100 = 5000 instances
  const size_t max_endpoints = 100;
  if (violating_ends.size() > max_endpoints) {
    violating_ends.resize(max_endpoints);
  }
  std::unordered_map<Instance*, float> crit_insts;
  std::unordered_set<Vertex*> visited;
  std::unordered_set<Instance*> notSwappable;
  for (const auto& [endpoint, slack] : violating_ends) {
    traverseFaninCone(endpoint, crit_insts, visited, notSwappable, params);
  }
  debugPrint(logger_,
             RSZ,
             "swap_crit_vt",
             1,
             "identified {} critical instances",
             crit_insts.size());

  // Do VT swap on critical instances for now
  // Other transforms can follow later
  VTSwapSpeedMove* move = resizer_->vt_swap_speed_move_.get();
  for (auto crit_inst_slack : crit_insts) {
    if (move->doMove(crit_inst_slack.first, notSwappable)) {
      changed = true;
      debugPrint(logger_,
                 RSZ,
                 "swap_crit_vt",
                 1,
                 "inst {} did crit VT swap",
                 network_->pathName(crit_inst_slack.first));
    }
  }
  if (changed) {
    move->commitMoves();
    estimate_parasitics_->updateParasitics();
    sta_->findRequireds();
    violating_ends.clear();
    for (Vertex* end : *endpoints) {
      const Slack end_slack = sta_->vertexSlack(end, max_);
      if (end_slack < params.setup_slack_margin) {
        violating_ends.emplace_back(end, end_slack);
      }
    }
    num_viols = violating_ends.size();
  }

  return changed;
}

// Traverse fanin code starting from this violaitng endpoint.
// Visit fanin instances only if they have violating slack.
// This avoids exponential path enumeration in findPathEnds.
void RepairSetup::traverseFaninCone(
    Vertex* endpoint,
    std::unordered_map<Instance*, float>& crit_insts,
    std::unordered_set<Vertex*>& visited,
    std::unordered_set<Instance*>& notSwappable,
    const OptoParams& params)

{
  if (visited.find(endpoint) != visited.end()) {
    return;
  }

  visited.insert(endpoint);
  // Limit number of critical instances per endpoint
  const int max_instances = 50;
  std::queue<Vertex*> queue;
  queue.push(endpoint);
  int endpoint_insts = 0;
  LibertyCell* best_lib_cell;

  while (!queue.empty() && endpoint_insts < max_instances) {
    Vertex* current = queue.front();
    queue.pop();

    // Get the instance associated with this vertex
    Instance* inst = nullptr;
    Pin* pin = current->pin();
    if (pin) {
      inst = network_->instance(pin);
    }

    if (inst) {
      // Check if VT swap is possible
      if (resizer_->checkAndMarkVTSwappable(
              inst, notSwappable, best_lib_cell)) {
        // Check if this instance has negative slack
        const Slack inst_slack = getInstanceSlack(inst);
        if (inst_slack < params.setup_slack_margin) {
          // Update worst slack for this instance
          auto it = crit_insts.find(inst);
          if (it == crit_insts.end()) {
            crit_insts[inst] = inst_slack;
            endpoint_insts++;
            debugPrint(logger_,
                       RSZ,
                       "swap_crit_vt",
                       1,
                       "swapVTCritCells: found crit inst {}: slack {}",
                       network_->name(inst),
                       inst_slack);
          }
        }
      }
    }

    // Traverse fanin edges
    VertexInEdgeIterator edge_iter(current, graph_);
    while (edge_iter.hasNext()) {
      Edge* edge = edge_iter.next();
      Vertex* fanin_vertex = edge->from(graph_);
      if (fanin_vertex->isRegClk()) {
        continue;
      }

      // Only traverse if we haven't visited and the fanin has negative slack
      if (visited.find(fanin_vertex) == visited.end()) {
        const Slack fanin_slack = sta_->vertexSlack(fanin_vertex, max_);
        if (fanin_slack < params.setup_slack_margin) {
          queue.push(fanin_vertex);
          visited.insert(fanin_vertex);
        }
      }
    }
  }

  debugPrint(logger_,
             RSZ,
             "swap_crit_vt",
             1,
             "traverseFaninCone: endpoint {} has {} critical instances:",
             endpoint->name(network_),
             endpoint_insts);
  if (logger_->debugCheck(RSZ, "swap_crit_vt", 1)) {
    for (auto crit_inst_slack : crit_insts) {
      logger_->report(" {}", network_->pathName(crit_inst_slack.first));
    }
  }
}

Slack RepairSetup::getInstanceSlack(Instance* inst)
{
  Slack worst_slack = std::numeric_limits<float>::max();

  // Check all output pins of the instance
  InstancePinIterator* pin_iter = network_->pinIterator(inst);
  while (pin_iter->hasNext()) {
    Pin* pin = pin_iter->next();
    if (network_->direction(pin)->isAnyOutput()) {
      Vertex* vertex = graph_->pinDrvrVertex(pin);
      if (vertex) {
        const Slack pin_slack = sta_->vertexSlack(vertex, max_);
        worst_slack = std::min(worst_slack, pin_slack);
      }
    }
  }
  delete pin_iter;

  return worst_slack;
}

}  // namespace rsz
