// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "ViolatorCollector.hh"

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <string>

#include "BaseMove.hh"
#include "BufferMove.hh"
#include "CloneMove.hh"
#include "Rebuffer.hh"
#include "RepairSetup.hh"
#include "rsz/Resizer.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/PathEnd.hh"
#include "sta/PortDirection.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/Sta.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

using std::map;
using std::set;
using std::string;
using utl::RSZ;

using sta::Edge;
using sta::InstancePinIterator;
using sta::NetConnectedPinIterator;
using sta::Pin;
using sta::Slew;
using sta::TimingArcSet;
using sta::Transition;
using sta::VertexInEdgeIterator;

const char* ViolatorCollector::getEnumString(ViolatorSortType sort_type)
{
  switch (sort_type) {
    case ViolatorSortType::SORT_BY_TNS:
      return "SORT_BY_TNS";
    case ViolatorSortType::SORT_BY_WNS:
      return "SORT_BY_WNS";
    case ViolatorSortType::SORT_BY_LOAD_DELAY:
      return "SORT_BY_LOAD_DELAY";
    case ViolatorSortType::SORT_AND_FILTER_BY_LOAD_DELAY:
      return "SORT_AND_FILTER_BY_LOAD_DELAY";
    case ViolatorSortType::SORT_BY_HEURISTIC:
      return "SORT_BY_HEURISTIC";
    default:
      return "UNKNOWN";
  }
}

void ViolatorCollector::printViolators(int numPrint = 0) const
{
  if (violating_pins_.empty()) {
    logger_->info(RSZ, 8, "No violating pins found.");
    return;
  }

  debugPrint(logger_,
             RSZ,
             "violator_collector",
             23,
             "============= Found {} violating pins. =================",
             violating_pins_.size());
  int count = 0;
  for (const Pin* pin : violating_pins_) {
    if (count++ >= numPrint && numPrint > 0) {
      break;
    }
    LibertyPort* port = network_->libertyPort(pin);
    LibertyCell* cell = port->libertyCell();
    float slack = pin_data_.at(pin).slack;
    float tns = pin_data_.at(pin).tns;
    float load_delay = pin_data_.at(pin).load_delay;
    float intrinsic_delay = pin_data_.at(pin).intrinsic_delay;
    float total_delay = load_delay + intrinsic_delay;
    Vertex* vertex = graph_->pinDrvrVertex(pin);
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               2,
               "{} ({}) slack={} tns={} level={} delay={} "
               "(load_delay={} + "
               "intrinsic_delay={})",
               vertex->name(network_),
               cell->name(),
               delayAsString(slack, sta_, 3),
               delayAsString(tns, sta_, 3),
               pin_data_.at(pin).level,
               delayAsString(total_delay, sta_, 3),
               delayAsString(load_delay, sta_, 3),
               delayAsString(intrinsic_delay, sta_, 3));
  }
}

int ViolatorCollector::getTotalViolations() const
{
  return violating_ends_.size();
}

void ViolatorCollector::printHistogram(int numBins) const
{
  if (violating_pins_.empty()) {
    debugPrint(
        logger_, RSZ, "violator_collector", 1, "No violating pins found.");
    return;
  }
  std::map<int, int> slack_histogram;

  // Dynamically determine the bucket size
  float min_slack = std::numeric_limits<float>::max();
  float max_slack = std::numeric_limits<float>::lowest();
  for (const Pin* pin : violating_pins_) {
    float slack = sta_->pinSlack(pin, max_);
    min_slack = std::min(min_slack, slack);
    max_slack = std::max(max_slack, slack);
  }
  float bucket_size = (max_slack - min_slack) / float(numBins - 1);
  debugPrint(logger_,
             RSZ,
             "violator_collector",
             1,
             "Slack histogram: pins={} min={} max={} bucket_size={}",
             violating_pins_.size(),
             delayAsString(min_slack, sta_, 3),
             delayAsString(max_slack, sta_, 3),
             delayAsString(bucket_size, sta_, 3));

  // Initialize the bins
  int min_bin_index = static_cast<int>(std::floor(min_slack / bucket_size));
  int max_bin_index = static_cast<int>(std::floor(max_slack / bucket_size));
  for (int i = min_bin_index; i <= max_bin_index; ++i) {
    slack_histogram[i] = 0;
  }

  // Track the slack counts in each bin
  for (const Pin* pin : violating_pins_) {
    float slack = sta_->pinSlack(pin, max_);
    int slack_bucket = static_cast<int>(std::floor(slack / bucket_size));
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               4,
               "Pin: {} Slack: {} Bucket: {}",
               network_->pathName(pin),
               delayAsString(slack, sta_, 3),
               slack_bucket);
    slack_histogram[slack_bucket]++;
  }

  for (int i = min_bin_index; i <= max_bin_index; ++i) {
    float bin_slack = (i + 1) * bucket_size;
    int count = slack_histogram[i];
    std::string bar(count, '*');
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               1,
               "Slack: < {:7}: ({:4}) {}",
               delayAsString(bin_slack, sta_, 3),
               count,
               bar);
  }
}

// Must be called after STA is initialized
void ViolatorCollector::init(float slack_margin)
{
  graph_ = sta_->graph();
  search_ = sta_->search();
  sdc_ = sta_->sdc();
  report_ = sta_->report();
  // IMPROVE ME: always looks at cmd corner
  corner_ = sta_->cmdCorner();

  slack_margin_ = slack_margin;

  collectViolatingEndpoints();

  pin_data_.clear();
  violating_pins_.clear();

  // Initialize endpoint iteration and reset pass tracking
  current_endpoint_index_ = 0;
  current_pass_count_ = 0;
  endpoint_times_considered_.clear();
  if (!violating_ends_.empty()) {
    setToEndpoint(0);
  }
}

void ViolatorCollector::collectBySlack()
{
  violating_pins_.clear();

  InstanceSeq insts = network_->leafInstances();
  for (auto inst : insts) {
    auto pin_iter
        = std::unique_ptr<InstancePinIterator>(network_->pinIterator(inst));
    while (pin_iter->hasNext()) {
      const Pin* pin = pin_iter->next();
      if (!network_->direction(pin)->isOutput() || network_->isTopLevelPort(pin)
          || sta_->isClock(pin)) {
        continue;
      }
      Vertex* vertex = graph_->pinDrvrVertex(pin);
      float slack = sta_->pinSlack(pin, max_);
      if (slack < slack_margin_) {
        LibertyPort* port = network_->libertyPort(pin);
        LibertyCell* cell = port->libertyCell();
        debugPrint(logger_,
                   RSZ,
                   "violator_collector",
                   4,
                   "Found violating instance: {} ({}) slack={} level={}",
                   network_->pathName(pin),
                   cell->name(),
                   delayAsString(slack, sta_, 3),
                   vertex->level());
        violating_pins_.push_back(pin);
      }
    }
  }

  debugPrint(logger_,
             RSZ,
             "violator_collector",
             2,
             "Found {} violating pins.",
             violating_pins_.size());
}

std::pair<Delay, Delay> ViolatorCollector::getEffortDelays(const Pin* pin)
{
  Vertex* path_vertex = graph_->pinDrvrVertex(pin);
  Delay selected_load_delay = -sta::INF;
  Delay selected_intrinsic_delay = -sta::INF;
  Slack worst_slack = sta::INF;

  // Find the arc (specific edge + rise/fall) with worst slack
  VertexInEdgeIterator edge_iter(path_vertex, graph_);
  while (edge_iter.hasNext()) {
    Edge* prev_edge = edge_iter.next();
    Vertex* from_vertex = prev_edge->from(graph_);

    // Ignore output-to-output timing arcs (e.g., in asap7 multi-output gates)
    const Pin* from_pin = from_vertex->pin();
    if (!from_pin) {
      continue;
    }
    if (network_->direction(from_pin)->isOutput()) {
      continue;
    }

    const TimingArcSet* arc_set = prev_edge->timingArcSet();
    for (const RiseFall* rf : RiseFall::range()) {
      TimingArc* prev_arc = arc_set->arcTo(rf);
      // This can happen for flops with only one transition type arc.
      if (!prev_arc) {
        continue;
      }

      // Get the input transition for this arc
      const sta::Transition* from_trans = prev_arc->fromEdge();
      const RiseFall* from_rf = from_trans->asRiseFall();

      // Get slack for the input transition
      Slack from_slack = sta_->vertexSlack(from_vertex, from_rf, max_);

      const TimingArc* corner_arc = prev_arc->cornerArc(lib_ap_);
      const Delay intrinsic_delay = corner_arc->intrinsicDelay();
      const Delay delay
          = graph_->arcDelay(prev_edge, prev_arc, dcalc_ap_->index());
      const Delay load_delay = delay - intrinsic_delay;

      debugPrint(logger_,
                 RSZ,
                 "violator_collector",
                 5,
                 "Arc: {}({})->{} ({}) Load Delay: {} Intrinsic "
                 "Delay: {} Slack: {}",
                 network_->pathName(from_vertex->pin()),
                 from_rf->name(),
                 network_->pathName(pin),
                 rf->name(),
                 delayAsString(load_delay, sta_, 3),
                 delayAsString(intrinsic_delay, sta_, 3),
                 delayAsString(from_slack, sta_, 3));

      // Select load_delay from arc with most negative slack
      // If slacks are equal, prefer arc with higher load_delay
      if (from_slack < worst_slack
          || (from_slack == worst_slack && load_delay > selected_load_delay)) {
        worst_slack = from_slack;
        selected_load_delay = load_delay;
        selected_intrinsic_delay = intrinsic_delay;
      }
    }
  }

  return {selected_load_delay, selected_intrinsic_delay};
}

void ViolatorCollector::updatePinData(const Pin* pin, pinData& pd)
{
  pd.name = network_->pathName(pin);

  auto [load_delay, intrinsic_delay] = getEffortDelays(pin);
  pd.load_delay = load_delay;
  pd.intrinsic_delay = intrinsic_delay;

  Vertex* vertex = graph_->pinDrvrVertex(pin);
  pd.level = vertex->level();
  pd.slack = sta_->pinSlack(pin, max_);
  pd.tns = getLocalPinTNS(pin);
}

int ViolatorCollector::repairsPerPass(int max_repairs_per_pass)
{
  Slack min_viol_ = -sta::INF;
  Slack max_viol_ = 0;
  if (!violating_ends_.empty()) {
    min_viol_ = -violating_ends_.back().second;
    max_viol_ = -violating_ends_.front().second;
  }

  Slack path_slack = getCurrentEndpointSlack();
  int repairs_per_pass = 1;
  if (max_viol_ - min_viol_ != 0.0) {
    repairs_per_pass
        += std::round((max_repairs_per_pass - 1) * (-path_slack - min_viol_)
                      / (max_viol_ - min_viol_));
  }

  return repairs_per_pass;
}

void ViolatorCollector::sortByLoadDelay(float load_delay_threshold)
{
  std::sort(violating_pins_.begin(),
            violating_pins_.end(),
            [this](const Pin* a, const Pin* b) {
              Delay load_delay1 = pin_data_[a].load_delay;
              Delay load_delay2 = pin_data_[b].load_delay;
              float tns1 = pin_data_[a].tns;
              float tns2 = pin_data_[b].tns;

              return load_delay1 > load_delay2
                     || (load_delay1 == load_delay2 && tns1 < tns2)
                     || (load_delay1 == load_delay2 && tns1 == tns2
                         && network_->pathNameLess(a, b));
            });

  // Filter: only keep pins where load_delay > load_delay_threshold * intrinsic_delay
  // If threshold is 0.0 or negative, skip filtering
  if (load_delay_threshold > 0.0) {
    auto it = std::remove_if(
        violating_pins_.begin(), violating_pins_.end(), [this, load_delay_threshold](const Pin* pin) {
          Delay load_delay = pin_data_[pin].load_delay;
          Delay intrinsic_delay = pin_data_[pin].intrinsic_delay;
          return load_delay <= load_delay_threshold * intrinsic_delay;
        });
    violating_pins_.erase(it, violating_pins_.end());
  }

  for (auto pin : violating_pins_) {
    Delay worst_load_delay = pin_data_[pin].load_delay;
    Delay worst_intrinsic_delay = pin_data_[pin].intrinsic_delay;
    Vertex* vertex = graph_->pinDrvrVertex(pin);

    debugPrint(logger_,
               RSZ,
               "violator_collector",
               4,
               "{} load_delay = {} intrinsic_delay = {}",
               vertex->name(network_),
               delayAsString(worst_load_delay, sta_, 3),
               delayAsString(worst_intrinsic_delay, sta_, 3));
  }
}

// Helper to get the slack of a specific path (by index) for an endpoint
Slack ViolatorCollector::getPathSlackByIndex(const Pin* endpoint_pin,
                                             int path_index)
{
  // Create ExceptionTo for this endpoint
  sta::PinSet* to_pins = new sta::PinSet(network_);
  to_pins->insert(endpoint_pin);
  sta::ExceptionTo* to = sdc_->makeExceptionTo(to_pins,
                                               nullptr,
                                               nullptr,
                                               sta::RiseFallBoth::riseFall(),
                                               sta::RiseFallBoth::riseFall());

  // Find paths to the endpoint - request only up to path_index+1 paths
  int num_paths_needed = path_index + 1;
  sta::PathEndSeq path_ends
      = search_->findPathEnds(nullptr,                // from
                              nullptr,                // thrus
                              to,                     // to
                              false,                  // unconstrained
                              corner_,                // corner
                              sta::MinMaxAll::all(),  // min_max
                              num_paths_needed,       // group_path_count
                              num_paths_needed,       // endpoint_path_count
                              false,                  // unique_pins
                              -sta::INF,              // slack_min
                              sta::INF,               // slack_max
                              true,                   // sort_by_slack
                              nullptr,                // group_names
                              true,
                              false,
                              true,
                              true,
                              true,
                              true);  // checks

  // Return the slack of the requested path index
  if (path_index < static_cast<int>(path_ends.size())) {
    return path_ends[path_index]->slack(search_);
  }

  // If we don't have enough paths, return INF (no path available)
  return sta::INF;
}

vector<const Pin*> ViolatorCollector::collectViolators(
    int numPathsPerEndpoint,
    int numPins,
    ViolatorSortType sort_type)
{
  dcalc_ap_ = corner_->findDcalcAnalysisPt(sta::MinMax::max());
  lib_ap_ = dcalc_ap_->libertyIndex();

  violating_pins_.clear();
  set<const Pin*> collected_pins;

  // If current_endpoint_ is set, collect only from that endpoint
  if (current_endpoint_) {
    const Pin* endpoint_pin = current_endpoint_->pin();
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               2,
               "Collecting from current endpoint {} (numPaths={})",
               network_->pathName(endpoint_pin),
               numPathsPerEndpoint);

    // Collect pins from the specified number of paths for this endpoint
    set<const Pin*> new_pins
        = collectPinsByPathEndpoint(endpoint_pin, numPathsPerEndpoint);
    collected_pins.insert(new_pins.begin(), new_pins.end());

    debugPrint(logger_,
               RSZ,
               "violator_collector",
               3,
               "Collected {} unique pins from current endpoint's {} path(s)",
               collected_pins.size(),
               numPathsPerEndpoint);

    // Increment the times this endpoint has been considered
    endpoint_times_considered_[endpoint_pin]++;
  } else {
    // Dynamic endpoint discovery mode - find worst paths across all endpoints
    // Get fresh list of violating endpoints each iteration
    collectViolatingEndpoints();

    debugPrint(logger_,
               RSZ,
               "violator_collector",
               2,
               "Found {} violating endpoints this iteration",
               violating_ends_.size());

    // Track which endpoints we've already processed and how many paths
    map<const Pin*, int> endpoint_paths_used;

    // Collect pins by comparing actual path slacks across all endpoints
    // If numPins is -1, collect all pins without limiting
    while (
        (numPins == -1 || collected_pins.size() < static_cast<size_t>(numPins))
        && !violating_ends_.empty()) {
      // Find the worst actual path slack among all available paths
      const Pin* best_endpoint = nullptr;
      Slack best_slack = sta::INF;
      int best_path_num = 0;

      // Check each violating endpoint
      for (const auto& [endpoint_pin, endpoint_slack] : violating_ends_) {
        // Skip endpoints that have been considered too many times
        int times_considered = endpoint_times_considered_[endpoint_pin];
        if (times_considered >= max_passes_per_endpoint_) {
          debugPrint(logger_,
                     RSZ,
                     "violator_collector",
                     4,
                     "Skipping endpoint {} (considered {} times >= {})",
                     network_->pathName(endpoint_pin),
                     times_considered,
                     max_passes_per_endpoint_);
          continue;
        }

        int paths_used = endpoint_paths_used[endpoint_pin];

        // If we haven't used all paths for this endpoint yet
        if (paths_used < numPathsPerEndpoint) {
          // Get the actual slack of the next available path for this endpoint
          Slack path_slack = getPathSlackByIndex(endpoint_pin, paths_used);

          // Skip if no path is available (shouldn't happen, but be defensive)
          if (path_slack >= slack_margin_) {
            continue;
          }

          // Compare actual path slack, not endpoint slack
          if (path_slack < best_slack) {
            best_slack = path_slack;
            best_endpoint = endpoint_pin;
            best_path_num = paths_used;
          }
        }
      }

      // No more endpoints/paths to process
      if (best_endpoint == nullptr) {
        break;
      }

      // Collect pins from the selected endpoint/path
      set<const Pin*> new_pins = collectPinsByPathEndpoint(best_endpoint, 1);

      size_t before_size = collected_pins.size();
      collected_pins.insert(new_pins.begin(), new_pins.end());

      debugPrint(
          logger_,
          RSZ,
          "violator_collector",
          3,
          "Collected {} pins ({} new) from endpoint {} path {} (slack={})",
          new_pins.size(),
          collected_pins.size() - before_size,
          network_->pathName(best_endpoint),
          best_path_num + 1,
          delayAsString(best_slack, sta_, 3));

      // Mark that we've used another path from this endpoint
      endpoint_paths_used[best_endpoint]++;

      // Increment the times this endpoint has been considered
      endpoint_times_considered_[best_endpoint]++;

      // Stop if we've reached the desired number of pins (unless numPins is -1)
      if (numPins != -1
          && collected_pins.size() >= static_cast<size_t>(numPins)) {
        break;
      }
    }
  }

  // Convert set to vector
  violating_pins_.clear();
  violating_pins_.insert(
      violating_pins_.end(), collected_pins.begin(), collected_pins.end());

  debugPrint(logger_,
             RSZ,
             "violator_collector",
             2,
             "Collected {} unique violating pins",
             violating_pins_.size());

  sortPins(numPins, sort_type);

  // Auto-increment pass count for current endpoint when collecting violators
  current_pass_count_++;
  debugPrint(logger_,
             RSZ,
             "violator_collector",
             4,
             "Endpoint {} pass count: {}/{}",
             network_->pathName(current_endpoint_->pin()),
             current_pass_count_,
             max_passes_per_endpoint_);

  return violating_pins_;
}

void ViolatorCollector::sortPins(int numPins, ViolatorSortType sort_type)
{
  for (auto pin : violating_pins_) {
    updatePinData(pin, pin_data_[pin]);
  }
  debugPrint(logger_,
             RSZ,
             "violator_collector",
             1,
             "Sorting {} violating pins by {}.",
             violating_pins_.size(),
             getEnumString(sort_type));
  switch (sort_type) {
    case ViolatorSortType::SORT_BY_TNS:
      sortByLocalTNS();
      break;
    case ViolatorSortType::SORT_BY_WNS:
      sortByWNS();
      break;
    case ViolatorSortType::SORT_BY_LOAD_DELAY:
      sortByLoadDelay(0.0);  // No filtering
      break;
    case ViolatorSortType::SORT_AND_FILTER_BY_LOAD_DELAY:
      sortByLoadDelay(0.75);  // Filter: keep only pins where load_delay > 0.75 * intrinsic_delay
      break;
    case ViolatorSortType::SORT_BY_HEURISTIC:
      sortByHeuristic();
      break;
    default:
      logger_->error(
          RSZ, 9, "Unknown sort type: {}.", getEnumString(sort_type));
  }

  // Truncate to keep only the top numPins pins (unless numPins is -1)
  if (numPins > 0 && numPins < static_cast<int>(violating_pins_.size())) {
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               1,
               "Keeping only {} of {} violating pins.",
               numPins,
               violating_pins_.size());
    violating_pins_.resize(numPins);
  } else if (numPins == -1) {
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               1,
               "Keeping all {} violating pins (no limit).",
               violating_pins_.size());
  }

  printViolators();
}

void ViolatorCollector::sortByWNS()
{
  std::sort(violating_pins_.begin(),
            violating_pins_.end(),
            [this](const Pin* a, const Pin* b) {
              float slack1 = pin_data_[a].slack;
              float slack2 = pin_data_[b].slack;
              int level1 = pin_data_[a].level;
              int level2 = pin_data_[b].level;

              return slack1 < slack2 || (slack1 == slack2 && level1 > level2)
                     || (slack1 == slack2 && level1 == level2
                         && network_->pathNameLess(a, b));
            });
}

Delay ViolatorCollector::getLocalPinTNS(const Pin* pin) const
{
  Delay tns = 0;
  Vertex* drvr_vertex = graph_->pinDrvrVertex(pin);
  sta::VertexOutEdgeIterator edge_iter(drvr_vertex, graph_);
  while (edge_iter.hasNext()) {
    Edge* edge = edge_iter.next();
    Vertex* fanout_vertex = edge->to(graph_);
    // Watch out for problematic asap7 output->output timing arcs.
    const Pin* fanout_pin = fanout_vertex->pin();
    if (!fanout_pin) {
      continue;
    }
    if (network_->direction(fanout_pin)->isOutput()) {
      continue;
    }
    const Slack fanout_slack = sta_->vertexSlack(fanout_vertex, resizer_->max_);
    if (fanout_slack < 0) {
      tns += fanout_slack;
    }
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               4,
               " pin {} fanout {} slack: {} tns: {}",
               network_->pathName(pin),
               network_->pathName(fanout_vertex->pin()),
               delayAsString(fanout_slack, sta_, 3),
               delayAsString(tns, sta_, 3));
  }
  return tns;
}

map<const Pin*, Delay> ViolatorCollector::getLocalTNS() const
{
  map<const Pin*, Delay> local_tns;
  for (auto pin : violating_pins_) {
    local_tns[pin] = getLocalPinTNS(pin);
  }
  return local_tns;
}

void ViolatorCollector::sortByLocalTNS()
{
  std::sort(violating_pins_.begin(),
            violating_pins_.end(),
            [this](const Pin* a, const Pin* b) {
              float tns1 = pin_data_[a].tns;
              float tns2 = pin_data_[b].tns;
              Delay load_delay1 = pin_data_[a].load_delay;
              Delay load_delay2 = pin_data_[b].load_delay;
              return tns1 < tns2 || (tns1 == tns2 && load_delay1 > load_delay2)
                     || (tns1 == tns2 && load_delay1 == load_delay2
                         && network_->pathNameLess(a, b));
            });
}

void ViolatorCollector::sortByHeuristic()
{
  std::sort(violating_pins_.begin(),
            violating_pins_.end(),
            [this](const Pin* a, const Pin* b) {
              // Linear combination: -TNS (to make it positive) + load_delay
              float tns1 = -pin_data_[a].tns;
              float tns2 = -pin_data_[b].tns;
              Delay load_delay1 = pin_data_[a].load_delay;
              Delay load_delay2 = pin_data_[b].load_delay;

              float score1 = tns1 + load_delay1;
              float score2 = tns2 + load_delay2;

              return score1 > score2
                     || (score1 == score2 && network_->pathNameLess(a, b));
            });
}

void ViolatorCollector::collectViolatingEndpoints()
{
  violating_ends_.clear();

  const VertexSet* endpoints = sta_->endpoints();
  for (Vertex* end : *endpoints) {
    const Slack end_slack = sta_->vertexSlack(end, max_);
    if (end_slack < slack_margin_) {
      violating_ends_.emplace_back(end->pin(), end_slack);
    }
  }
  std::stable_sort(violating_ends_.begin(),
                   violating_ends_.end(),
                   [](const auto& end_slack1, const auto& end_slack2) {
                     return end_slack1.second < end_slack2.second;
                   });

  debugPrint(logger_,
             RSZ,
             "violator_collector",
             1,
             "Violating endpoints {}/{} {}%",
             violating_ends_.size(),
             endpoints->size(),
             int(violating_ends_.size() / double(endpoints->size()) * 100));
}

void ViolatorCollector::collectByPaths(int endPointIndex,
                                       int numEndpoints,
                                       int numPathsPerEndpoint)
{
  // For skipping duplicates
  set<const Pin*> viol_pins;

  size_t old_size = 0;
  int endpoint_count = 0;
  for (int i = endPointIndex; i < violating_ends_.size(); i++) {
    const auto end_original_slack = violating_ends_[i];
    // Only count the critical endpoints
    if (end_original_slack.second < slack_margin_) {
      const Pin* endpoint_pin = end_original_slack.first;
      set<const Pin*> end_pins
          = collectPinsByPathEndpoint(endpoint_pin, numPathsPerEndpoint);
      viol_pins.insert(end_pins.begin(), end_pins.end());

      debugPrint(
          logger_,
          RSZ,
          "violator_collector",
          2,
          "Collected {} pins ({} unique) for endpoint {} total collected {}",
          end_pins.size(),
          viol_pins.size() - old_size,
          network_->pathName(endpoint_pin),
          viol_pins.size());
      old_size = viol_pins.size();
      endpoint_count++;
    }
    if (numEndpoints > 0 && endpoint_count >= numEndpoints) {
      break;
    }
  }

  violating_pins_.clear();
  violating_pins_.insert(
      violating_pins_.end(), viol_pins.begin(), viol_pins.end());
}

set<const Pin*> ViolatorCollector::collectPinsByPathEndpoint(
    const sta::Pin* endpoint_pin,
    size_t paths_per_endpoint)

{
  // Create a set to remove duplciates
  set<const Pin*> viol_pins;

  // This uses the old method for a single path at a time
  /*if (paths_per_endpoint == 1) {
    Vertex* end = graph_->pinDrvrVertex(endpoint_pin);
    Path* end_path = sta_->vertexWorstSlackPath(end, max_);
    PathExpanded expanded(end_path, sta_);
    if (expanded.size() > 1) {
      const int path_length = expanded.size();
      const int start_index = expanded.startIndex();
      for (int i = start_index; i < path_length; i++) {
        const Path* path = expanded.path(i);
        const Pin* path_pin = path->pin(sta_);
        Vertex* path_vertex = path->vertex(sta_);
        if (!path_vertex->isDriver(network_)
            || network_->isTopLevelPort(path_pin)) {
          continue;
        }
        viol_pins.insert(path_pin);
        debugPrint(logger_,
                   RSZ,
                   "violator_collector",
                   3,
                   "  - Pin: {}",
                   network_->pathName(path_pin));
      }
      return viol_pins;
    }
  } else {
    // FIXME later
    return viol_pins;
  }
    */

  // This code does not behave properly for single path case. At some points it
  // does not match the above vertexWorstSlackPath method. It seems to be
  // related to the use of the ExceptionTo object.
  // 1. Define the single endpoint for the path search.
  sta::PinSet* to_pins = new sta::PinSet(network_);
  to_pins->insert(endpoint_pin);
  // The ExceptionTo object will be owned and deleted by the SDC.
  sta::ExceptionTo* to = sdc_->makeExceptionTo(to_pins,
                                               nullptr,
                                               nullptr,
                                               sta::RiseFallBoth::riseFall(),
                                               sta::RiseFallBoth::riseFall());

  // 2. Find paths to the endpoint.
  sta::PathEndSeq path_ends
      = search_->findPathEnds(nullptr,                // from
                              nullptr,                // thrus
                              to,                     // to
                              false,                  // unconstrained
                              corner_,                // corner
                              sta::MinMaxAll::all(),  // min_max
                              paths_per_endpoint,     // group_path_count
                              paths_per_endpoint,     // endpoint_path_count
                              false,                  // unique_pins
                              -sta::INF,              // slack_min
                              sta::INF,               // slack_max
                              true,                   // sort_by_slack
                              nullptr,                // group_names
                              true,
                              false,
                              true,
                              true,
                              true,
                              true);  // checks

  int path_num = 1;
  for (const sta::PathEnd* path_end : path_ends) {
    // 3. Use PathExpanded to access the individual nodes in the path.
    sta::PathExpanded expanded(path_end->path(), search_);
    float path_slack = path_end->slack(search_);

    if (path_slack > 0.0) {
      break;
    }

    debugPrint(logger_,
               RSZ,
               "violator_collector",
               4,
               "Critical path {} (Slack: {}ps) Pins: {}",
               path_num++,
               delayAsString(path_slack, sta_, 3),
               expanded.size());

    // 4. Iterate over the nodes (path segments) in the expanded path.
    for (size_t i = 0; i < expanded.size(); i++) {
      const sta::Path* path = expanded.path(i);
      const sta::Pin* pin = path->pin(graph_);
      if (!network_->direction(pin)->isOutput() || network_->isTopLevelPort(pin)
          || sta_->isClock(pin)) {
        continue;
      }

      const sta::Instance* inst = network_->instance(pin);

      if (network_->isTopLevelPort(pin)) {
        debugPrint(logger_,
                   RSZ,
                   "violator_collector",
                   5,
                   "  - Port: {}",
                   network_->pathName(pin));
      } else if (inst) {
        const sta::LibertyCell* lib_cell = network_->libertyCell(inst);
        if (lib_cell) {
          float pin_slack = path->slack(search_);
          debugPrint(logger_,
                     RSZ,
                     "violator_collector",
                     5,
                     "  - Gate: {} (Type: {}) -> Pin: {} Slack: {}",
                     network_->pathName(inst),
                     lib_cell->name(),
                     network_->portName(pin),
                     delayAsString(pin_slack, sta_, 3));
          viol_pins.insert(pin);
        }
      }
    }
    debugPrint(logger_, RSZ, "violator_collector", 5, "\n");
  }

  return viol_pins;
}

vector<const Pin*> ViolatorCollector::collectViolatorsFromEndpoints(
    const vector<Vertex*>& endpoints,
    int numPathsPerEndpoint,
    int numPins,
    ViolatorSortType sort_type)
{
  dcalc_ap_ = corner_->findDcalcAnalysisPt(sta::MinMax::max());
  lib_ap_ = dcalc_ap_->libertyIndex();

  violating_pins_.clear();

  debugPrint(logger_,
             RSZ,
             "violator_collector",
             2,
             "Collecting violators from {} specific endpoints: paths={}, "
             "pins={}, sort={}",
             endpoints.size(),
             numPathsPerEndpoint,
             numPins,
             getEnumString(sort_type));

  // Collect violating pins from the specified endpoints
  for (Vertex* endpoint : endpoints) {
    Pin* endpoint_pin = endpoint->pin();

    debugPrint(logger_,
               RSZ,
               "violator_collector",
               3,
               "Processing endpoint: {} (slack={})",
               endpoint->name(network_),
               delayAsString(sta_->vertexSlack(endpoint, max_), sta_, 3));

    // Collect pins from paths through this endpoint
    set<const Pin*> endpoint_pins
        = collectPinsByPathEndpoint(endpoint_pin, numPathsPerEndpoint);

    for (const Pin* pin : endpoint_pins) {
      violating_pins_.push_back(pin);
    }
  }

  debugPrint(logger_,
             RSZ,
             "violator_collector",
             2,
             "Collected {} violating pins from {} endpoints",
             violating_pins_.size(),
             endpoints.size());

  // Sort and limit the pins
  sortPins(numPins, sort_type);

  return violating_pins_;
}

void ViolatorCollector::setMaxPassesPerEndpoint(int max_passes)
{
  max_passes_per_endpoint_ = max_passes;
}

bool ViolatorCollector::shouldSkipEndpoint() const
{
  return current_pass_count_ >= max_passes_per_endpoint_;
}

int ViolatorCollector::getEndpointPassCount() const
{
  return current_pass_count_;
}

void ViolatorCollector::resetEndpointPasses()
{
  current_pass_count_ = 0;
  endpoint_times_considered_.clear();
  debugPrint(
      logger_, RSZ, "violator_collector", 2, "Reset endpoint pass tracking");
}

bool ViolatorCollector::hasMoreEndpoints() const
{
  return current_endpoint_index_ + 1 < static_cast<int>(violating_ends_.size());
}

void ViolatorCollector::setToEndpoint(int index)
{
  current_endpoint_index_ = index;
  const auto& end_slack_pair = violating_ends_[current_endpoint_index_];
  current_endpoint_ = graph_->pinLoadVertex(end_slack_pair.first);
  current_end_original_slack_ = end_slack_pair.second;
}
void ViolatorCollector::advanceToNextEndpoint()
{
  if (hasMoreEndpoints()) {
    current_endpoint_index_++;
    current_pass_count_ = 0;  // Reset pass count for new endpoint
    setToEndpoint(current_endpoint_index_);
    debugPrint(
        logger_,
        RSZ,
        "violator_collector",
        2,
        "Advancing to next endpoint {}/{} ({})",
        current_endpoint_index_,
        violating_ends_.size(),
        network_->pathName(violating_ends_[current_endpoint_index_].first));
  }
}

Slack ViolatorCollector::getCurrentEndpointSlack() const
{
  if (current_endpoint_) {
    return sta_->vertexSlack(current_endpoint_, max_);
  }
  return 0.0;
}

void ViolatorCollector::useWorstEndpoint(Vertex* end)
{
  current_endpoint_ = end;
}

int ViolatorCollector::getCurrentPass() const
{
  return current_pass_count_;
}

}  // namespace rsz
