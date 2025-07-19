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
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/PathEnd.hh"
#include "sta/PortDirection.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/Sta.hh"
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
             1,
             "============= Found {} violating pins. =================",
             violating_pins_.size());
  int count = 0;
  for (const Pin* pin : violating_pins_) {
    if (count++ >= numPrint && numPrint > 0) {
      break;
    }
    LibertyPort* port = network_->libertyPort(pin);
    LibertyCell* cell = port->libertyCell();
    Vertex* vertex = graph_->pinDrvrVertex(pin);
    float slack = sta_->pinSlack(pin, max_);
    pair<Delay, Delay> delays = getDelays(pin);
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               1,
               "{} ({}) slack={} level={} delay={} load_delay={} + "
               "intrinsic_delay={}",
               network_->pathName(pin),
               cell->name(),
               delayAsString(slack, sta_, 3),
               vertex->level(),
               delayAsString(delays.first + delays.second, sta_, 3),
               delayAsString(delays.first, sta_, 3),
               delayAsString(delays.second, sta_, 3));
  }
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

pair<Delay, Delay> ViolatorCollector::getDelays(const Pin* pin) const
{
  Vertex* path_vertex = graph_->pinDrvrVertex(pin);
  Delay worst_load_delay = -sta::INF;
  Delay worst_intrinsic_delay = -sta::INF;

  VertexInEdgeIterator edge_iter(path_vertex, graph_);
  while (edge_iter.hasNext()) {
    Edge* prev_edge = edge_iter.next();
    const TimingArcSet* arc_set = prev_edge->timingArcSet();
    for (const RiseFall* rf : RiseFall::range()) {
      TimingArc* prev_arc = arc_set->arcTo(rf);
      const TimingArc* corner_arc = prev_arc->cornerArc(lib_ap_);
      const Delay intrinsic_delay = corner_arc->intrinsicDelay();
      const Delay delay
          = graph_->arcDelay(prev_edge, prev_arc, dcalc_ap_->index());
      const Delay load_delay = delay - intrinsic_delay;
      if (load_delay > worst_load_delay) {
        worst_load_delay = load_delay;
        worst_intrinsic_delay = intrinsic_delay;
      }
    }
  }

  return std::make_pair(worst_load_delay, worst_intrinsic_delay);
}

void ViolatorCollector::sortByLoadDelay()
{
  map<const Pin*, Delay> load_delays;
  for (auto pin : violating_pins_) {
    std::pair<Delay, Delay> delays = getDelays(pin);
    Delay worst_load_delay = delays.first;
    Delay worst_intrinsic_delay = delays.second;
    Delay worst_delay = worst_load_delay + worst_intrinsic_delay;

    load_delays[pin] = worst_load_delay;
    debugPrint(logger_,
               RSZ,
               "violator_collector",
               3,
               "{} load_delay = {} intrinsic_delay = {}",
               network_->portName(pin),
               delayAsString(worst_delay, sta_, 3),
               delayAsString(worst_load_delay, sta_, 3),
               delayAsString(worst_intrinsic_delay, sta_, 3));
  }

  std::sort(violating_pins_.begin(),
            violating_pins_.end(),
            [this, &load_delays](const Pin* a, const Pin* b) {
              Vertex* vertex1 = graph_->pinDrvrVertex(a);
              Vertex* vertex2 = graph_->pinDrvrVertex(b);
              int level1 = vertex1->level();
              int level2 = vertex2->level();
              return load_delays[a] > load_delays[b]
                     || (load_delays[a] == load_delays[b] && level1 > level2)
                     || (load_delays[a] == load_delays[b] && level1 == level2
                         && network_->pathNameLess(a, b));
            });
}

vector<const Pin*> ViolatorCollector::collectViolatorsByEndpoint(
    int endpoint_index,
    ViolatorSortType sort_type)
{
  dcalc_ap_ = corner_->findDcalcAnalysisPt(sta::MinMax::max());
  lib_ap_ = dcalc_ap_->libertyIndex();

  violating_pins_.clear();
  if (endpoint_index > violating_ends_.size()) {
    logger_->error(RSZ,
                   14,
                   "Endpoint index {} out of range ({} endpoints).",
                   endpoint_index,
                   violating_ends_.size());
    return violating_pins_;
  }
  set<const Pin*> pin_set
      = collectPinsByPathEndpoint(violating_ends_[endpoint_index].first, 1);
  violating_pins_.insert(violating_pins_.end(), pin_set.begin(), pin_set.end());

  sortPins(0, sort_type);

  return (violating_pins_);
}

vector<const Pin*> ViolatorCollector::collectViolators(
    int numPaths,
    int numEndpoints,
    int numPins,
    ViolatorSortType sort_type)
{
  dcalc_ap_ = corner_->findDcalcAnalysisPt(sta::MinMax::max());
  lib_ap_ = dcalc_ap_->libertyIndex();

  if (numPaths > 0) {
    collectByPaths(numPaths, numEndpoints);
  } else {
    collectBySlack();
  }

  sortPins(numPins, sort_type);

  return violating_pins_;
}

void ViolatorCollector::sortPins(int numPins, ViolatorSortType sort_type)
{
  debugPrint(logger_,
             RSZ,
             "violator_collector",
             1,
             "Sorting {} violating pins by {}.",
             violating_pins_.size(),
             getEnumString(sort_type));
  switch (sort_type) {
    case ViolatorSortType::SORT_BY_TNS:
      sortByTNS();
      break;
    case ViolatorSortType::SORT_BY_WNS:
      sortByWNS();
      break;
    case ViolatorSortType::SORT_BY_LOAD_DELAY:
      sortByLoadDelay();
      break;
    default:
      logger_->error(
          RSZ, 9, "Unknown sort type: {}.", getEnumString(sort_type));
  }

  // Truncate to keep only the top numPins pins
  if (numPins > 0 && numPins < violating_pins_.size()) {
    violating_pins_.resize(numPins);
  }

  printViolators(10);
}

void ViolatorCollector::sortByWNS()
{
  std::sort(violating_pins_.begin(),
            violating_pins_.end(),
            [this](const Pin* a, const Pin* b) {
              float slack1 = sta_->pinSlack(a, max_);
              float slack2 = sta_->pinSlack(b, max_);
              Vertex* vertex1 = graph_->pinDrvrVertex(a);
              Vertex* vertex2 = graph_->pinDrvrVertex(b);
              int level1 = vertex1->level();
              int level2 = vertex2->level();
              return slack1 < slack2 || (slack1 == slack2 && level1 > level2)
                     || (slack1 == slack2 && level1 == level2
                         && network_->pathNameLess(a, b));
            });
}

void ViolatorCollector::sortByTNS()
{
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

void ViolatorCollector::collectByPaths(int numPaths, int numEndpoints)
{
  // For skipping duplicates
  set<const Pin*> viol_pins;

  collectViolatingEndpoints();

  size_t old_size = viol_pins.size();
  int endpoint_count = 0;
  for (const auto& end_original_slack : violating_ends_) {
    const Pin* endpoint_pin = end_original_slack.first;
    Vertex* end = graph_->pinDrvrVertex(endpoint_pin);
    set<const Pin*> end_pins = collectPinsByPathEndpoint(end->pin(), numPaths);
    viol_pins.insert(end_pins.begin(), end_pins.end());

    debugPrint(
        logger_,
        RSZ,
        "violator_collector",
        2,
        "Collected {} pins ({} unique) for endpoint {} total collected {}",
        end_pins.size(),
        viol_pins.size() - old_size,
        network_->pathName(end->pin()),
        viol_pins.size());

    old_size = viol_pins.size();
    endpoint_count++;
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
               3,
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
                   4,
                   "  - Port: {}",
                   network_->pathName(pin));
      } else if (inst) {
        const sta::LibertyCell* lib_cell = network_->libertyCell(inst);
        if (lib_cell) {
          float pin_slack = path->slack(search_);
          debugPrint(logger_,
                     RSZ,
                     "violator_collector",
                     4,
                     "  - Gate: {} (Type: {}) -> Pin: {} Slack: {}",
                     network_->pathName(inst),
                     lib_cell->name(),
                     network_->portName(pin),
                     delayAsString(pin_slack, sta_, 3));
          viol_pins.insert(pin);
        }
      }
    }
    debugPrint(logger_, RSZ, "violator_collector", 4, "\n");
  }

  return viol_pins;
}

}  // namespace rsz
