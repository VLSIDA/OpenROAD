// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include <cmath>

#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/Sta.hh"
#include "utl/Logger.h"

namespace rsz {

using sta::Pin;
using std::set;
using std::vector;
using utl::RSZ;

enum class ViolatorSortType
{
  SORT_BY_TNS,
  SORT_BY_WNS,
  SORT_BY_LOAD_DELAY
};

// Class to collect instances with violating output pins.
class ViolatorCollector
{
 public:
  // Constructor
  ViolatorCollector(Resizer* resizer) : resizer_(resizer)
  {
    logger_ = resizer_->logger_;
    sta_ = resizer_->sta_;
    network_ = resizer_->network_;
    max_ = resizer_->max_;
  }

  void init(float slack_margin);
  void printHistogram(int numBins = 20) const;
  void printViolators(int numPrint) const;
  void collectBySlack(int numPins = 1000);
  void collectByPaths(int numPaths = 1);
  set<const Pin*> collectPinsByPathEndpoint(const sta::Pin* endpoint_pin,
                                            size_t paths_per_endpoint = 1);
  void sortByLoadDelay();
  void sortByWNS();
  void sortByTNS();

  vector<const Pin*> collectViolators(int numPaths = 1,
                                      ViolatorSortType sort_type
                                      = ViolatorSortType::SORT_BY_LOAD_DELAY);

 private:
  float slack_margin_;
  vector<const Pin*> violating_pins_;
  Resizer* resizer_;
  Logger* logger_;
  sta::Sta* sta_;
  sta::Graph* graph_;
  sta::Network* network_;
  sta::dbNetwork* db_network_;
  const MinMax* max_;
  sta::Search* search_;
  sta::Sdc* sdc_;
  sta::Report* report_;
  sta::Corner* corner_;
};

}  // namespace rsz
