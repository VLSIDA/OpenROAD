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
  SORT_BY_LOAD_DELAY = 0,
  SORT_BY_WNS = 1,
  SORT_BY_TNS = 2,
  MAX = 3
};

struct pinData
{
  std::string name;
  Delay slack;
  Delay tns;
  float intrinsic_delay;
  float load_delay;
  int level;
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
    dcalc_ap_ = nullptr;
    lib_ap_ = -1;
    move_count_ = 0;
  }

  void init(float slack_margin);
  void printHistogram(int numBins = 20) const;
  void printViolators(int numPrint) const;

  // For backwards compatibility to the old resizer
  vector<const Pin*> collectViolatorsByEndpoint(
      int endpoint_index,
      ViolatorSortType sort_type = ViolatorSortType::SORT_BY_LOAD_DELAY);

  vector<const Pin*> collectViolators(int endPointIndex = -1,
                                      int numEndpoints = 1,
                                      int numPathsPerEndpoint = 1,
                                      int numPins = 0,
                                      ViolatorSortType sort_type
                                      = ViolatorSortType::SORT_BY_LOAD_DELAY);
  void trackMove(const Pin* pin, std::string move_type, bool accept);
  void printMoveSummary();
  void clearMoveSummary();

 private:
  const char* getEnumString(ViolatorSortType sort_type);
  void collectViolatingEndpoints();
  void updatePinData(const Pin* pin, pinData& pd);

  set<const Pin*> collectPinsByPathEndpoint(const sta::Pin* endpoint_pin,
                                            size_t paths_per_endpoint = 1);
  void collectBySlack();
  void collectByPaths(int endPointIndex = 1,
                      int numEndpoints = 1,
                      int numPathsPerEndpoint = 1);

  void sortPins(int numPins = 0,
                ViolatorSortType sort_type
                = ViolatorSortType::SORT_BY_LOAD_DELAY);
  void sortByLoadDelay();
  void sortByWNS();
  void sortByLocalTNS();
  std::map<const Pin*, Delay> getLocalTNS() const;
  Delay getLocalPinTNS(const Pin* pin) const;

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
  sta::DcalcAnalysisPt* dcalc_ap_;
  int lib_ap_;

  float slack_margin_;
  vector<const Pin*> violating_pins_;
  std::map<const Pin*, pinData> pin_data_;
  vector<std::pair<const Pin*, Slack>> violating_ends_;

  int move_count_ = 0;
  std::map<const Pin*, int> visit_count_;
  std::vector<std::tuple<const Pin*, int, std::string, bool>> moves_;
};

}  // namespace rsz
