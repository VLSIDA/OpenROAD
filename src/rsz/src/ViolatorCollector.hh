// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include <cmath>

#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/Sta.hh"
#include "utl/Logger.h"

namespace rsz {

using sta::Pin;
using sta::Slack;
using sta::Vertex;
using sta::VertexSet;
using std::pair;
using std::set;
using std::vector;
using utl::RSZ;

enum class ViolatorSortType
{
  SORT_BY_LOAD_DELAY = 0,
  SORT_BY_WNS = 1,
  SORT_BY_TNS = 2,
  SORT_BY_HEURISTIC = 3,
  MAX = 4
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
    max_passes_per_endpoint_ = 1000;  // Default value
  }

  void init(float slack_margin);
  void printHistogram(int numBins = 20) const;
  void printViolators(int numPrint) const;

  int repairsPerPass(int max_repairs_per_pass);

  // Endpoint pass tracking
  void setMaxPassesPerEndpoint(int max_passes);
  bool shouldSkipEndpoint() const;
  int getEndpointPassCount() const;
  void resetEndpointPasses();

  // Endpoint iteration - ViolatorCollector manages iteration internally
  bool hasMoreEndpoints() const;
  void advanceToNextEndpoint();
  void setToEndpoint(int index);
  Vertex* getCurrentEndpoint() const { return current_endpoint_; }
  Slack getCurrentEndpointSlack() const;
  Slack getCurrentEndpointOriginalSlack() const
  {
    return current_end_original_slack_;
  }
  int getCurrentEndpointIndex() const { return current_endpoint_index_; }
  int getMaxEndpointCount() const { return violating_ends_.size(); }
  int getCurrentEndpointPass() const;
  void useWorstEndpoint(Vertex* end);

  // Collect violators for the current endpoint
  vector<const Pin*> collectViolators(int numPathsPerEndpoint = 1,
                                      int numPins = 1000,
                                      ViolatorSortType sort_type
                                      = ViolatorSortType::SORT_BY_LOAD_DELAY);

  vector<const Pin*> collectViolatorsFromEndpoints(
      const vector<Vertex*>& endpoints,
      int numPathsPerEndpoint,
      int numPins,
      ViolatorSortType sort_type);

  // For statistics on critical paths
  int getTotalViolations() const;

  // Public utility methods
  const char* getEnumString(ViolatorSortType sort_type);

 private:
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
  void sortByHeuristic();
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

  // Endpoint pass tracking
  int max_passes_per_endpoint_;
  int current_endpoint_pass_count_;

  // Current endpoint iteration state
  Vertex* current_endpoint_;
  Slack current_end_original_slack_;
  int current_endpoint_index_;
};

}  // namespace rsz
