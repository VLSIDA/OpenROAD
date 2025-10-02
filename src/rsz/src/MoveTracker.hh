// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace sta {
class Pin;
class Sta;
}  // namespace sta

namespace utl {
class Logger;
}  // namespace utl

namespace rsz {

enum class MoveStateType
{
  ATTEMPT = 0,
  ATTEMPT_REJECT = 1,
  ATTEMPT_COMMIT = 2
};

struct MoveStateData
{
  const sta::Pin* pin;
  int order;
  std::string move_type;
  MoveStateType state;

  MoveStateData(const sta::Pin* p, int c, const std::string mt, MoveStateType s)
      : pin(p), order(c), move_type(mt), state(s)
  {
  }

  MoveStateData(const sta::Pin* p, const std::string mt, MoveStateType s)
      : pin(p), order(0), move_type(mt), state(s)
  {
  }
};

// Class to track optimization moves (attempts, commits, rejections) for pins.
// Provides statistics and summaries of which pins were attempted for optimization,
// which moves succeeded, and which failed.
class MoveTracker
{
 public:
  MoveTracker(utl::Logger* logger, sta::Sta* sta);

  // Track that a pin was identified as a violator for potential optimization
  void trackViolator(const sta::Pin* pin);

  // Track an attempted move on a pin
  void trackMove(const sta::Pin* pin,
                 const std::string& move_type,
                 MoveStateType state);

  // Commit all pending moves (mark them as successful)
  void commitMoves();

  // Reject all pending moves (mark them as failed)
  void rejectMoves();

  // Print statistics summary for current pass and cumulative totals
  void printMoveSummary();

  // Get the visit count for a specific pin
  int getVisitCount(const sta::Pin* pin) const;

  // Clear all tracking data for the current pass
  void clear();

  // Get total statistics
  int getTotalAttempts() const { return total_attempt_count_; }
  int getTotalCommits() const { return total_commit_count_; }
  int getTotalRejects() const { return total_reject_count_; }

 private:
  void clearMoveSummary();

  utl::Logger* logger_;
  sta::Sta* sta_;

  // Current pass tracking
  int move_count_;
  std::map<const sta::Pin*, int> visit_count_;
  std::vector<MoveStateData> moves_;
  std::vector<MoveStateData> pending_moves_;

  // Cumulative statistics across all passes
  int total_move_count_;
  int total_no_attempt_count_;
  int total_attempt_count_;
  int total_reject_count_;
  int total_commit_count_;
  std::map<std::string, std::tuple<int, int, int>> total_move_type_counts_;
};

}  // namespace rsz
