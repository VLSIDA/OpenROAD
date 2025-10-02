// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "MoveTracker.hh"

#include <algorithm>

#include "sta/Sta.hh"
#include "utl/Logger.h"

namespace rsz {

using std::map;
using std::string;
using std::tuple;
using std::vector;
using utl::RSZ;

using sta::Pin;

MoveTracker::MoveTracker(utl::Logger* logger, sta::Sta* sta)
    : logger_(logger),
      sta_(sta),
      move_count_(0),
      total_move_count_(0),
      total_no_attempt_count_(0),
      total_attempt_count_(0),
      total_reject_count_(0),
      total_commit_count_(0)
{
}

void MoveTracker::clear()
{
  move_count_ = 0;
  visit_count_.clear();
  moves_.clear();
  pending_moves_.clear();
}

void MoveTracker::clearMoveSummary()
{
  move_count_ = 0;
  moves_.clear();
  visit_count_.clear();
}

void MoveTracker::trackViolator(const sta::Pin* pin)
{
  if (!logger_->debugCheck(RSZ, "move_summary", 1)) {
    return;
  }
  if (visit_count_.find(pin) == visit_count_.end()) {
    visit_count_[pin] = 0;
  }
  visit_count_.at(pin)++;
}

void MoveTracker::trackMove(const sta::Pin* pin,
                             const string& move_type,
                             MoveStateType state)
{
  if (!logger_->debugCheck(RSZ, "move_summary", 1)) {
    return;
  }
  assert(visit_count_.find(pin) != visit_count_.end()
         && "Pin must be visited before tracking moves.");
  pending_moves_.emplace_back(pin, move_type, state);
}

void MoveTracker::commitMoves()
{
  for (const auto& pending_move : pending_moves_) {
    moves_.emplace_back(pending_move.pin,
                        move_count_++,
                        pending_move.move_type,
                        MoveStateType::ATTEMPT_COMMIT);
  }
  pending_moves_.clear();
}

void MoveTracker::rejectMoves()
{
  for (const auto& pending_move : pending_moves_) {
    moves_.emplace_back(pending_move.pin,
                        move_count_++,
                        pending_move.move_type,
                        MoveStateType::ATTEMPT_REJECT);
  }
  pending_moves_.clear();
}

int MoveTracker::getVisitCount(const sta::Pin* pin) const
{
  auto it = visit_count_.find(pin);
  return (it != visit_count_.end()) ? it->second : 0;
}

void MoveTracker::printMoveSummary()
{
  if (!logger_->debugCheck(RSZ, "move_summary", 1)) {
    return;
  }
  if (moves_.size() == 0) {
    return;
  }

  // attempt, reject, commit
  map<string, tuple<int, int, int>> move_type_counts;
  // We tried a move
  vector<const Pin*> attempted_pins;
  // We tried a move and doMove succeeded, but it was rejected
  vector<const Pin*> rejected_pins;
  // We tried a move and it was committed
  vector<const Pin*> committed_pins;

  for (const auto& move : moves_) {
    const Pin* pin = move.pin;

    switch (move.state) {
      case MoveStateType::ATTEMPT:
        attempted_pins.push_back(pin);
        std::get<(int) MoveStateType::ATTEMPT>(
            move_type_counts[move.move_type])++;
        std::get<(int) MoveStateType::ATTEMPT>(
            total_move_type_counts_[move.move_type])++;
        break;
      case MoveStateType::ATTEMPT_REJECT:
        attempted_pins.push_back(pin);
        std::get<(int) MoveStateType::ATTEMPT>(
            move_type_counts[move.move_type])++;
        std::get<(int) MoveStateType::ATTEMPT>(
            total_move_type_counts_[move.move_type])++;
        rejected_pins.push_back(pin);
        std::get<(int) MoveStateType::ATTEMPT_REJECT>(
            move_type_counts[move.move_type])++;
        std::get<(int) MoveStateType::ATTEMPT_REJECT>(
            total_move_type_counts_[move.move_type])++;
        break;
      case MoveStateType::ATTEMPT_COMMIT:
        attempted_pins.push_back(pin);
        std::get<(int) MoveStateType::ATTEMPT>(
            move_type_counts[move.move_type])++;
        std::get<(int) MoveStateType::ATTEMPT>(
            total_move_type_counts_[move.move_type])++;
        committed_pins.push_back(pin);
        std::get<(int) MoveStateType::ATTEMPT_COMMIT>(
            move_type_counts[move.move_type])++;
        std::get<(int) MoveStateType::ATTEMPT_COMMIT>(
            total_move_type_counts_[move.move_type])++;
        break;
    }
  }

  int no_attempt_count = 0;
  for (const auto& [pin, visit_count] : visit_count_) {
    if (std::find(attempted_pins.begin(), attempted_pins.end(), pin)
        != attempted_pins.end()) {
      continue;
    }
    no_attempt_count++;
  }

  debugPrint(logger_, RSZ, "move_summary", 2, "Current pass statistics:");
  debugPrint(logger_,
             RSZ,
             "move_summary",
             1,
             "Current Summary: Not Attempted: {} Attempts: {} Rejects: "
             "{} Commits: {} ",
             no_attempt_count,
             attempted_pins.size(),
             rejected_pins.size(),
             committed_pins.size());
  float attempt_rate_
      = (moves_.size() > 0)
            ? (static_cast<float>(attempted_pins.size()) / moves_.size()) * 100
            : 0;
  float reject_rate_
      = (moves_.size() > 0)
            ? (static_cast<float>(rejected_pins.size()) / moves_.size()) * 100
            : 0;
  float commit_rate_
      = (moves_.size() > 0)
            ? (static_cast<float>(committed_pins.size()) / moves_.size()) * 100
            : 0;
  debugPrint(logger_,
             RSZ,
             "move_summary",
             1,
             "Overall attempt_rate: {:.2f}% ({}) reject_rate: {:0.2f}% "
             "({}) commit_rate: {:0.2f}% ({})",
             attempt_rate_,
             attempted_pins.size(),
             reject_rate_,
             rejected_pins.size(),
             commit_rate_,
             committed_pins.size());

  total_no_attempt_count_ += no_attempt_count;
  total_attempt_count_ += attempted_pins.size();
  total_reject_count_ += rejected_pins.size();
  total_commit_count_ += committed_pins.size();
  total_move_count_ += move_count_;

  for (const auto& [move_type, counts] : move_type_counts) {
    float move_attempt_count
        = static_cast<float>(std::get<(int) MoveStateType::ATTEMPT>(counts));
    float move_reject_count = static_cast<float>(
        std::get<(int) MoveStateType::ATTEMPT_REJECT>(counts));
    float move_commit_count = static_cast<float>(
        std::get<(int) MoveStateType::ATTEMPT_COMMIT>(counts));
    float move_attempt_rate
        = (moves_.size() > 0) ? (move_attempt_count / moves_.size()) * 100 : 0;
    float move_reject_rate
        = (moves_.size() > 0) ? (move_reject_count / moves_.size()) * 100 : 0;
    float move_commit_rate
        = (moves_.size() > 0) ? (move_commit_count / moves_.size()) * 100 : 0;
    debugPrint(logger_,
               RSZ,
               "move_summary",
               1,
               "{} attempt_rate: {:.2f}% ({}) reject_rate: {:.2f}% ({})  "
               "commit_rate: {:.2f}% ({})",
               move_type,
               move_attempt_rate,
               move_attempt_count,
               move_reject_rate,
               move_reject_count,
               move_commit_rate,
               move_commit_count);
  }

  debugPrint(logger_, RSZ, "move_summary", 2, "Total statistics:");
  debugPrint(logger_,
             RSZ,
             "move_summary",
             1,
             "Total Summary: Not Attempted: {} Attempts: {} Rejects: "
             "{} Commits: {} ",
             total_no_attempt_count_,
             total_attempt_count_,
             total_reject_count_,
             total_commit_count_);
  float total_attempt_rate_
      = (total_move_count_ > 0)
            ? (static_cast<float>(total_attempt_count_) / total_move_count_)
                  * 100
            : 0;
  float total_reject_rate_
      = (total_move_count_ > 0)
            ? (static_cast<float>(total_reject_count_) / total_move_count_)
                  * 100
            : 0;
  float total_commit_rate_
      = (total_move_count_ > 0)
            ? (static_cast<float>(total_commit_count_) / total_move_count_)
                  * 100
            : 0;
  debugPrint(logger_,
             RSZ,
             "move_summary",
             1,
             "Overall attempt_rate: {:.2f}% ({}) reject_rate: {:0.2f}% "
             "({}) commit_rate: {:0.2f}% ({})",
             total_attempt_rate_,
             total_attempt_count_,
             total_reject_rate_,
             total_reject_count_,
             total_commit_rate_,
             total_commit_count_);

  for (const auto& [move_type, counts] : total_move_type_counts_) {
    float total_move_attempt_count
        = static_cast<float>(std::get<(int) MoveStateType::ATTEMPT>(counts));
    float total_move_reject_count = static_cast<float>(
        std::get<(int) MoveStateType::ATTEMPT_REJECT>(counts));
    float total_move_commit_count = static_cast<float>(
        std::get<(int) MoveStateType::ATTEMPT_COMMIT>(counts));

    float total_move_attempt_rate
        = (total_move_count_ > 0)
              ? (total_move_attempt_count / total_move_count_) * 100
              : 0;
    float total_move_reject_rate
        = (total_move_count_ > 0)
              ? (total_move_reject_count / total_move_count_) * 100
              : 0;
    float total_move_commit_rate
        = (total_move_count_ > 0)
              ? (total_move_commit_count / total_move_count_) * 100
              : 0;
    debugPrint(logger_,
               RSZ,
               "move_summary",
               1,
               "{} attempt_rate: {:.2f}% ({}) reject_rate: {:.2f}% ({})  "
               "commit_rate: {:.2f}% ({})",
               move_type,
               total_move_attempt_rate,
               total_move_attempt_count,
               total_move_reject_rate,
               total_move_reject_count,
               total_move_commit_rate,
               total_move_commit_count);
  }
  clearMoveSummary();
}

}  // namespace rsz
