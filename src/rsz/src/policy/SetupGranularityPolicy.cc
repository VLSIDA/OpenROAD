// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SetupGranularityPolicy.hh"

#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace rsz {

// Coarse: large area / placement-displacement moves (need legalization).
void SetupCoarsePolicy::buildMainMoveSequence(const bool log_sequence)
{
  move_sequence_.clear();
  move_sequence_.push_back(MoveType::kSizeUp);
  pushMoveIfEnabled(!config_.skip_gate_cloning, MoveType::kClone);
  pushMoveIfEnabled(!config_.skip_buffering, MoveType::kBuffer);
  pushMoveIfEnabled(!config_.skip_buffering, MoveType::kSplitLoad);
  activateMoveSequence(log_sequence);
}

// Fine: little/no area change or legalization.
void SetupFinePolicy::buildMainMoveSequence(const bool log_sequence)
{
  move_sequence_.clear();
  pushMoveIfEnabled(!config_.skip_vt_swap && hasVtSwapCells(),
                    MoveType::kVtSwap);
  pushMoveIfEnabled(!config_.skip_pin_swap, MoveType::kSwapPins);
  pushMoveIfEnabled(!config_.skip_size_down_fanout, MoveType::kSizeDownFanout);
  pushMoveIfEnabled(!config_.skip_buffer_removal, MoveType::kUnbuffer);
  move_sequence_.push_back(MoveType::kReroute);
  activateMoveSequence(log_sequence);
}

}  // namespace rsz
