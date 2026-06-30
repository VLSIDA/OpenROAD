// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <memory>

#include "MoveCandidate.hh"
#include "OptimizerTypes.hh"
#include "rsz/Resizer.hh"

namespace sta {
class Instance;
class LibertyPort;
class Pin;
}  // namespace sta

namespace rsz {

// Candidate that swaps two functionally equivalent input pins on one gate.
//
// When a cell has symmetric input arcs (determined by Boolean functional
// equivalence), moving the critical-path signal to a faster arc can reduce
// delay without changing the netlist topology.  The generator precomputes the
// net improvement -- the gate output-arc gain MINUS the change in the critical
// input net's driver delay (the swapped pins can have different input caps,
// e.g. a tapered stack) -- and estimate() funnels it through the shared
// accept rule.  apply() rewires the two input nets via Resizer::swapPins.
class SwapPinsCandidate : public MoveCandidate
{
 public:
  // === Construction =========================================================
  SwapPinsCandidate(Resizer& resizer,
                    const Target& target,
                    sta::Instance* drvr,
                    sta::LibertyPort* drvr_port,
                    sta::LibertyPort* input_port,
                    sta::LibertyPort* swap_port,
                    float net_improvement);

  // === MoveCandidate API ====================================================
  Estimate estimate() override;
  MoveResult apply() override;
  MoveType type() const override { return MoveType::kSwapPins; }

 private:
  // === Candidate state ======================================================
  sta::Instance* drvr_{nullptr};
  sta::LibertyPort* drvr_port_{nullptr};
  sta::LibertyPort* input_port_{nullptr};
  sta::LibertyPort* swap_port_{nullptr};
  float net_improvement_{0.0f};
};

}  // namespace rsz
