// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#pragma once

#include <map>
#include <memory>
#include <set>
#include <vector>

#include "OptimizerTypes.hh"
#include "sta/ArcDelayCalc.hh"

namespace sta {
class Parasitic;
class Pvt;
class TimingArc;
class Vertex;
}  // namespace sta

namespace rsz {

class Resizer;
struct Target;

// === Local subgraph STA for the -neighbor_check veto =======================
//
// SubgraphTimer evaluates a candidate move's timing effect on a small local
// subgraph -- the target driver stage plus one fanin driver stage per target
// input pin -- with the full STA delay model: the real timing arcs, the real
// input slews from the timing graph, and the real parasitics, all through
// sta::ArcDelayCalc::gateDelay().  Boundary conditions are FROZEN: arrivals
// and slews entering the subgraph and required times at its frontier are
// assumed unchanged by the move, so a frontier pin's post-move slack is its
// current (cache-read) slack minus the locally computed arrival change.
// OLD and NEW electrical states are evaluated through the SAME local model,
// so any systematic model-vs-graph bias cancels in the delta.
//
// Discipline (matches the constraints this feature was built under):
//  - OpenSTA is never modified and the global timing graph / search state is
//    never written or dirtied: no journal apply, no setPiModel, nothing that
//    triggers findArrivals()/findRequireds().  The only STA calls are
//    ArcDelayCalc::gateDelay() + finishDrvrPin() plus pure cache reads
//    guarded by arrivalsValid() && requiredsExist().
//  - Main thread only: the shared ArcDelayCalc is not reentrant.  Policies
//    that generate on worker threads clear
//    OptimizationPolicyConfig::generate_on_main_thread and the check skips
//    permissively.
//  - Any missing ingredient (guard fails, no matching arc, oversized
//    subgraph) makes build()/evaluate*() return false and the caller skips
//    the veto for that candidate (permissive: the ordinary accept machinery
//    still rejects non-improving moves).  A missing parasitic is NOT a
//    failure -- gateDelay() handles a null parasitic with the lumped load
//    cap, still bias-cancelling.

// One frontier load pin: where a locally computed arrival change is charged
// against a frozen required time (via the pin's current slack).
struct SubgraphLoad
{
  const sta::Pin* pin{nullptr};
  sta::Vertex* vertex{nullptr};
  size_t load_index{0};      // index in the owning stage's LoadPinIndexMap
  float slack_before{0.0f};  // guarded cache read
  bool on_path{false};       // the endpoint-path load of the target stage
  // Depth-1 receiver conversion: one Liberty table lookup turning the change
  // in this pin's input slew into a delay change inside the receiving gate
  // (charged at this same frontier vertex; no deeper recursion).
  const sta::TimingArc* rcv_arc{nullptr};
  const sta::Pvt* rcv_pvt{nullptr};
  float rcv_load_cap{0.0f};
};

// One evaluated driver stage (the target driver or one fanin driver).
struct SubgraphStage
{
  sta::Pin* drvr_pin{nullptr};
  sta::Vertex* drvr_vertex{nullptr};
  sta::Instance* inst{nullptr};
  const sta::TimingArc* arc{nullptr};  // path arc (target) / graph-worst arc
  const sta::Pvt* pvt{nullptr};
  float input_slew{0.0f};
  float load_cap{0.0f};
  sta::Parasitic* parasitic{nullptr};  // may be null (lumped-cap dcalc)
  std::unique_ptr<sta::LoadPinIndexMap> load_map;
  // For fanin stages: which target input pin this stage's net feeds and its
  // index in load_map (the on-path coupling point into the target stage).
  const sta::Pin* target_input_pin{nullptr};
  size_t target_input_load_index{0};
  bool has_target_input_index{false};
  bool feeds_on_path_input{false};
  // OLD-model baseline (from build()).
  float old_gate_delay{0.0f};
  float old_drvr_slew{0.0f};
  std::vector<float> old_wire_delay;  // by load_index
  std::vector<float> old_load_slew;   // by load_index
  float graph_gate_delay{0.0f};       // debug cross-check only
  std::vector<SubgraphLoad> frontier;
};

// Result of one stage dcalc pass.
struct SubgraphStageEval
{
  bool valid{false};
  float gate_delay{0.0f};
  float drvr_slew{0.0f};
  std::vector<float> wire_delay;  // by load_index
  std::vector<float> load_slew;   // by load_index
};

// Declarative what-if for the generic evaluate() walker.  A move is a set
// of substitutions applied to the region's stages; the walker owns the
// composition (fanin re-evaluation, slew coupling into the center, frontier
// deltas), so new moves are new builders rather than new engines.
struct SubgraphMove
{
  // Replace an instance's cell (center gate for size up / VT swap / size
  // down fanout).  Fanin stages feeding a substituted instance pick up the
  // input-pin cap deltas automatically.
  std::map<const sta::Instance*, const sta::LibertyCell*> cell_subs;
  // Explicit load-cap deltas by stage driver pin (clone's duplicated fanin
  // load, split/clone's reduced center load).
  std::map<const sta::Pin*, float> load_cap_delta;
  // Table-only appendage stages hanging off the center (split buffer,
  // clone gate).  replaces_center: the moved loads leave the center gate
  // entirely (clone) rather than being driven through it (split buffer).
  struct VirtualStage
  {
    const sta::LibertyCell* cell{nullptr};
    std::set<const sta::Pin*> moved_loads;
    bool replaces_center{false};
  };
  std::vector<VirtualStage> virtual_stages;
};

class SubgraphTimer
{
 public:
  explicit SubgraphTimer(Resizer& resizer);

  // Snapshot the frozen subgraph around any center gate: the center driver
  // stage plus one fanin driver stage per center input pin.  The center
  // need not be the path driver (e.g. size down fanout centers on the
  // downsized fanout gate; the critical driver is then one of its fanin
  // stages and "fanins of fanouts" fall out of the same construction).
  // Uses the path arc when the center is the endpoint-path driver, the
  // graph-worst arc otherwise.  Main thread only; false => caller skips
  // the veto (permissive).
  bool build(const Target& target, sta::Instance* inst, sta::Pin* drvr_pin);

  // Generic walker: evaluate the declarative move over the region.  On
  // success fills the on-path observation and the neighbor observations
  // for wnsDegraded() and returns true.
  bool evaluate(const SubgraphMove& move,
                LocalSlack& on_path,
                std::vector<LocalSlack>& neighbors);

  // Move builder: cell substitution at the center (size up / VT swap).
  bool evaluateCellSwap(const sta::LibertyCell* candidate,
                        LocalSlack& on_path,
                        std::vector<LocalSlack>& neighbors);

  // Evaluate a commutative-pin swap at the target instance: the critical
  // input moves to swap_port and the net currently on swap_port moves to
  // input_port.
  bool evaluatePinSwap(const sta::LibertyPort* input_port,
                       const sta::LibertyPort* swap_port,
                       LocalSlack& on_path,
                       std::vector<LocalSlack>& neighbors);

  // Move builder: split the given loads off the center net behind a new
  // buffer of buffer_cell (Liberty-table virtual stage: the buffer's net
  // does not exist yet, so no parasitic can exist for it).
  bool evaluateSplitLoad(const sta::LibertyCell* buffer_cell,
                         const sta::PinSet& moved_loads,
                         LocalSlack& on_path,
                         std::vector<LocalSlack>& neighbors);

  // Move builder: clone the center gate -- moved_loads transfer to a
  // table-only clone stage and every fanin net gains the clone's full
  // input cap.
  bool evaluateClone(const sta::LibertyCell* clone_cell,
                     const std::vector<sta::Pin*>& moved_loads,
                     LocalSlack& on_path,
                     std::vector<LocalSlack>& neighbors);

 private:
  bool guardOK() const;
  bool cachedSlack(sta::Vertex* vertex, float& slack_out) const;
  static const sta::Pvt* findPvt(const sta::Scene* scene,
                                 sta::Instance* inst,
                                 const sta::MinMax* min_max);
  // Liberty-table gate delay + output slew for one arc (no parasitics).
  static bool tableDelay(const sta::Pvt* pvt,
                         const sta::TimingArc* arc,
                         float in_slew,
                         float load_cap,
                         float& delay,
                         float& out_slew);
  // Candidate cell's arc matching ref_arc (exact, then relaxed).
  static const sta::TimingArc* findCandidateArc(
      const sta::TimingArc* ref_arc,
      const sta::LibertyCell* candidate,
      const sta::Scene* scene,
      const sta::MinMax* min_max);
  // Graph-worst gate arc driving `drvr_vertex` (for off-path fanin stages
  // that have no path arc) together with its from-pin input slew.
  bool worstGraphArcInto(sta::Vertex* drvr_vertex,
                         const sta::TimingArc*& arc,
                         float& input_slew) const;
  // Populate common stage fields + OLD baseline; false => build fails.
  bool buildStage(sta::Pin* drvr_pin,
                  const sta::TimingArc* arc,
                  float input_slew,
                  SubgraphStage& stage);
  // Frontier collection for `stage`, excluding `exclude_pin` (the target
  // input pin on fanin stages).  on_path_pin marks the endpoint-path load.
  void collectFrontier(SubgraphStage& stage,
                       const sta::Pin* exclude_pin,
                       const sta::Pin* on_path_pin);
  // One dcalc pass of a stage with substituted electrical state.
  bool evalStage(const SubgraphStage& stage,
                 const sta::TimingArc* arc,
                 float input_slew,
                 float load_cap,
                 SubgraphStageEval& out) const;
  // Depth-1 receiver conversion: delay change inside the receiving gate when
  // this frontier pin's input slew changes from old_slew to new_slew.
  float receiverSlewToDelay(const SubgraphLoad& load,
                            float old_slew,
                            float new_slew) const;
  // Change of this stage's contribution to a frontier load's arrival when
  // the stage is re-evaluated as `eval` (gate + wire + receiver slew term).
  float loadArrivalDelta(const SubgraphStage& stage,
                         const SubgraphStageEval& eval,
                         const SubgraphLoad& load) const;
  // Emit LocalSlack entries for a re-evaluated stage's frontier.
  // extra_delta is added to every load (upstream on-path coupling).
  void appendFrontierSlacks(const SubgraphStage& stage,
                            const SubgraphStageEval& eval,
                            float extra_delta,
                            LocalSlack& on_path,
                            bool& on_path_seen,
                            std::vector<LocalSlack>& neighbors) const;

  // Per-fanin-stage load-cap delta implied by the move: any explicit
  // load_cap_delta on the stage's driver pin plus the input-pin cap change
  // when the instance the stage feeds has a cell substitution.
  bool faninCapDeltas(const SubgraphMove& move,
                      std::vector<float>& delta_by_stage) const;

  Resizer& resizer_;
  const sta::Scene* scene_{nullptr};
  const sta::MinMax* min_max_{nullptr};
  bool valid_{false};
  float target_slack_{0.0f};  // fallback on-path observation
  const sta::Pin* on_path_load_pin_{nullptr};
  SubgraphStage target_stage_;
  std::vector<SubgraphStage> fanin_stages_;

  static constexpr int kMaxFaninStages = 8;
  static constexpr int kMaxFrontierLoads = 64;
};

}  // namespace rsz
