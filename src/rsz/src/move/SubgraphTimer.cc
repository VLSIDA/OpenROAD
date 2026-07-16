// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026-2026, The OpenROAD Authors

#include "SubgraphTimer.hh"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/LibertyClass.hh"
#include "sta/MinMax.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Parasitics.hh"
#include "sta/Path.hh"
#include "sta/PathExpanded.hh"
#include "sta/PortDirection.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/TableModel.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingRole.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace rsz {

namespace {

constexpr float kMinSlewRatioDenominator = 1.0e-15f;

}  // namespace

SubgraphTimer::SubgraphTimer(Resizer& resizer) : resizer_(resizer)
{
}

bool SubgraphTimer::guardOK() const
{
  sta::Search* search = resizer_.sta()->search();
  return search->arrivalsValid() && search->requiredsExist();
}

bool SubgraphTimer::cachedSlack(sta::Vertex* vertex, float& slack_out) const
{
  // Pure cache read: build() already verified arrivals/requireds exist.
  if (vertex == nullptr) {
    return false;
  }
  slack_out = sta::delayAsFloat(
      resizer_.sta()->slack(vertex, resizer_.maxAnalysisMode()));
  return true;
}

const sta::Pvt* SubgraphTimer::findPvt(const sta::Scene* scene,
                                       sta::Instance* inst,
                                       const sta::MinMax* min_max)
{
  if (scene == nullptr || min_max == nullptr) {
    return nullptr;
  }
  const sta::Pvt* pvt = scene->sdc()->pvt(inst, min_max);
  if (pvt == nullptr) {
    pvt = scene->sdc()->operatingConditions(min_max);
  }
  return pvt;
}

bool SubgraphTimer::tableDelay(const sta::Pvt* pvt,
                               const sta::TimingArc* arc,
                               const float in_slew,
                               const float load_cap,
                               float& delay,
                               float& out_slew)
{
  const sta::GateTableModel* model
      = dynamic_cast<const sta::GateTableModel*>(arc->model());
  if (model == nullptr) {
    return false;
  }
  out_slew = 0.0f;
  model->gateDelay(pvt, in_slew, load_cap, delay, out_slew);
  return true;
}

const sta::TimingArc* SubgraphTimer::findCandidateArc(
    const sta::TimingArc* ref_arc,
    const sta::LibertyCell* candidate,
    const sta::Scene* scene,
    const sta::MinMax* min_max)
{
  if (ref_arc == nullptr || candidate == nullptr) {
    return nullptr;
  }
  sta::LibertyCell* scene_cell
      = const_cast<sta::LibertyCell*>(candidate)->sceneCell(scene, min_max);
  if (scene_cell == nullptr) {
    return nullptr;
  }
  sta::LibertyPort* from = scene_cell->findLibertyPort(ref_arc->from()->name());
  sta::LibertyPort* to = scene_cell->findLibertyPort(ref_arc->to()->name());
  if (from == nullptr || to == nullptr) {
    return nullptr;
  }
  // Exact conditional/default match first, then relaxed port-name + RF.
  const sta::TimingArcSetSeq& arc_sets = scene_cell->timingArcSets(from, to);
  for (const sta::TimingArcSet* arc_set : arc_sets) {
    if (arc_set->role()->isTimingCheck()) {
      continue;
    }
    const sta::TimingArc* arc = findMatchingTimingArc(ref_arc, arc_set);
    if (arc != nullptr) {
      return arc;
    }
  }
  for (const sta::TimingArcSet* arc_set : arc_sets) {
    if (arc_set->role()->isTimingCheck()) {
      continue;
    }
    for (const sta::TimingArc* arc : arc_set->arcs()) {
      if (matchTimingArc(ref_arc, arc, ArcMatchMode::kRelaxedCandidate)
          == ArcMatchType::kRelaxed) {
        return arc;
      }
    }
  }
  return nullptr;
}

bool SubgraphTimer::worstGraphArcInto(sta::Vertex* drvr_vertex,
                                      const sta::TimingArc*& arc,
                                      float& input_slew) const
{
  // Off-path stages have no path arc; use the gate arc the graph says is
  // currently worst (pure cache reads).
  arc = nullptr;
  input_slew = 0.0f;
  const sta::DcalcAPIndex ap_index = scene_->dcalcAnalysisPtIndex(min_max_);
  sta::Graph* graph = resizer_.graph();
  float worst_delay = -std::numeric_limits<float>::max();
  sta::Vertex* worst_from = nullptr;
  sta::VertexInEdgeIterator edge_iter(drvr_vertex, graph);
  while (edge_iter.hasNext()) {
    sta::Edge* edge = edge_iter.next();
    if (edge->isWire() || edge->role()->isTimingCheck()) {
      continue;
    }
    for (const sta::TimingArc* edge_arc : edge->timingArcSet()->arcs()) {
      const float delay
          = sta::delayAsFloat(graph->arcDelay(edge, edge_arc, ap_index));
      if (delay > worst_delay) {
        worst_delay = delay;
        arc = edge_arc;
        worst_from = edge->from(graph);
      }
    }
  }
  if (arc == nullptr || worst_from == nullptr || arc->fromEdge() == nullptr) {
    return false;
  }
  const sta::RiseFall* from_rf = arc->fromEdge()->asRiseFall();
  if (from_rf == nullptr) {
    return false;
  }
  input_slew = resizer_.sta()->graphDelayCalc()->edgeFromSlew(
      worst_from, from_rf, arc->role(), scene_, min_max_);
  return true;
}

bool SubgraphTimer::evalStage(const SubgraphStage& stage,
                              const sta::TimingArc* arc,
                              const float input_slew,
                              const float load_cap,
                              SubgraphStageEval& out) const
{
  out = SubgraphStageEval{};
  if (arc == nullptr || stage.drvr_pin == nullptr
      || stage.load_map == nullptr) {
    return false;
  }
  sta::ArcDelayCalc* dcalc = resizer_.sta()->arcDelayCalc();
  sta::ArcDcalcResult result = dcalc->gateDelay(stage.drvr_pin,
                                                arc,
                                                input_slew,
                                                load_cap,
                                                stage.parasitic,
                                                *stage.load_map,
                                                scene_,
                                                min_max_);
  dcalc->finishDrvrPin();
  out.gate_delay = sta::delayAsFloat(result.gateDelay());
  out.drvr_slew = sta::delayAsFloat(result.drvrSlew());
  if (!std::isfinite(out.gate_delay) || !std::isfinite(out.drvr_slew)) {
    return false;
  }
  const size_t load_count = stage.load_map->size();
  out.wire_delay.resize(load_count, 0.0f);
  out.load_slew.resize(load_count, 0.0f);
  for (size_t i = 0; i < load_count; ++i) {
    out.wire_delay[i] = sta::delayAsFloat(result.wireDelay(i));
    out.load_slew[i] = sta::delayAsFloat(result.loadSlew(i));
  }
  out.valid = true;
  return true;
}

bool SubgraphTimer::buildStage(sta::Pin* drvr_pin,
                               const sta::TimingArc* arc,
                               const float input_slew,
                               SubgraphStage& stage)
{
  stage.drvr_pin = drvr_pin;
  stage.drvr_vertex = resizer_.graph()->pinDrvrVertex(drvr_pin);
  stage.inst = resizer_.network()->instance(drvr_pin);
  stage.arc = arc;
  stage.input_slew = input_slew;
  if (stage.drvr_vertex == nullptr || stage.inst == nullptr || arc == nullptr) {
    return false;
  }
  stage.pvt = findPvt(scene_, stage.inst, min_max_);

  sta::GraphDelayCalc* graph_dcalc = resizer_.sta()->graphDelayCalc();
  stage.load_cap = graph_dcalc->loadCap(drvr_pin, scene_, min_max_);
  stage.load_map = std::make_unique<sta::LoadPinIndexMap>(
      graph_dcalc->makeLoadPinIndexMap(stage.drvr_vertex));

  const sta::RiseFall* out_rf
      = arc->toEdge() != nullptr ? arc->toEdge()->asRiseFall() : nullptr;
  if (out_rf == nullptr) {
    return false;
  }
  // Lookup of the reduced parasitic; a full dcalc pass already ran (guard),
  // so this hits the store.  Null is acceptable (lumped-cap dcalc).
  stage.parasitic = resizer_.sta()->arcDelayCalc()->findParasitic(
      drvr_pin, out_rf, scene_, min_max_);

  SubgraphStageEval baseline;
  if (!evalStage(stage, arc, input_slew, stage.load_cap, baseline)) {
    return false;
  }
  stage.old_gate_delay = baseline.gate_delay;
  stage.old_drvr_slew = baseline.drvr_slew;
  stage.old_wire_delay = std::move(baseline.wire_delay);
  stage.old_load_slew = std::move(baseline.load_slew);
  return true;
}

void SubgraphTimer::collectFrontier(SubgraphStage& stage,
                                    const sta::Pin* exclude_pin,
                                    const sta::Pin* on_path_pin)
{
  sta::Graph* graph = resizer_.graph();
  const sta::DcalcAPIndex ap_index = scene_->dcalcAnalysisPtIndex(min_max_);
  for (const auto& [pin, load_index] : *stage.load_map) {
    if (pin == exclude_pin) {
      if (exclude_pin != nullptr) {
        stage.target_input_pin = pin;
        stage.target_input_load_index = load_index;
        stage.has_target_input_index = true;
      }
      continue;
    }
    sta::Vertex* load_vertex = graph->pinLoadVertex(const_cast<sta::Pin*>(pin));
    float slack_before = 0.0f;
    if (!cachedSlack(load_vertex, slack_before)) {
      continue;  // unobservable: omit (treat as no harm), as elsewhere
    }
    SubgraphLoad load;
    load.pin = pin;
    load.vertex = load_vertex;
    load.load_index = load_index;
    load.slack_before = slack_before;
    load.on_path = (pin == on_path_pin);
    // Depth-1 receiver conversion arc: the graph-worst gate arc out of this
    // input pin, evaluated at the receiver's unchanged output load.
    float worst_delay = -std::numeric_limits<float>::max();
    sta::VertexOutEdgeIterator out_iter(load_vertex, graph);
    while (out_iter.hasNext()) {
      sta::Edge* edge = out_iter.next();
      if (edge->isWire() || edge->role()->isTimingCheck()) {
        continue;
      }
      for (const sta::TimingArc* edge_arc : edge->timingArcSet()->arcs()) {
        const float delay
            = sta::delayAsFloat(graph->arcDelay(edge, edge_arc, ap_index));
        if (delay > worst_delay) {
          worst_delay = delay;
          load.rcv_arc = edge_arc;
          sta::Vertex* rcv_drvr = edge->to(graph);
          load.rcv_load_cap = resizer_.sta()->graphDelayCalc()->loadCap(
              rcv_drvr->pin(), scene_, min_max_);
        }
      }
    }
    if (load.rcv_arc != nullptr) {
      load.rcv_pvt
          = findPvt(scene_, resizer_.network()->instance(pin), min_max_);
    }
    stage.frontier.push_back(load);
  }
}

bool SubgraphTimer::build(const Target& target,
                          sta::Instance* inst,
                          sta::Pin* drvr_pin)
{
  valid_ = false;
  fanin_stages_.clear();
  target_stage_ = SubgraphStage{};
  if (!guardOK() || inst == nullptr || drvr_pin == nullptr
      || !target.canBePathDriver()) {
    return false;
  }

  scene_ = target.activeScene(resizer_);
  min_max_ = target.minMax(resizer_);
  if (scene_ == nullptr || min_max_ == nullptr) {
    return false;
  }
  target_slack_ = sta::delayAsFloat(target.slack);

  // Path arc of the target stage, resolved onto the scene cell.
  sta::PathExpanded expanded(target.endpoint_path, resizer_.staState());
  const int path_index = target.path_index;
  if (path_index < static_cast<int>(expanded.startIndex())
      || path_index >= static_cast<int>(expanded.size())) {
    return false;
  }
  const sta::Path* driver_path = expanded.path(path_index);
  const sta::Pin* path_driver_pin = driver_path != nullptr
                                        ? driver_path->pin(resizer_.staState())
                                        : nullptr;
  const bool center_on_path = (drvr_pin == path_driver_pin);

  // Center stage arc: the path arc when the center IS the endpoint-path
  // driver; the graph-worst gate arc for any other center (e.g. size down
  // fanout centers on the downsized fanout gate).
  const sta::TimingArc* path_arc = nullptr;
  const sta::TimingArc* ref_arc = nullptr;
  float center_in_slew = 0.0f;
  if (center_on_path) {
    path_arc = driver_path->prevArc(resizer_.staState());
    if (path_arc == nullptr || path_arc->fromEdge() == nullptr
        || path_arc->toEdge() == nullptr) {
      return false;
    }
    const sta::LibertyPort* output_port
        = resizer_.network()->libertyPort(drvr_pin);
    if (output_port == nullptr) {
      return false;
    }
    ref_arc = findCandidateArc(
        path_arc, output_port->libertyCell(), scene_, min_max_);
    if (ref_arc == nullptr) {
      return false;
    }
  } else {
    sta::Vertex* center_vertex = resizer_.graph()->pinDrvrVertex(drvr_pin);
    if (center_vertex == nullptr
        || !worstGraphArcInto(center_vertex, ref_arc, center_in_slew)) {
      return false;
    }
  }

  // On-path load: the pin the endpoint path exits to (the next stage's
  // input pin on the path); it may sit on any region stage's frontier.
  on_path_load_pin_ = nullptr;
  if (path_index + 1 < static_cast<int>(expanded.size())) {
    const sta::Path* next_path = expanded.path(path_index + 1);
    if (next_path != nullptr) {
      on_path_load_pin_ = next_path->pin(resizer_.staState());
    }
  }

  // Center stage: input slew from the graph at the arc's input pin.  The
  // fanin stage feeding that pin is the one whose slew/arrival changes
  // couple through the center (arc_input_pin below).
  const sta::Pin* arc_input_pin
      = resizer_.network()->findPin(inst, ref_arc->from()->name());
  if (center_on_path) {
    const sta::RiseFall* in_rf = ref_arc->fromEdge()->asRiseFall();
    if (arc_input_pin != nullptr && in_rf != nullptr) {
      sta::Vertex* input_vertex
          = resizer_.graph()->pinDrvrVertex(arc_input_pin);
      if (input_vertex != nullptr) {
        center_in_slew = resizer_.sta()->graphDelayCalc()->edgeFromSlew(
            input_vertex, in_rf, ref_arc->role(), scene_, min_max_);
      }
    }
  }
  if (!buildStage(drvr_pin, ref_arc, center_in_slew, target_stage_)) {
    return false;
  }
  collectFrontier(target_stage_, nullptr, on_path_load_pin_);

  // Debug cross-check: the OLD-model gate delay should track the graph's
  // annotated delay for the same arc; a large gap flags a broken snapshot
  // (wrong slew, wrong parasitic, wrong arc).  The veto itself uses only
  // OLD-vs-NEW deltas, so this never enters the decision.
  if (center_on_path) {
    sta::Edge* gate_edge = driver_path->prevEdge(resizer_.staState());
    if (gate_edge != nullptr) {
      target_stage_.graph_gate_delay
          = sta::delayAsFloat(resizer_.graph()->arcDelay(
              gate_edge, path_arc, scene_->dcalcAnalysisPtIndex(min_max_)));
      debugPrint(resizer_.logger(),
                 utl::RSZ,
                 "neighbor_subgraph",
                 3,
                 "target {} old-model gate {:.3e} vs graph {:.3e}",
                 resizer_.network()->pathName(drvr_pin),
                 target_stage_.old_gate_delay,
                 target_stage_.graph_gate_delay);
    }
  }

  // One fanin driver stage per connected target input pin.
  std::unique_ptr<sta::InstancePinIterator> pin_iter(
      resizer_.network()->pinIterator(inst));
  size_t frontier_total = target_stage_.frontier.size();
  while (pin_iter->hasNext()) {
    const sta::Pin* input_pin = pin_iter->next();
    if (!resizer_.network()->direction(input_pin)->isInput()) {
      continue;
    }
    if (resizer_.sta()->isClock(input_pin, scene_->mode())) {
      continue;  // clock networks are not fair game for slack trades
    }
    sta::Vertex* load_vertex
        = resizer_.graph()->pinLoadVertex(const_cast<sta::Pin*>(input_pin));
    if (load_vertex == nullptr) {
      continue;
    }
    // The fanin driver is the wire edge's from-vertex.
    sta::Vertex* fanin_drvr = nullptr;
    sta::VertexInEdgeIterator in_iter(load_vertex, resizer_.graph());
    while (in_iter.hasNext()) {
      sta::Edge* edge = in_iter.next();
      if (edge->isWire()) {
        fanin_drvr = edge->from(resizer_.graph());
        break;
      }
    }
    if (fanin_drvr == nullptr || fanin_drvr->pin() == nullptr) {
      continue;
    }
    sta::Instance* fanin_inst = resizer_.network()->instance(fanin_drvr->pin());
    if (fanin_inst == nullptr || fanin_inst == inst) {
      continue;  // self-loop: not representable as an independent stage
    }
    const sta::TimingArc* fanin_arc = nullptr;
    float fanin_in_slew = 0.0f;
    if (!worstGraphArcInto(fanin_drvr, fanin_arc, fanin_in_slew)) {
      continue;  // e.g. top-level port driver: fixed drive, nothing to slow
    }
    SubgraphStage stage;
    if (!buildStage(fanin_drvr->pin(), fanin_arc, fanin_in_slew, stage)) {
      continue;
    }
    collectFrontier(stage, input_pin, on_path_load_pin_);
    stage.feeds_on_path_input = (input_pin == arc_input_pin);
    if (!stage.has_target_input_index) {
      continue;  // the target input pin must be one of this stage's loads
    }
    frontier_total += stage.frontier.size();
    fanin_stages_.push_back(std::move(stage));
    if (static_cast<int>(fanin_stages_.size()) > kMaxFaninStages
        || frontier_total > kMaxFrontierLoads) {
      return false;  // oversized region: permissive skip
    }
  }

  valid_ = true;
  return true;
}

float SubgraphTimer::receiverSlewToDelay(const SubgraphLoad& load,
                                         const float old_slew,
                                         const float new_slew) const
{
  if (load.rcv_arc == nullptr || old_slew == new_slew) {
    return 0.0f;
  }
  float old_delay = 0.0f;
  float new_delay = 0.0f;
  float unused_slew = 0.0f;
  if (!tableDelay(load.rcv_pvt,
                  load.rcv_arc,
                  old_slew,
                  load.rcv_load_cap,
                  old_delay,
                  unused_slew)
      || !tableDelay(load.rcv_pvt,
                     load.rcv_arc,
                     new_slew,
                     load.rcv_load_cap,
                     new_delay,
                     unused_slew)) {
    return 0.0f;
  }
  return new_delay - old_delay;
}

float SubgraphTimer::loadArrivalDelta(const SubgraphStage& stage,
                                      const SubgraphStageEval& eval,
                                      const SubgraphLoad& load) const
{
  const size_t i = load.load_index;
  float delta = eval.gate_delay - stage.old_gate_delay;
  if (i < eval.wire_delay.size() && i < stage.old_wire_delay.size()) {
    delta += eval.wire_delay[i] - stage.old_wire_delay[i];
    delta
        += receiverSlewToDelay(load, stage.old_load_slew[i], eval.load_slew[i]);
  }
  return delta;
}

void SubgraphTimer::appendFrontierSlacks(
    const SubgraphStage& stage,
    const SubgraphStageEval& eval,
    const float extra_delta,
    LocalSlack& on_path,
    bool& on_path_seen,
    std::vector<LocalSlack>& neighbors) const
{
  for (const SubgraphLoad& load : stage.frontier) {
    const float delta = extra_delta + loadArrivalDelta(stage, eval, load);
    const LocalSlack entry{load.slack_before, load.slack_before - delta};
    if (load.on_path) {
      on_path = entry;
      on_path_seen = true;
    } else {
      neighbors.push_back(entry);
    }
  }
}

bool SubgraphTimer::faninCapDeltas(const SubgraphMove& move,
                                   std::vector<float>& delta_by_stage) const
{
  delta_by_stage.assign(fanin_stages_.size(), 0.0f);
  for (size_t s = 0; s < fanin_stages_.size(); ++s) {
    const SubgraphStage& stage = fanin_stages_[s];
    const auto explicit_delta = move.load_cap_delta.find(stage.drvr_pin);
    if (explicit_delta != move.load_cap_delta.end()) {
      delta_by_stage[s] += explicit_delta->second;
    }
    // Input-pin cap change when the instance this stage feeds is
    // substituted (the coupling pin's port on old vs new cell).
    const sta::Pin* input_pin = stage.target_input_pin;
    const sta::Instance* fed_inst
        = input_pin != nullptr ? resizer_.network()->instance(input_pin)
                               : nullptr;
    const auto sub = fed_inst != nullptr ? move.cell_subs.find(fed_inst)
                                         : move.cell_subs.end();
    if (sub == move.cell_subs.end()) {
      continue;
    }
    sta::LibertyCell* scene_cell = const_cast<sta::LibertyCell*>(sub->second)
                                       ->sceneCell(scene_, min_max_);
    const sta::LibertyPort* cur_port
        = resizer_.network()->libertyPort(input_pin);
    if (scene_cell == nullptr || cur_port == nullptr) {
      return false;
    }
    const sta::LibertyPort* new_port
        = scene_cell->findLibertyPort(cur_port->name());
    if (new_port == nullptr) {
      continue;
    }
    delta_by_stage[s]
        += new_port->capacitance(min_max_) - cur_port->capacitance(min_max_);
  }
  return true;
}

bool SubgraphTimer::evaluate(const SubgraphMove& move,
                             LocalSlack& on_path,
                             std::vector<LocalSlack>& neighbors)
{
  neighbors.clear();
  if (!valid_) {
    return false;
  }

  // Center arc under the move: substituted cell's matching arc, else the
  // snapshot arc.
  const sta::TimingArc* center_arc = target_stage_.arc;
  const auto center_sub = move.cell_subs.find(target_stage_.inst);
  if (center_sub != move.cell_subs.end()) {
    center_arc = findCandidateArc(
        target_stage_.arc, center_sub->second, scene_, min_max_);
    if (center_arc == nullptr) {
      return false;
    }
  }

  std::vector<float> delta_by_stage;
  if (!faninCapDeltas(move, delta_by_stage)) {
    return false;
  }

  bool on_path_seen = false;
  float coupling = 0.0f;  // arrival delta into the center's arc input
  float center_in_slew = target_stage_.input_slew;

  // Fanin stages whose load changes: re-evaluate; sibling loads are
  // frontier observations, and the stage feeding the center's arc input
  // couples its arrival/slew change into the center stage.
  for (size_t i = 0; i < fanin_stages_.size(); ++i) {
    const SubgraphStage& stage = fanin_stages_[i];
    if (delta_by_stage[i] == 0.0f) {
      continue;
    }
    SubgraphStageEval eval;
    if (!evalStage(stage,
                   stage.arc,
                   stage.input_slew,
                   stage.load_cap + delta_by_stage[i],
                   eval)) {
      return false;
    }
    appendFrontierSlacks(stage, eval, 0.0f, on_path, on_path_seen, neighbors);
    if (stage.feeds_on_path_input && stage.has_target_input_index) {
      const size_t idx = stage.target_input_load_index;
      coupling = eval.gate_delay - stage.old_gate_delay;
      if (idx < eval.wire_delay.size()) {
        coupling += eval.wire_delay[idx] - stage.old_wire_delay[idx];
        // Receiver-slew ratio: preserve the net's degradation ratio when
        // scaling the center's input slew (estimateReceiverInputSlew).
        const float denom
            = std::max(stage.old_drvr_slew, kMinSlewRatioDenominator);
        center_in_slew = target_stage_.input_slew * (eval.drvr_slew / denom);
      }
    }
  }

  // Center stage with the substituted arc, coupled input slew, and any
  // explicit load-cap delta (split/clone move loads off the center net).
  float center_load_cap = target_stage_.load_cap;
  const auto center_delta = move.load_cap_delta.find(target_stage_.drvr_pin);
  if (center_delta != move.load_cap_delta.end()) {
    center_load_cap = std::max(0.0f, center_load_cap + center_delta->second);
  }
  SubgraphStageEval center_eval;
  if (!evalStage(target_stage_,
                 center_arc,
                 center_in_slew,
                 center_load_cap,
                 center_eval)) {
    return false;
  }
  const float center_gate_delta
      = center_eval.gate_delay - target_stage_.old_gate_delay;

  // Virtual appendage stages (table-only: their nets do not exist yet).
  // Precompute each stage's delay/output slew at its moved cap.
  struct VirtualEval
  {
    const SubgraphMove::VirtualStage* spec;
    float delay;
    float out_slew;
  };
  std::vector<VirtualEval> virtuals;
  for (const SubgraphMove::VirtualStage& vs : move.virtual_stages) {
    sta::LibertyCell* v_cell
        = const_cast<sta::LibertyCell*>(vs.cell)->sceneCell(scene_, min_max_);
    if (v_cell == nullptr) {
      return false;
    }
    float moved_cap = 0.0f;
    for (const sta::Pin* pin : vs.moved_loads) {
      const sta::LibertyPort* port = resizer_.network()->libertyPort(pin);
      if (port != nullptr) {
        moved_cap += port->capacitance(min_max_);
      }
    }
    const sta::TimingArc* v_arc = nullptr;
    if (v_cell->isBuffer()) {
      sta::LibertyPort* v_in = nullptr;
      sta::LibertyPort* v_out = nullptr;
      v_cell->bufferPorts(v_in, v_out);
      if (v_in == nullptr || v_out == nullptr) {
        return false;
      }
      for (const sta::TimingArcSet* arc_set :
           v_cell->timingArcSets(v_in, v_out)) {
        if (arc_set->role()->isTimingCheck() || arc_set->arcs().empty()) {
          continue;
        }
        v_arc = arc_set->arcs().front();
        break;
      }
    } else {
      v_arc = findCandidateArc(target_stage_.arc, vs.cell, scene_, min_max_);
    }
    if (v_arc == nullptr) {
      return false;
    }
    // Driven through the center (split buffer): sees the center's NEW
    // output slew.  Replacing the center for its loads (clone): sees the
    // center's coupled input slew.
    const float v_in_slew
        = vs.replaces_center ? center_in_slew : center_eval.drvr_slew;
    float v_delay = 0.0f;
    float v_out_slew = 0.0f;
    if (!tableDelay(target_stage_.pvt,
                    v_arc,
                    v_in_slew,
                    moved_cap,
                    v_delay,
                    v_out_slew)) {
      return false;
    }
    virtuals.push_back({&vs, v_delay, v_out_slew});
  }

  // Center frontier: moved loads go through their virtual stage; the rest
  // see the re-evaluated center.
  for (const SubgraphLoad& load : target_stage_.frontier) {
    const size_t i = load.load_index;
    const VirtualEval* via_virtual = nullptr;
    for (const VirtualEval& ve : virtuals) {
      if (ve.spec->moved_loads.count(load.pin) > 0) {
        via_virtual = &ve;
        break;
      }
    }
    float delta = 0.0f;
    if (via_virtual != nullptr) {
      const float old_slew = i < target_stage_.old_load_slew.size()
                                 ? target_stage_.old_load_slew[i]
                                 : target_stage_.old_drvr_slew;
      if (via_virtual->spec->replaces_center) {
        // Clone: the virtual gate replaces the center gate for this load;
        // wire delay retained (the clone lands at the moved-load centroid).
        delta = coupling + (via_virtual->delay - target_stage_.old_gate_delay)
                + receiverSlewToDelay(load, old_slew, via_virtual->out_slew);
      } else {
        // Split buffer: center relief, then the buffer stage; wire delay
        // retained (the buffer lands near the driver).
        delta = center_gate_delta + via_virtual->delay
                + receiverSlewToDelay(load, old_slew, via_virtual->out_slew);
      }
    } else {
      delta = coupling + loadArrivalDelta(target_stage_, center_eval, load);
    }
    const LocalSlack entry{load.slack_before, load.slack_before - delta};
    if (load.on_path) {
      on_path = entry;
      on_path_seen = true;
    } else {
      neighbors.push_back(entry);
    }
  }

  if (!on_path_seen) {
    // Endpoint-side load unobservable in the region: fall back to the
    // target's own slack with the gate-level delta (no wire term).
    const float delta = coupling + center_gate_delta;
    on_path = LocalSlack{target_slack_, target_slack_ - delta};
  }
  return true;
}

bool SubgraphTimer::evaluateCellSwap(const sta::LibertyCell* candidate,
                                     LocalSlack& on_path,
                                     std::vector<LocalSlack>& neighbors)
{
  if (!valid_ || candidate == nullptr) {
    return false;
  }
  SubgraphMove move;
  move.cell_subs[target_stage_.inst] = candidate;
  return evaluate(move, on_path, neighbors);
}

bool SubgraphTimer::evaluatePinSwap(const sta::LibertyPort* input_port,
                                    const sta::LibertyPort* swap_port,
                                    LocalSlack& on_path,
                                    std::vector<LocalSlack>& neighbors)
{
  neighbors.clear();
  if (!valid_ || input_port == nullptr || swap_port == nullptr) {
    return false;
  }
  const sta::Pin* input_pin
      = resizer_.network()->findPin(target_stage_.inst, input_port);
  const sta::Pin* swap_pin
      = resizer_.network()->findPin(target_stage_.inst, swap_port);
  if (input_pin == nullptr || swap_pin == nullptr) {
    return false;
  }

  // The arc the critical signal will use after the swap: same output
  // transition, from swap_port.
  sta::LibertyCell* cell = target_stage_.arc->set()->libertyCell();
  const sta::TimingArc* swap_arc = nullptr;
  {
    sta::LibertyPort* from = cell->findLibertyPort(swap_port->name());
    sta::LibertyPort* to
        = cell->findLibertyPort(target_stage_.arc->to()->name());
    if (from == nullptr || to == nullptr) {
      return false;
    }
    for (const sta::TimingArcSet* arc_set : cell->timingArcSets(from, to)) {
      if (arc_set->role()->isTimingCheck()) {
        continue;
      }
      for (const sta::TimingArc* arc : arc_set->arcs()) {
        if (arc->toEdge() == target_stage_.arc->toEdge()) {
          swap_arc = arc;
          break;
        }
      }
      if (swap_arc != nullptr) {
        break;
      }
    }
  }
  if (swap_arc == nullptr) {
    return false;
  }

  const float cap_input = input_port->capacitance(min_max_);
  const float cap_swap = swap_port->capacitance(min_max_);

  bool on_path_seen = false;
  float on_path_input_delta = 0.0f;
  float target_in_slew = target_stage_.input_slew;

  for (SubgraphStage& stage : fanin_stages_) {
    float delta_cap = 0.0f;
    float through_delta = 0.0f;
    if (stage.target_input_pin == input_pin) {
      // Critical net moves onto swap_port.
      delta_cap = cap_swap - cap_input;
    } else if (stage.target_input_pin == swap_pin) {
      // Other net moves onto input_port; it also changes its through-arc,
      // charged at this stage's coupling point below.
      delta_cap = cap_input - cap_swap;
    } else {
      continue;
    }
    SubgraphStageEval eval;
    if (!evalStage(stage,
                   stage.arc,
                   stage.input_slew,
                   stage.load_cap + delta_cap,
                   eval)) {
      return false;
    }
    appendFrontierSlacks(stage, eval, 0.0f, on_path, on_path_seen, neighbors);

    const size_t idx = stage.target_input_load_index;
    float coupling = eval.gate_delay - stage.old_gate_delay;
    if (idx < eval.wire_delay.size()) {
      coupling += eval.wire_delay[idx] - stage.old_wire_delay[idx];
    }
    if (stage.target_input_pin == input_pin) {
      on_path_input_delta = coupling;
      const float denom
          = std::max(stage.old_drvr_slew, kMinSlewRatioDenominator);
      target_in_slew = target_stage_.input_slew * (eval.drvr_slew / denom);
    } else {
      // The other net's own path continues through the target gate on a
      // different arc after the swap; charge the driver slowdown plus the
      // through-arc table delta at the swapped input pin's vertex.
      float old_through = 0.0f;
      float new_through = 0.0f;
      float unused = 0.0f;
      const sta::TimingArc* other_old_arc = swap_arc;  // via swap_port today
      const sta::TimingArc* other_new_arc = target_stage_.arc;
      float slew_at_pin = stage.old_load_slew.size() > idx
                              ? stage.old_load_slew[idx]
                              : stage.old_drvr_slew;
      if (tableDelay(target_stage_.pvt,
                     other_old_arc,
                     slew_at_pin,
                     target_stage_.load_cap,
                     old_through,
                     unused)
          && tableDelay(target_stage_.pvt,
                        other_new_arc,
                        slew_at_pin,
                        target_stage_.load_cap,
                        new_through,
                        unused)) {
        through_delta = new_through - old_through;
      }
      sta::Vertex* swap_vertex = resizer_.graph()->pinLoadVertex(
          const_cast<sta::Pin*>(stage.target_input_pin));
      float slack_before = 0.0f;
      if (cachedSlack(swap_vertex, slack_before)) {
        neighbors.push_back(
            {slack_before, slack_before - (coupling + through_delta)});
      }
    }
  }

  SubgraphStageEval target_eval;
  if (!evalStage(target_stage_,
                 swap_arc,
                 target_in_slew,
                 target_stage_.load_cap,
                 target_eval)) {
    return false;
  }
  appendFrontierSlacks(target_stage_,
                       target_eval,
                       on_path_input_delta,
                       on_path,
                       on_path_seen,
                       neighbors);
  if (!on_path_seen) {
    const float delta
        = on_path_input_delta
          + (target_eval.gate_delay - target_stage_.old_gate_delay);
    on_path = LocalSlack{target_slack_, target_slack_ - delta};
  }
  return true;
}

bool SubgraphTimer::evaluateSplitLoad(const sta::LibertyCell* buffer_cell,
                                      const sta::PinSet& moved_loads,
                                      LocalSlack& on_path,
                                      std::vector<LocalSlack>& neighbors)
{
  if (!valid_ || buffer_cell == nullptr) {
    return false;
  }
  sta::LibertyCell* buf_cell
      = const_cast<sta::LibertyCell*>(buffer_cell)->sceneCell(scene_, min_max_);
  if (buf_cell == nullptr) {
    return false;
  }
  sta::LibertyPort* buf_in = nullptr;
  sta::LibertyPort* buf_out = nullptr;
  buf_cell->bufferPorts(buf_in, buf_out);
  if (buf_in == nullptr) {
    return false;
  }
  float moved_cap = 0.0f;
  SubgraphMove move;
  SubgraphMove::VirtualStage buffer_stage;
  buffer_stage.cell = buffer_cell;
  for (const sta::Pin* pin : moved_loads) {
    buffer_stage.moved_loads.insert(pin);
    const sta::LibertyPort* port = resizer_.network()->libertyPort(pin);
    if (port != nullptr) {
      moved_cap += port->capacitance(min_max_);
    }
  }
  move.load_cap_delta[target_stage_.drvr_pin]
      = buf_in->capacitance(min_max_) - moved_cap;
  move.virtual_stages.push_back(std::move(buffer_stage));
  return evaluate(move, on_path, neighbors);
}

bool SubgraphTimer::evaluateClone(const sta::LibertyCell* clone_cell,
                                  const std::vector<sta::Pin*>& moved_loads,
                                  LocalSlack& on_path,
                                  std::vector<LocalSlack>& neighbors)
{
  if (!valid_ || clone_cell == nullptr) {
    return false;
  }
  sta::LibertyCell* scene_cell
      = const_cast<sta::LibertyCell*>(clone_cell)->sceneCell(scene_, min_max_);
  if (scene_cell == nullptr) {
    return false;
  }
  SubgraphMove move;
  // Every fanin net gains the clone's FULL input cap (the clone is a new
  // gate on the net; the original keeps its pin).
  for (const SubgraphStage& stage : fanin_stages_) {
    const sta::LibertyPort* cur_port
        = resizer_.network()->libertyPort(stage.target_input_pin);
    const sta::LibertyPort* clone_port
        = cur_port != nullptr ? scene_cell->findLibertyPort(cur_port->name())
                              : nullptr;
    if (clone_port != nullptr) {
      move.load_cap_delta[stage.drvr_pin] += clone_port->capacitance(min_max_);
    }
  }
  SubgraphMove::VirtualStage clone_stage;
  clone_stage.cell = clone_cell;
  clone_stage.replaces_center = true;
  float moved_cap = 0.0f;
  for (sta::Pin* pin : moved_loads) {
    clone_stage.moved_loads.insert(pin);
    const sta::LibertyPort* port = resizer_.network()->libertyPort(pin);
    if (port != nullptr) {
      moved_cap += port->capacitance(min_max_);
    }
  }
  move.load_cap_delta[target_stage_.drvr_pin] += -moved_cap;
  move.virtual_stages.push_back(std::move(clone_stage));
  return evaluate(move, on_path, neighbors);
}

}  // namespace rsz
