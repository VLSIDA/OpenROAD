// SPDX-License-Identifier: BSD-3-Clause

#include "cms/ClockMesh.hh"

#include <algorithm>
#include <climits>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <sstream>

#include "cms/LoadAdaptiveBuffer.hh"
#include "cts/TritonCTS.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "odb/dbShape.h"
#include "odb/dbTransform.h"
#include "odb/dbWireCodec.h"
#include "odb/wOrder.h"
#include "ord/OpenRoad.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/MinMax.hh"
#include "sta/Sdc.hh"
#include "sta/Sta.hh"
#include "sta/Transition.hh"
#include "utl/Logger.h"

namespace cms {
using utl::CMS;
ClockMesh::ClockMesh() = default;

// Initializes ClockMesh with OpenROAD database and STA access
void ClockMesh::init(ord::OpenRoad* openroad)
{
  openroad_ = openroad;

  if (openroad_ == nullptr) {
    return;
  }
  logger_ = openroad_->getLogger();
  db_ = openroad_->getDb();
  sta_ = openroad_->getSta();
  network_ = sta_->getDbNetwork();

  if (db_) {
    odb::dbChip* chip = db_->getChip();
    if (chip) {
      block_ = chip->getBlock();
    }
  }
}

// Finds all clock sinks from SDC-defined clocks and populates clockToSinks_
void ClockMesh::findClockSinks()
{
  if (db_) {
    odb::dbChip* chip = db_->getChip();
    if (chip) {
      block_ = chip->getBlock();
    }
  }
  if (!block_) {
    logger_->error(CMS, 1, "No block found in database");
    return;
  }
  if (!sta_ || !network_) {
    logger_->error(CMS, 2, "STA not initialized");
    return;
  }

  clockToSinks_.clear();
  visitedClockNets_.clear();
  sta::Sdc* sdc = sta_->cmdSdc();

  if (!sdc) {
    logger_->warn(CMS, 4, "No SDC constraints found");
    return;
  }

  for (auto clk : sdc->clocks()) {
    std::string clkName = clk->name();
    std::set<odb::dbNet*, odb::ODBPtrLess> clkNets;
    findClockRoots(clk, clkNets);
    for (odb::dbNet* net : clkNets) {
      if (net && visitedClockNets_.find(net) == visitedClockNets_.end()) {
        visitedClockNets_.insert(net);
        std::vector<ClockSink> sinks;
        if (separateSinks(net, sinks)) {
          clockToSinks_[clkName].insert(
              clockToSinks_[clkName].end(), sinks.begin(), sinks.end());
        }
      }
    }
  }
}

// Extracts the nets connected to clock leaf pins
void ClockMesh::findClockRoots(
    sta::Clock* clk,
    std::set<odb::dbNet*, odb::ODBPtrLess>& clockNets)
{
  for (const sta::Pin* pin : clk->leafPins()) {
    odb::dbITerm* instTerm;
    odb::dbBTerm* port;
    odb::dbModITerm* moditerm;
    network_->staToDb(pin, instTerm, port, moditerm);
    odb::dbNet* net
        = instTerm ? instTerm->getNet() : (port ? port->getNet() : nullptr);
    if (net) {
      clockNets.insert(net);
    }
  }
}

// Checks if an ITerm is a clock sink (register clock pin or macro)
bool ClockMesh::isSink(odb::dbITerm* iterm)
{
  odb::dbInst* inst = iterm->getInst();
  // Macros (block-class instances) are NOT mesh sinks — CTS already drives
  // their clock pins via the original clock net. The mesh is for low-skew
  // distribution to many tiny std-cell flop loads; macro clock pins have
  // their own (much larger) cap and timing characteristics that CTS handles
  // properly. Letting them onto the mesh pollutes load-adaptive sizing and
  // duplicates the clock drive. Check this BEFORE the Liberty early-return
  // below, because LEF-only macros (e.g. fakeram*) have no Liberty cell and
  // would otherwise fall through to the "treat as sink" fallback.
  if (inst->isBlock()) {
    return false;
  }
  sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
  sta::LibertyCell* libertyCell = network_->libertyCell(masterCell);
  if (!libertyCell) {
    return true;
  }
  sta::LibertyPort* inputPort
      = libertyCell->findLibertyPort(iterm->getMTerm()->getConstName());

  if (inputPort) {
    return inputPort->isRegClk();
  }

  return false;
}

// Computes the center position of an ITerm from its shapes
void ClockMesh::computeITermPosition(odb::dbITerm* term, int& x, int& y) const
{
  odb::dbITermShapeItr itr;
  odb::dbShape shape;
  x = 0;
  y = 0;
  unsigned numShapes = 0;

  for (itr.begin(term); itr.next(shape);) {
    if (!shape.isVia()) {
      x += shape.xMin() + (shape.xMax() - shape.xMin()) / 2;
      y += shape.yMin() + (shape.yMax() - shape.yMin()) / 2;
      ++numShapes;
    }
  }

  if (numShapes > 0) {
    x /= numShapes;
    y /= numShapes;
  }
}

// Collects clock sinks from a net into the sinks vector
bool ClockMesh::separateSinks(odb::dbNet* net, std::vector<ClockSink>& sinks)
{
  if (!net) {
    return false;
  }

  for (odb::dbITerm* iterm : net->getITerms()) {
    odb::dbInst* inst = iterm->getInst();
    if (iterm->isInputSignal() && inst->isPlaced()) {
      odb::dbMTerm* mterm = iterm->getMTerm();
      if (isSink(iterm)) {
        std::string name = std::string(inst->getConstName()) + "/"
                           + std::string(mterm->getConstName());
        int x, y;
        computeITermPosition(iterm, x, y);
        bool isMacro = inst->isBlock();
        sinks.emplace_back(name, x, y, iterm, isMacro);
      }
    }
  }

  return !sinks.empty();
}

// Calculates bounding box enclosing all sinks, expanded to core area
odb::Rect ClockMesh::calculateBoundingBox(const std::vector<ClockSink>& sinks)
{
  if (sinks.empty()) {
    logger_->warn(CMS, 100, "No sinks provided for bounding box calculation");
    return odb::Rect(0, 0, 0, 0);
  }

  int min_x = sinks[0].x;
  int max_x = sinks[0].x;
  int min_y = sinks[0].y;
  int max_y = sinks[0].y;

  for (const auto& sink : sinks) {
    min_x = std::min(min_x, sink.x);
    max_x = std::max(max_x, sink.x);
    min_y = std::min(min_y, sink.y);
    max_y = std::max(max_y, sink.y);
  }

  odb::Rect bbox(min_x, min_y, max_x, max_y);

  if (block_) {
    odb::Rect core_area = block_->getCoreArea();
    bbox.set_xlo(std::min(bbox.xMin(), core_area.xMin()));
    bbox.set_ylo(std::min(bbox.yMin(), core_area.yMin()));
    bbox.set_xhi(std::max(bbox.xMax(), core_area.xMax()));
    bbox.set_yhi(std::max(bbox.yMax(), core_area.yMax()));
  }

  return bbox;
}

// Collects macro and blockage bounding boxes, bloated by halo_dbu, into
// blockage_rects_
void ClockMesh::collectBlockageRects(int halo_dbu)
{
  blockage_rects_.clear();
  if (!block_) {
    return;
  }

  for (odb::dbInst* inst : block_->getInsts()) {
    if (!inst->isBlock()) {
      continue;
    }
    odb::dbBox* bbox = inst->getBBox();
    if (!bbox) {
      continue;
    }
    odb::Rect r = bbox->getBox();
    if (halo_dbu > 0) {
      r.bloat(halo_dbu, r);
    }
    blockage_rects_.push_back(r);
  }

  for (odb::dbBlockage* blk : block_->getBlockages()) {
    odb::dbBox* bbox = blk->getBBox();
    if (!bbox) {
      continue;
    }
    odb::Rect r = bbox->getBox();
    if (halo_dbu > 0) {
      r.bloat(halo_dbu, r);
    }
    blockage_rects_.push_back(r);
  }

  logger_->info(CMS,
                130,
                "Collected {} blockage rects (halo = {} DBU)",
                blockage_rects_.size(),
                halo_dbu);
}

// True if (x, y) falls inside any blockage rect
bool ClockMesh::isBlocked(int x, int y) const
{
  odb::Point p(x, y);
  for (const odb::Rect& r : blockage_rects_) {
    if (r.intersects(p)) {
      return true;
    }
  }
  return false;
}

// True if r overlaps any blockage rect
bool ClockMesh::isBlocked(const odb::Rect& r) const
{
  for (const odb::Rect& blk : blockage_rects_) {
    if (blk.intersects(r)) {
      return true;
    }
  }
  return false;
}

// Clips a wire rect against blockage_rects_ along its long axis.
// Returns 0..N surviving subsegments; segments shorter than min_segment_length
// dropped.
std::vector<odb::Rect> ClockMesh::clipWireByBlockages(
    const odb::Rect& wire_rect,
    bool is_horizontal,
    int min_segment_length) const
{
  std::vector<std::pair<int, int>> blocked;
  for (const odb::Rect& blk : blockage_rects_) {
    if (!blk.intersects(wire_rect)) {
      continue;
    }
    int lo, hi;
    if (is_horizontal) {
      lo = std::max(blk.xMin(), wire_rect.xMin());
      hi = std::min(blk.xMax(), wire_rect.xMax());
    } else {
      lo = std::max(blk.yMin(), wire_rect.yMin());
      hi = std::min(blk.yMax(), wire_rect.yMax());
    }
    if (lo <= hi) {
      blocked.emplace_back(lo, hi);
    }
  }

  if (blocked.empty()) {
    return {wire_rect};
  }

  std::sort(blocked.begin(), blocked.end());
  std::vector<std::pair<int, int>> merged;
  for (const auto& iv : blocked) {
    if (!merged.empty() && iv.first <= merged.back().second) {
      merged.back().second = std::max(merged.back().second, iv.second);
    } else {
      merged.push_back(iv);
    }
  }

  std::vector<odb::Rect> result;
  int cursor = is_horizontal ? wire_rect.xMin() : wire_rect.yMin();
  int end = is_horizontal ? wire_rect.xMax() : wire_rect.yMax();

  auto emit = [&](int seg_start, int seg_end) {
    if (seg_end - seg_start < min_segment_length) {
      return;
    }
    if (is_horizontal) {
      result.emplace_back(
          seg_start, wire_rect.yMin(), seg_end, wire_rect.yMax());
    } else {
      result.emplace_back(
          wire_rect.xMin(), seg_start, wire_rect.xMax(), seg_end);
    }
  };

  for (const auto& [lo, hi] : merged) {
    if (cursor < lo) {
      emit(cursor, lo - 1);
    }
    cursor = std::max(cursor, hi + 1);
  }
  if (cursor <= end) {
    emit(cursor, end);
  }
  return result;
}

// Collects merged x-intervals of VERTICAL power straps (taller than wide) on
// the power/ground special wires (BM3 in gt2n). The via columns under these
// straps drop pads onto the clock mesh layers (BM1/BM2) and would short the
// mesh; vertical clock wires shift clear of these bands and horizontal clock
// wires are notched at them.
void ClockMesh::collectPdnVStraps()
{
  pdn_vstrap_x_.clear();
  if (!block_ || !mesh_v_layer_) {
    return;
  }
  std::vector<std::pair<int, int>> iv;
  for (odb::dbNet* net : block_->getNets()) {
    const odb::dbSigType st = net->getSigType();
    if (st != odb::dbSigType::POWER && st != odb::dbSigType::GROUND) {
      continue;
    }
    for (odb::dbSWire* swire : net->getSWires()) {
      for (odb::dbSBox* sbox : swire->getWires()) {
        // Same-layer only (mirror of collectPdnHStraps): the clock mesh only
        // needs to dodge power straps ON ITS OWN vertical layer. A frontside
        // M5 mesh must NOT be shifted to avoid the backside BM1 straps -- they
        // are on different layers/sides and never conflict. Without this filter
        // the frontside grid gets spuriously deformed.
        if (sbox->isVia() || sbox->getTechLayer() != mesh_v_layer_) {
          continue;
        }
        const odb::Rect r = sbox->getBox();
        if (r.dy() > r.dx()) {  // vertical strap
          iv.emplace_back(r.xMin(), r.xMax());
        }
      }
    }
  }
  if (iv.empty()) {
    return;
  }
  std::sort(iv.begin(), iv.end());
  for (const auto& b : iv) {
    if (!pdn_vstrap_x_.empty() && b.first <= pdn_vstrap_x_.back().second) {
      pdn_vstrap_x_.back().second
          = std::max(pdn_vstrap_x_.back().second, b.second);
    } else {
      pdn_vstrap_x_.push_back(b);
    }
  }
  const double dbu = block_->getDbUnitsPerMicron();
  std::ostringstream gaps;
  for (size_t i = 1; i < pdn_vstrap_x_.size(); ++i) {
    gaps << ' ' << (pdn_vstrap_x_[i].first - pdn_vstrap_x_[i - 1].second) / dbu;
  }
  logger_->info(CMS,
                140,
                "PDN vertical straps: {} bands, first at x={:.3f}um; "
                "inter-strap gaps (um):{}",
                pdn_vstrap_x_.size(),
                pdn_vstrap_x_.front().first / dbu,
                gaps.str());
}

// Returns an x-center for a vertical clock wire that clears every PDN strap by
// >= spacing, shifted toward the roomier side. Returns INT_MIN if no clear
// position exists in the local gap (caller drops the wire).
int ClockMesh::shiftVClearOfPdn(int x_center, int half_w, int spacing) const
{
  const int lo = x_center - half_w - spacing;
  const int hi = x_center + half_w + spacing;
  int idx = -1;
  for (size_t i = 0; i < pdn_vstrap_x_.size(); ++i) {
    if (lo < pdn_vstrap_x_[i].second && pdn_vstrap_x_[i].first < hi) {
      idx = static_cast<int>(i);
      break;
    }
  }
  if (idx < 0) {
    return x_center;  // no conflict
  }
  const auto& band = pdn_vstrap_x_[idx];
  const int prev_hi = (idx > 0) ? pdn_vstrap_x_[idx - 1].second : INT_MIN / 2;
  const int next_lo = (idx + 1 < static_cast<int>(pdn_vstrap_x_.size()))
                          ? pdn_vstrap_x_[idx + 1].first
                          : INT_MAX / 2;
  int left_x = band.first - spacing - half_w;
  int right_x = band.second + spacing + half_w;
  // Keep shifted wires ON the layer's routing tracks: snap outward (away from
  // the band) to the nearest track so clearance is preserved.
  if (block_ && mesh_v_layer_) {
    if (odb::dbTrackGrid* tg = block_->findTrackGrid(mesh_v_layer_)) {
      std::vector<int> xs;
      tg->getGridX(xs);
      auto lo = std::upper_bound(xs.begin(), xs.end(), left_x);
      if (lo != xs.begin()) {
        left_x = *std::prev(lo);
      }
      auto hi = std::lower_bound(xs.begin(), xs.end(), right_x);
      if (hi != xs.end()) {
        right_x = *hi;
      }
    }
  }
  const bool left_ok = (left_x - half_w - spacing) > prev_hi;
  const bool right_ok = (right_x + half_w + spacing) < next_lo;
  if (left_ok && right_ok) {
    const int left_gap = band.first - prev_hi;
    const int right_gap = next_lo - band.second;
    return (right_gap >= left_gap) ? right_x : left_x;
  }
  if (right_ok) {
    return right_x;
  }
  if (left_ok) {
    return left_x;
  }
  return INT_MIN;  // cannot place a vertical wire in this gap
}

// Horizontal power straps ON the mesh H layer (co-planar fine-layer PDN).
// Same-layer only: BPR followpins / other layers cross freely.
void ClockMesh::collectPdnHStraps()
{
  pdn_hstrap_y_.clear();
  if (!block_ || !mesh_h_layer_) {
    return;
  }
  std::vector<std::pair<int, int>> iv;
  for (odb::dbNet* net : block_->getNets()) {
    const odb::dbSigType st = net->getSigType();
    if (st != odb::dbSigType::POWER && st != odb::dbSigType::GROUND) {
      continue;
    }
    for (odb::dbSWire* swire : net->getSWires()) {
      for (odb::dbSBox* sbox : swire->getWires()) {
        if (sbox->isVia() || sbox->getTechLayer() != mesh_h_layer_) {
          continue;
        }
        const odb::Rect r = sbox->getBox();
        if (r.dx() > r.dy()) {  // horizontal strap
          iv.emplace_back(r.yMin(), r.yMax());
        }
      }
    }
  }
  if (iv.empty()) {
    return;
  }
  std::sort(iv.begin(), iv.end());
  for (const auto& b : iv) {
    if (!pdn_hstrap_y_.empty() && b.first <= pdn_hstrap_y_.back().second) {
      pdn_hstrap_y_.back().second
          = std::max(pdn_hstrap_y_.back().second, b.second);
    } else {
      pdn_hstrap_y_.push_back(b);
    }
  }
  const double dbu = block_->getDbUnitsPerMicron();
  logger_->info(CMS,
                152,
                "PDN horizontal straps on {}: {} bands, first at y={:.3f}um",
                mesh_h_layer_->getName(),
                pdn_hstrap_y_.size(),
                pdn_hstrap_y_.front().first / dbu);
}

// Mirror of shiftVClearOfPdn for horizontal clock wires vs the same-layer
// horizontal power straps: slide to the roomier side of the conflicting band,
// keeping clear of the neighbor bands; INT_MIN if neither side fits.
int ClockMesh::shiftHClearOfPdn(int y_center, int half_w, int spacing) const
{
  const int lo = y_center - half_w - spacing;
  const int hi = y_center + half_w + spacing;
  int idx = -1;
  for (size_t i = 0; i < pdn_hstrap_y_.size(); ++i) {
    if (lo < pdn_hstrap_y_[i].second && pdn_hstrap_y_[i].first < hi) {
      idx = static_cast<int>(i);
      break;
    }
  }
  if (idx < 0) {
    return y_center;  // no conflict
  }
  const auto& band = pdn_hstrap_y_[idx];
  const int prev_hi = (idx > 0) ? pdn_hstrap_y_[idx - 1].second : INT_MIN / 2;
  const int next_lo = (idx + 1 < static_cast<int>(pdn_hstrap_y_.size()))
                          ? pdn_hstrap_y_[idx + 1].first
                          : INT_MAX / 2;
  int below_y = band.first - spacing - half_w;
  int above_y = band.second + spacing + half_w;
  // Snap shifted wires outward onto the H layer's routing tracks.
  if (block_ && mesh_h_layer_) {
    if (odb::dbTrackGrid* tg = block_->findTrackGrid(mesh_h_layer_)) {
      std::vector<int> ys;
      tg->getGridY(ys);
      auto lo = std::upper_bound(ys.begin(), ys.end(), below_y);
      if (lo != ys.begin()) {
        below_y = *std::prev(lo);
      }
      auto hi = std::lower_bound(ys.begin(), ys.end(), above_y);
      if (hi != ys.end()) {
        above_y = *hi;
      }
    }
  }
  const bool below_ok = (below_y - half_w - spacing) > prev_hi;
  const bool above_ok = (above_y + half_w + spacing) < next_lo;
  if (below_ok && above_ok) {
    const int below_gap = band.first - prev_hi;
    const int above_gap = next_lo - band.second;
    return (above_gap >= below_gap) ? above_y : below_y;
  }
  if (above_ok) {
    return above_y;
  }
  if (below_ok) {
    return below_y;
  }
  return INT_MIN;  // cannot place a horizontal wire in this gap
}

// Notches a horizontal clock wire segment at every vertical PDN strap (removing
// [strap - spacing, strap + spacing]). Returns the surviving sub-segments.
std::vector<odb::Rect> ClockMesh::notchHByPdn(const odb::Rect& seg,
                                              int spacing,
                                              int min_segment_length) const
{
  if (pdn_vstrap_x_.empty()) {
    return {seg};
  }
  std::vector<std::pair<int, int>> rem;
  for (const auto& b : pdn_vstrap_x_) {
    const int lo = std::max(b.first - spacing, seg.xMin());
    const int hi = std::min(b.second + spacing, seg.xMax());
    if (lo <= hi) {
      rem.emplace_back(lo, hi);
    }
  }
  if (rem.empty()) {
    return {seg};
  }
  std::sort(rem.begin(), rem.end());
  std::vector<odb::Rect> out;
  int cur = seg.xMin();
  for (const auto& [lo, hi] : rem) {
    if (cur < lo && (lo - 1 - cur) >= min_segment_length) {
      out.emplace_back(cur, seg.yMin(), lo - 1, seg.yMax());
    }
    cur = std::max(cur, hi + 1);
  }
  if (cur <= seg.xMax() && (seg.xMax() - cur) >= min_segment_length) {
    out.emplace_back(cur, seg.yMin(), seg.xMax(), seg.yMax());
  }
  return out;
}

// Drops horizontal mesh segments that no vertical clock wire crosses. A region
// (gap between PDN straps) with no vertical wire can host no buffer/TSV, so its
// horizontal metal would be dangling — remove it.
void ClockMesh::pruneOrphanHSegments(std::vector<MeshWire>& h_wires,
                                     const std::vector<MeshWire>& v_wires) const
{
  const size_t before = h_wires.size();
  h_wires.erase(
      std::remove_if(h_wires.begin(),
                     h_wires.end(),
                     [&](const MeshWire& h) {
                       const int hy = (h.rect.yMin() + h.rect.yMax()) / 2;
                       for (const MeshWire& v : v_wires) {
                         const int vx = (v.rect.xMin() + v.rect.xMax()) / 2;
                         if (vx >= h.rect.xMin() && vx <= h.rect.xMax()
                             && hy >= v.rect.yMin() && hy <= v.rect.yMax()) {
                           return false;  // crossed by a vertical wire -> keep
                         }
                       }
                       return true;  // no vertical wire in this region -> drop
                     }),
      h_wires.end());
  const size_t dropped = before - h_wires.size();
  if (dropped > 0) {
    logger_->info(CMS,
                  142,
                  "Pruned {} orphan horizontal segments (no vertical clock "
                  "wire in that region)",
                  dropped);
  }
}

// Analytic twin of collectPdnVStraps(): build the merged vertical-strap x-band
// list from the PDN parameters (pitch/offset/width) instead of scanning a built
// PDN. Strap centers are core.xMin() + offset + k*pitch. Same sorted,
// non-overlapping format the scanner produces, so the deform helpers are
// agnostic to which one filled pdn_vstrap_x_.
void ClockMesh::collectPdnVStrapsFromParams(int strap_pitch,
                                            int strap_offset,
                                            int strap_width,
                                            const odb::Rect& core)
{
  pdn_vstrap_x_.clear();
  if (strap_pitch <= 0 || strap_width <= 0) {
    return;
  }
  const int half = strap_width / 2;
  for (long long c = static_cast<long long>(core.xMin()) + strap_offset;
       c <= core.xMax();
       c += strap_pitch) {
    const int lo = static_cast<int>(c) - half;
    const int hi = static_cast<int>(c) + half;
    if (hi < core.xMin() || lo > core.xMax()) {
      continue;
    }
    pdn_vstrap_x_.emplace_back(std::max(lo, core.xMin()),
                               std::min(hi, core.xMax()));
  }
  if (!pdn_vstrap_x_.empty() && block_) {
    const double dbu = block_->getDbUnitsPerMicron();
    logger_->info(CMS,
                  143,
                  "PDN vertical straps (analytic): {} bands, pitch={:.3f}um, "
                  "width={:.3f}um, first center x~{:.3f}um",
                  pdn_vstrap_x_.size(),
                  strap_pitch / dbu,
                  strap_width / dbu,
                  (pdn_vstrap_x_.front().first + half) / dbu);
  }
}

// Verification helper: compute the frozen grid and report fragment sizes.
void ClockMesh::reportFrozenGrid(odb::dbTechLayer* h_layer,
                                 odb::dbTechLayer* v_layer,
                                 int pitch,
                                 int v_strap_pitch,
                                 int v_strap_offset,
                                 int v_strap_width)
{
  if (db_) {
    if (odb::dbChip* chip = db_->getChip()) {
      block_ = chip->getBlock();
    }
  }
  if (!block_) {
    logger_->error(CMS, 145, "No block for frozen-grid report");
    return;
  }
  FrozenGrid g = computeFrozenGrid(
      h_layer, v_layer, pitch, v_strap_pitch, v_strap_offset, v_strap_width);
  std::vector<size_t> sizes;
  for (const auto& f : g.fragments) {
    sizes.push_back(f.size());
  }
  std::sort(sizes.begin(), sizes.end());
  std::ostringstream hist;
  for (size_t s : sizes) {
    hist << ' ' << s;
  }
  size_t singletons = 0;
  for (size_t s : sizes) {
    if (s == 1) {
      singletons++;
    }
  }
  logger_->info(CMS,
                146,
                "Fragment sizes (sorted):{}  [{} fragments, {} singletons]",
                hist.str(),
                g.fragments.size(),
                singletons);
}

// Phase 1: reserve TSV sites + layer-selective keepouts on the frozen grid.
void ClockMesh::reserveMeshTsvSites(odb::dbTechLayer* h_layer,
                                    odb::dbTechLayer* v_layer,
                                    int pitch,
                                    int v_strap_pitch,
                                    int v_strap_offset,
                                    int v_strap_width,
                                    const std::string& tsv_master,
                                    int keepout_w,
                                    int keepout_h,
                                    int target_spacing)
{
  if (db_) {
    if (odb::dbChip* chip = db_->getChip()) {
      block_ = chip->getBlock();
    }
  }
  if (!block_) {
    logger_->error(CMS, 147, "No block for TSV reserve");
    return;
  }
  odb::dbMaster* tsv = db_->findMaster(tsv_master.c_str());
  if (!tsv) {
    logger_->error(CMS, 148, "TSV master '{}' not found", tsv_master);
    return;
  }

  const FrozenGrid g = computeFrozenGrid(
      h_layer, v_layer, pitch, v_strap_pitch, v_strap_offset, v_strap_width);
  if (g.intersections.empty()) {
    logger_->warn(CMS, 149, "Frozen grid produced no intersections");
    return;
  }
  const odb::Rect core = block_->getCoreArea();
  odb::dbTech* tech = db_->getTech();

  // Row geometry (sorted unique origin-y + per-row orient + site width).
  std::vector<int> row_ys;
  std::map<int, odb::dbOrientType> row_orient;
  int site_w = tsv->getWidth();
  for (odb::dbRow* r : block_->getRows()) {
    const int ry = r->getOrigin().y();
    row_ys.push_back(ry);
    row_orient[ry] = r->getOrient();
    if (r->getSite()) {
      site_w = r->getSite()->getWidth();
    }
  }
  std::sort(row_ys.begin(), row_ys.end());
  row_ys.erase(std::unique(row_ys.begin(), row_ys.end()), row_ys.end());
  if (row_ys.empty() || site_w <= 0) {
    logger_->error(CMS, 151, "No rows / bad site width for TSV reserve");
    return;
  }
  const int row_h
      = (row_ys.size() >= 2) ? row_ys[1] - row_ys[0] : tsv->getHeight();

  // Y-pin x offset within the master (R0); cell origin = ix - y_pin_off so the
  // Y pad centers on the vertical line.
  int y_pin_off = tsv->getWidth() / 2;
  if (odb::dbMTerm* ymt = tsv->findMTerm("Y")) {
    const odb::Rect yb = ymt->getBBox();
    y_pin_off = (yb.xMin() + yb.xMax()) / 2;
  }

  const int kw2 = keepout_w / 2;
  const int kext = std::max(
      0, (keepout_h - row_h) / 2);  // extend past row to catch both rails
  int placed = 0, sites = 0;

  for (const std::vector<int>& frag : g.fragments) {
    // Driver selection: >=1 per fragment, greedily spaced by target_spacing.
    std::vector<int> sel;
    for (int idx : frag) {
      const GridIntersection& p = g.intersections[idx];
      bool ok = true;
      for (int s : sel) {
        const GridIntersection& q = g.intersections[s];
        const long long dx = p.x - q.x, dy = p.y - q.y;
        if (dx * dx + dy * dy
            < static_cast<long long>(target_spacing) * target_spacing) {
          ok = false;
          break;
        }
      }
      if (ok) {
        sel.push_back(idx);
      }
    }
    if (sel.empty()) {
      sel.push_back(frag.front());
    }
    sites += static_cast<int>(sel.size());

    for (int idx : sel) {
      const int ix = g.intersections[idx].x;
      const int iy = g.intersections[idx].y;

      // Offset one row off the crossing row (toward core center; stay in core).
      int nr = 0, bd = INT_MAX;
      for (size_t i = 0; i < row_ys.size(); ++i) {
        const int d = std::abs(row_ys[i] - iy);
        if (d < bd) {
          bd = d;
          nr = static_cast<int>(i);
        }
      }
      int trow = nr;
      if (nr + 1 < static_cast<int>(row_ys.size())) {
        trow = nr + 1;
      } else if (nr > 0) {
        trow = nr - 1;
      }
      const int ty = row_ys[trow];
      const odb::dbOrientType orient
          = row_orient.count(ty) ? row_orient[ty]
                                 : odb::dbOrientType(odb::dbOrientType::R0);

      // Cell origin: align Y pad to the V line, then snap to the site grid so
      // the fixed cell is legal (Y still overlaps the wire within half a site).
      int ox = ix - y_pin_off;
      ox = core.xMin() + ((ox - core.xMin() + site_w / 2) / site_w) * site_w;

      const std::string nm
          = "tsv_resv_" + std::to_string(ix) + "_" + std::to_string(ty);
      odb::dbInst* inst = placeTsvCell(tsv, nm, ox, ty, orient);
      if (!inst) {
        continue;
      }
      placed++;

      // Layer-selective keepout centered on the V line, covering the TSV row +
      // both bounding BPR rails.
      const int kx0 = ix - kw2, kx1 = ix + kw2;
      const int ky0 = ty - kext, ky1 = ty + row_h + kext;
      odb::dbBlockage::create(block_, kx0, ky0, kx1, ky1);  // no cells/taps
      // Obstruct ONLY BPR -> breaks the followpin rails for the front<->back
      // crossing. Do NOT obstruct BM3/BM4: those coarse power straps live well
      // above the TSV's M1/BM1 and obstructing them collapses the whole upper
      // PDN (pdngen drops the BM3-V/BM4-H straps + via columns). They pass
      // harmlessly over the keepout; their via columns land at strap crossings
      // (off the keepouts), where BPR stays intact.
      if (odb::dbTechLayer* bpr = tech->findLayer("BPR")) {
        odb::dbObstruction::create(block_, bpr, kx0, ky0, kx1, ky1);
      }
    }
  }
  logger_->info(CMS,
                150,
                "Reserved {} TSV sites (+keepouts) across {} fragments "
                "({} candidate sites)",
                placed,
                g.fragments.size(),
                sites);
}

// THE shared, deterministic mesh grid. Computes the same deformed/pruned grid
// the old createMeshGrid builds, but (a) spans the CORE box (placement-
// independent, valid even before any cell is placed), and (b) takes the PDN
// strap geometry from parameters so it can run pre-PDN. Returns geometry only.
FrozenGrid ClockMesh::computeFrozenGrid(odb::dbTechLayer* h_layer,
                                        odb::dbTechLayer* v_layer,
                                        int pitch,
                                        int v_strap_pitch,
                                        int v_strap_offset,
                                        int v_strap_width)
{
  FrozenGrid g;
  if (!block_ || !h_layer || !v_layer) {
    return g;
  }
  mesh_h_layer_ = h_layer;
  mesh_v_layer_ = v_layer;
  const odb::Rect core = block_->getCoreArea();

  // --- track alignment (must land on the mesh layers' routing tracks) ---
  int h_track_pitch = 0, h_track_offset = 0;
  int v_track_pitch = 0, v_track_offset = 0;
  if (odb::dbTrackGrid* ht = block_->findTrackGrid(h_layer)) {
    std::vector<int> yt;
    ht->getGridY(yt);
    if (yt.size() >= 2) {
      h_track_pitch = yt[1] - yt[0];
      h_track_offset = yt[0];
    }
  }
  if (odb::dbTrackGrid* vt = block_->findTrackGrid(v_layer)) {
    std::vector<int> xt;
    vt->getGridX(xt);
    if (xt.size() >= 2) {
      v_track_pitch = xt[1] - xt[0];
      v_track_offset = xt[0];
    }
  }

  // Round the requested pitch to a multiple of the combined track pitch so
  // wires always fall on tracks.
  auto gcd_int = [](int a, int b) {
    while (b != 0) {
      int t = b;
      b = a % b;
      a = t;
    }
    return a;
  };
  auto lcm_int = [&](int a, int b) {
    if (a == 0) {
      return b;
    }
    if (b == 0) {
      return a;
    }
    return (a / gcd_int(a, b)) * b;
  };
  int combined = lcm_int(lcm_int(0, h_track_pitch), v_track_pitch);
  int aligned_pitch = pitch;
  if (combined > 0) {
    aligned_pitch = ((pitch + combined / 2) / combined) * combined;
    if (aligned_pitch < combined) {
      aligned_pitch = combined;
    }
  }
  g.aligned_pitch = aligned_pitch;

  // Track-aligned start, then clamp inside the core (CMS-122): no node/keepout
  // may sit in the BPR-free margin outside the core (that is where the router
  // builds the front<->back via stack).
  int ax = core.xMin(), ay = core.yMin();
  if (v_track_pitch > 0) {
    int idx
        = (core.xMin() - v_track_offset + v_track_pitch / 2) / v_track_pitch;
    ax = v_track_offset + idx * v_track_pitch;
  }
  if (h_track_pitch > 0) {
    int idx
        = (core.yMin() - h_track_offset + h_track_pitch / 2) / h_track_pitch;
    ay = h_track_offset + idx * h_track_pitch;
  }
  const int x_margin = v_layer->getWidth();
  const int y_margin = h_layer->getWidth();
  while (v_track_pitch > 0 && ax < core.xMin() + x_margin) {
    ax += v_track_pitch;
  }
  while (h_track_pitch > 0 && ay < core.yMin() + y_margin) {
    ay += h_track_pitch;
  }
  const int x_end = core.xMax() - x_margin;
  const int y_end = core.yMax() - y_margin;

  // PDN straps: prefer the REAL built PDN (post-placement) so the shift dodges
  // the exact straps -- the analytic params miss the vdd/vss strap PAIR
  // (pdn.tcl -spacing makes two stripes per pitch). Fall back to analytic only
  // when no PDN exists yet (floorplan).
  collectPdnVStraps();
  if (pdn_vstrap_x_.empty()) {
    collectPdnVStrapsFromParams(
        v_strap_pitch, v_strap_offset, v_strap_width, core);
  }
  collectPdnHStraps();  // same-layer horizontal straps (co-planar PDN)

  const int v_half = v_layer->getWidth() / 2;
  const int h_half = h_layer->getWidth() / 2;
  // 2x min spacing (2 x 0.056 = 0.112um): extra same-metal distance to the
  // power straps and between clock wires so there is no coupling/interference.
  const int spacing = 2
                      * (v_layer->getSpacing() > 0 ? v_layer->getSpacing()
                                                   : v_layer->getWidth());
  const int h_spacing = 2
                        * (h_layer->getSpacing() > 0 ? h_layer->getSpacing()
                                                     : h_layer->getWidth());

  // Vertical wires, shifted clear of the strap bands (dropped if no room).
  for (int x = ax; x <= x_end; x += aligned_pitch) {
    const int xs = shiftVClearOfPdn(x, v_half, spacing);
    if (xs == INT_MIN) {
      continue;
    }
    g.v_wires.emplace_back(
        v_layer,
        nullptr,
        odb::Rect(xs - v_half, core.yMin(), xs + v_half, core.yMax()),
        false);
  }

  // Horizontal wires, shifted clear of same-layer strap bands (co-planar PDN:
  // nothing punches through the mesh layers, so no notching -- each wire stays
  // continuous and the mesh remains ONE connected grid, no fragments).
  int last_hy = INT_MIN;
  for (int y = ay; y <= y_end; y += aligned_pitch) {
    const int ys = shiftHClearOfPdn(y, h_half, h_spacing);
    if (ys == INT_MIN) {
      continue;
    }
    // post-shift spacing check vs the previous clock wire (same metal)
    int dy = ys - last_hy;
    if (dy < 0) {
      dy = -dy;
    }
    if (last_hy != INT_MIN && dy < h_layer->getWidth() + h_spacing) {
      continue;
    }
    last_hy = ys;
    g.h_wires.emplace_back(
        h_layer,
        nullptr,
        odb::Rect(core.xMin(), ys - h_half, core.xMax(), ys + h_half),
        true);
  }

  // Drop horizontal segments that no vertical wire crosses -> fragments the
  // mesh.
  pruneOrphanHSegments(g.h_wires, g.v_wires);

  // Intersections + connected fragments (union-find over shared V wire / H
  // seg).
  std::vector<int> parent;
  std::function<int(int)> find = [&](int a) {
    while (parent[a] != a) {
      parent[a] = parent[parent[a]];
      a = parent[a];
    }
    return a;
  };
  auto unite = [&](int a, int b) { parent[find(a)] = find(b); };

  std::map<int, std::vector<int>> by_vx;  // shared vertical wire
  for (const MeshWire& h : g.h_wires) {
    const int hy = (h.rect.yMin() + h.rect.yMax()) / 2;
    std::vector<int> on_h;  // intersections sharing this H segment
    for (const MeshWire& v : g.v_wires) {
      const int vx = (v.rect.xMin() + v.rect.xMax()) / 2;
      if (vx >= h.rect.xMin() && vx <= h.rect.xMax() && hy >= v.rect.yMin()
          && hy <= v.rect.yMax()) {
        const int idx = static_cast<int>(g.intersections.size());
        g.intersections.emplace_back(vx, hy, v_layer);
        parent.push_back(idx);
        on_h.push_back(idx);
        by_vx[vx].push_back(idx);
      }
    }
    for (size_t i = 1; i < on_h.size(); ++i) {
      unite(on_h[0], on_h[i]);
    }
  }
  for (const auto& [vx, idxs] : by_vx) {
    for (size_t i = 1; i < idxs.size(); ++i) {
      unite(idxs[0], idxs[i]);
    }
  }
  std::map<int, std::vector<int>> comps;
  for (int i = 0; i < static_cast<int>(g.intersections.size()); ++i) {
    comps[find(i)].push_back(i);
  }
  for (auto& [root, members] : comps) {
    g.fragments.push_back(std::move(members));
  }

  logger_->info(
      CMS,
      144,
      "Frozen grid: {} V wires, {} H segments, {} intersections, "
      "{} fragments (aligned pitch {:.3f}um)",
      g.v_wires.size(),
      g.h_wires.size(),
      g.intersections.size(),
      g.fragments.size(),
      aligned_pitch / static_cast<double>(block_->getDbUnitsPerMicron()));
  return g;
}

// Creates horizontal mesh wires at regular pitch intervals
void ClockMesh::createHorizontalWires(odb::dbNet* net,
                                      odb::dbTechLayer* layer,
                                      const odb::Rect& bbox,
                                      int pitch,
                                      std::vector<MeshWire>& wires)
{
  if (!net || !layer) {
    logger_->error(CMS, 101, "Invalid net or layer for horizontal wires");
    return;
  }
  const int wire_width = layer->getWidth();
  const int half_width = wire_width / 2;
  const int x_start = bbox.xMin();
  const int x_end = bbox.xMax();
  const int min_segment_length = wire_width * 2;
  // 2x min spacing: extra distance to power straps / other clock wires
  const int spacing = 2 * std::max(layer->getSpacing(), 1);

  int last_y = INT_MIN;
  int shifted = 0, dropped = 0;
  for (int y_pos = bbox.yMin(); y_pos <= bbox.yMax(); y_pos += pitch) {
    // co-planar PDN: SHIFT clear of same-layer horizontal power straps
    // (no notching -- nothing punches through the mesh layers, so the wire
    // stays continuous and the mesh stays one connected grid)
    int y = pdn_hstrap_y_.empty()
                ? y_pos
                : shiftHClearOfPdn(y_pos, half_width, spacing);
    if (y == INT_MIN) {  // no room to clear a strap -> drop this wire
      ++dropped;
      continue;
    }
    if (y != y_pos) {
      // wire collides with a horizontal PDN strap. Opt-in
      // (-remove_colliding_wires): DROP it instead of shifting -> fewer mesh
      // intersections -> fewer mesh drivers -> lower clock power.
      if (remove_colliding_) {
        ++dropped;
        continue;
      }
      ++shifted;
    }
    // post-shift spacing check: keep same-metal distance to the previous
    // clock wire (no DRC short, no coupling between bunched-up wires)
    int dy = y - last_y;
    if (dy < 0) {
      dy = -dy;
    }
    if (last_y != INT_MIN && dy < wire_width + spacing) {
      continue;
    }
    last_y = y;
    odb::Rect full_rect(x_start, y - half_width, x_end, y + half_width);
    for (const odb::Rect& seg :
         clipWireByBlockages(full_rect, true, min_segment_length)) {
      wires.emplace_back(layer, net, seg, true);
    }
  }
  if (!pdn_hstrap_y_.empty()) {
    logger_->info(CMS,
                  153,
                  "Horizontal clock wires: {} shifted clear of PDN straps, "
                  "{} dropped ({})",
                  shifted,
                  dropped,
                  remove_colliding_ ? "-remove_colliding_wires: on straps"
                                    : "no room in gap");
  }
}

// Creates vertical mesh wires at regular pitch intervals
void ClockMesh::createVerticalWires(odb::dbNet* net,
                                    odb::dbTechLayer* layer,
                                    const odb::Rect& bbox,
                                    int pitch,
                                    std::vector<MeshWire>& wires)
{
  if (!net || !layer) {
    logger_->error(CMS, 102, "Invalid net or layer for vertical wires");
    return;
  }
  const int wire_width = layer->getWidth();
  const int half_width = wire_width / 2;
  const int y_start = bbox.yMin();
  const int y_end = bbox.yMax();
  const int min_segment_length = wire_width * 2;
  // 2x min spacing: extra distance to power straps / other clock wires
  const int spacing = 2 * std::max(layer->getSpacing(), 1);

  int last_x = INT_MIN;
  int shifted = 0, dropped = 0;
  for (int x_pos = bbox.xMin(); x_pos <= bbox.xMax(); x_pos += pitch) {
    int x = pdn_vstrap_x_.empty()
                ? x_pos
                : shiftVClearOfPdn(x_pos, half_width, spacing);
    if (x == INT_MIN) {  // no room to clear a strap -> drop this wire
      ++dropped;
      continue;
    }
    if (x != x_pos) {
      // wire collides with a vertical PDN strap. Opt-in
      // (-remove_colliding_wires): DROP it instead of shifting -> fewer mesh
      // intersections -> fewer mesh drivers -> lower clock power.
      if (remove_colliding_) {
        ++dropped;
        continue;
      }
      ++shifted;
    }
    // avoid stacking two shifted wires on top of each other
    int dx = x - last_x;
    if (dx < 0) {
      dx = -dx;
    }
    if (last_x != INT_MIN && dx < wire_width + spacing) {
      continue;
    }
    last_x = x;
    const int x_min = x - half_width;
    const int x_max = x + half_width;
    odb::Rect full_rect(x_min, y_start, x_max, y_end);
    for (const odb::Rect& seg :
         clipWireByBlockages(full_rect, false, min_segment_length)) {
      wires.emplace_back(layer, net, seg, false);
    }
  }
  if (!pdn_vstrap_x_.empty()) {
    logger_->info(CMS,
                  141,
                  "Vertical clock wires: {} shifted clear of PDN straps, "
                  "{} dropped ({})",
                  shifted,
                  dropped,
                  remove_colliding_ ? "-remove_colliding_wires: on straps"
                                    : "no room in gap");
  }
}

// Creates vias where horizontal and vertical wires intersect
void ClockMesh::createViasAtIntersections(const std::vector<MeshWire>& h_wires,
                                          const std::vector<MeshWire>& v_wires,
                                          std::vector<MeshVia>& vias)
{
  if (h_wires.empty() || v_wires.empty()) {
    logger_->warn(CMS, 103, "No wires provided for via creation");
    return;
  }

  for (const auto& h_wire : h_wires) {
    for (const auto& v_wire : v_wires) {
      odb::Rect intersection = h_wire.rect;
      if (intersection.intersects(v_wire.rect)) {
        intersection.set_xlo(std::max(h_wire.rect.xMin(), v_wire.rect.xMin()));
        intersection.set_ylo(std::max(h_wire.rect.yMin(), v_wire.rect.yMin()));
        intersection.set_xhi(std::min(h_wire.rect.xMax(), v_wire.rect.xMax()));
        intersection.set_yhi(std::min(h_wire.rect.yMax(), v_wire.rect.yMax()));

        if (h_wire.layer == v_wire.layer) {
          continue;
        }

        odb::dbTechLayer* lower_layer = nullptr;
        odb::dbTechLayer* upper_layer = nullptr;
        if (h_wire.layer->getRoutingLevel() < v_wire.layer->getRoutingLevel()) {
          lower_layer = h_wire.layer;
          upper_layer = v_wire.layer;
        } else {
          lower_layer = v_wire.layer;
          upper_layer = h_wire.layer;
        }
        vias.emplace_back(lower_layer, upper_layer, h_wire.net, intersection);
      }
    }
  }
}

// Gets or creates the mesh net with name "{clock}_mesh"
odb::dbNet* ClockMesh::getOrCreateClockNet(const std::string& clock_name)
{
  if (!block_) {
    logger_->error(CMS, 104, "No block available for net creation");
    return nullptr;
  }

  // Find the root clock net name from SDC and use it as base for everything
  if (mesh_net_name_.empty()) {
    sta::Sdc* sdc = sta_->cmdSdc();
    if (sdc) {
      for (auto clk : sdc->clocks()) {
        if (std::string(clk->name()) == clock_name) {
          for (const sta::Pin* pin : clk->pins()) {
            odb::dbITerm* iterm;
            odb::dbBTerm* bterm;
            odb::dbModITerm* moditerm;
            network_->staToDb(pin, iterm, bterm, moditerm);
            odb::dbNet* net
                = bterm ? bterm->getNet() : (iterm ? iterm->getNet() : nullptr);
            if (net) {
              mesh_net_name_ = net->getName();
              break;
            }
          }
          break;
        }
      }
    }
    if (mesh_net_name_.empty()) {
      mesh_net_name_ = clock_name;
    }
  }

  std::string mesh_net_name = mesh_net_name_ + "_mesh";

  odb::dbNet* mesh_net = block_->findNet(mesh_net_name.c_str());
  if (!mesh_net) {
    mesh_net = odb::dbNet::create(block_, mesh_net_name.c_str());
    if (mesh_net) {
      mesh_net->setSpecial();
      mesh_net->setSigType(odb::dbSigType::CLOCK);
    }
  } else {
    if (!mesh_net->isSpecial()) {
      mesh_net->setSpecial();
    }
    if (mesh_net->getSigType() != odb::dbSigType::CLOCK) {
      mesh_net->setSigType(odb::dbSigType::CLOCK);
    }
  }

  return mesh_net;
}

// Writes mesh wire geometries to the database as special wires
void ClockMesh::writeWiresToDb(const std::vector<MeshWire>& wires)
{
  if (!block_) {
    logger_->error(CMS, 106, "No block available for writing wires");
    return;
  }

  for (auto& wire : wires) {
    if (!wire.net || !wire.layer) {
      continue;
    }
    odb::dbSWire* swire = nullptr;
    auto swires = wire.net->getSWires();
    if (!swires.empty()) {
      swire = *swires.begin();
    } else {
      swire = odb::dbSWire::create(wire.net, odb::dbWireType::ROUTED);
    }
    if (!swire) {
      logger_->warn(CMS,
                    107,
                    "Failed to get/create SWire for net {}",
                    wire.net->getName());
      continue;
    }
    odb::dbSBox::create(swire,
                        wire.layer,
                        wire.rect.xMin(),
                        wire.rect.yMin(),
                        wire.rect.xMax(),
                        wire.rect.yMax(),
                        odb::dbWireShapeType::STRIPE);
  }
}

// Writes via geometries to the database using tech vias
void ClockMesh::writeViasToDb(const std::vector<MeshVia>& vias)
{
  if (!block_) {
    logger_->error(CMS, 109, "No block available for writing vias");
    return;
  }
  odb::dbTech* tech = db_->getTech();
  if (!tech) {
    logger_->error(CMS, 110, "No technology available");
    return;
  }

  for (auto& via : vias) {
    if (!via.net || !via.lower_layer || !via.upper_layer) {
      continue;
    }
    int lower_level = via.lower_layer->getRoutingLevel();
    int upper_level = via.upper_layer->getRoutingLevel();
    int via_x = (via.area.xMin() + via.area.xMax()) / 2;
    int via_y = (via.area.yMin() + via.area.yMax()) / 2;

    for (int level = lower_level; level < upper_level; level++) {
      odb::dbTechLayer* lower = tech->findRoutingLayer(level);
      odb::dbTechLayer* upper = tech->findRoutingLayer(level + 1);
      if (!lower || !upper) {
        continue;
      }

      odb::dbTechVia* tech_via = nullptr;
      for (odb::dbTechVia* tv : tech->getVias()) {
        if (tv->getBottomLayer() == lower && tv->getTopLayer() == upper) {
          tech_via = tv;
          break;
        }
      }
      if (!tech_via) {
        continue;
      }

      odb::dbSWire* swire = nullptr;
      auto swires = via.net->getSWires();
      if (!swires.empty()) {
        swire = *swires.begin();
      } else {
        swire = odb::dbSWire::create(via.net, odb::dbWireType::ROUTED);
      }
      if (!swire) {
        continue;
      }

      odb::dbSBox::create(
          swire, tech_via, via_x, via_y, odb::dbWireShapeType::NONE);
    }
  }
}

// Main function: creates mesh grid, places buffers, and runs CTS
void ClockMesh::createMeshGrid(const std::string& clock_name,
                               odb::dbTechLayer* h_layer,
                               odb::dbTechLayer* v_layer,
                               int pitch,
                               const std::vector<std::string>& buffer_list,
                               int macro_halo_dbu,
                               const std::vector<std::string>& cts_buffer_list,
                               const std::string& mesh_strategy,
                               bool remove_colliding,
                               bool checkerboard_buffers)
{
  if (db_) {
    odb::dbChip* chip = db_->getChip();
    if (chip) {
      block_ = chip->getBlock();
    }
  }
  if (!logger_ || !block_) {
    logger_->error(CMS, 113, "ClockMesh not properly initialized");
    return;
  }

  mesh_h_layer_ = h_layer;
  mesh_v_layer_ = v_layer;
  remove_colliding_ = remove_colliding;
  checkerboard_buffers_ = checkerboard_buffers;

  collectBlockageRects(macro_halo_dbu);
  collectPdnVStraps();
  collectPdnHStraps();

  odb::dbTech* tech = db_->getTech();
  int h_level = h_layer ? h_layer->getRoutingLevel() : 0;
  int v_level = v_layer ? v_layer->getRoutingLevel() : 0;
  int bterm_level = std::max(h_level, v_level) + 1;

  odb::dbTechLayer* bterm_layer = tech->findRoutingLayer(bterm_level);
  if (bterm_layer) {
    bterm_layer_ = bterm_layer;
  }

  // Align grid to the actual mesh layer tracks (generalized for any tech).
  // Horizontal wires on h_layer: y-positions must align with h_layer tracks.
  // Vertical wires on v_layer: x-positions must align with v_layer tracks.
  // For each mesh layer we need BOTH directions of routing tracks: the buffer
  // proxy BTerm at (inter.x, inter.y) must lie on the X-track AND Y-track of
  // the M4 grid for the detail router to terminate exactly at the BTerm; same
  // for M5. Without this, the router lands at the nearest valid grid
  // intersection (which can be ~500 dbu off) and the routed wire never
  // geometrically overlaps the SWire stripe -> ~22% buffers isolated.
  int h_track_pitch = 0, h_track_offset = 0;  // M4 Y-tracks
  int h_xtrack_pitch = 0,
      h_xtrack_offset = 0;  // M4 X-tracks (for BTerm X access)
  int v_track_pitch = 0, v_track_offset = 0;  // M5 X-tracks
  int v_ytrack_pitch = 0,
      v_ytrack_offset = 0;  // M5 Y-tracks (for BTerm Y access)

  if (h_layer) {
    odb::dbTrackGrid* h_tracks = block_->findTrackGrid(h_layer);
    if (h_tracks) {
      std::vector<int> y_tracks, x_tracks;
      h_tracks->getGridY(y_tracks);
      h_tracks->getGridX(x_tracks);
      if (y_tracks.size() >= 2) {
        h_track_pitch = y_tracks[1] - y_tracks[0];
        h_track_offset = y_tracks[0];
      }
      if (x_tracks.size() >= 2) {
        h_xtrack_pitch = x_tracks[1] - x_tracks[0];
        h_xtrack_offset = x_tracks[0];
      }
    }
    if (h_track_pitch == 0) {
      logger_->warn(CMS,
                    211,
                    "No y-tracks found for horizontal mesh layer {}",
                    h_layer->getName());
    }
  }

  if (v_layer) {
    odb::dbTrackGrid* v_tracks = block_->findTrackGrid(v_layer);
    if (v_tracks) {
      std::vector<int> x_tracks, y_tracks;
      v_tracks->getGridX(x_tracks);
      v_tracks->getGridY(y_tracks);
      if (x_tracks.size() >= 2) {
        v_track_pitch = x_tracks[1] - x_tracks[0];
        v_track_offset = x_tracks[0];
      }
      if (y_tracks.size() >= 2) {
        v_ytrack_pitch = y_tracks[1] - y_tracks[0];
        v_ytrack_offset = y_tracks[0];
      }
    }
    if (v_track_pitch == 0) {
      logger_->warn(CMS,
                    212,
                    "No x-tracks found for vertical mesh layer {}",
                    v_layer->getName());
    }
  }

  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    findClockSinks();
  }
  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    logger_->error(CMS, 114, "No sinks found for clock: {}", clock_name);
    return;
  }
  const std::vector<ClockSink>& sinks = clockToSinks_[clock_name];

  odb::Rect bbox = calculateBoundingBox(sinks);
  if (bbox.area() == 0) {
    logger_->error(CMS, 115, "Invalid bounding box calculated");
    return;
  }

  // BUG FIX: previously aligned pitch to max(h_track_pitch, v_track_pitch),
  // which only honors ONE layer's Y-tracks. The proxy BTerm position must lie
  // on BOTH X-track AND Y-track of the mesh routing layer so detail router
  // can terminate exactly on it. Otherwise router lands at the nearest valid
  // grid intersection (~500 dbu off) and the routed wire never overlaps the
  // SWire stripe -> ~22% buffers electrically isolated.
  //
  // Align pitch to LCM of ALL FOUR track pitches (M4-Y, M4-X, M5-Y, M5-X).
  auto gcd_int = [](int a, int b) {
    while (b != 0) {
      int t = b;
      b = a % b;
      a = t;
    }
    return a;
  };
  auto lcm_int = [&](int a, int b) {
    if (a == 0) {
      return b;
    }
    if (b == 0) {
      return a;
    }
    return (a / gcd_int(a, b)) * b;
  };
  int aligned_pitch = pitch;
  int combined = 0;
  combined = lcm_int(combined, h_track_pitch);
  combined = lcm_int(combined, h_xtrack_pitch);
  combined = lcm_int(combined, v_track_pitch);
  combined = lcm_int(combined, v_ytrack_pitch);
  if (combined > 0) {
    aligned_pitch = ((pitch + combined / 2) / combined) * combined;
    if (aligned_pitch < combined) {
      aligned_pitch = combined;
    }
  }
  logger_->info(
      CMS,
      213,
      "Track-aligned pitch: input={}dbu, LCM of all track pitches={}dbu, "
      "aligned pitch={}dbu",
      pitch,
      combined,
      aligned_pitch);

  // Align x-start to v_layer tracks (vertical wire x-positions)
  int aligned_x_start = bbox.xMin();
  if (v_track_pitch > 0) {
    int idx
        = (bbox.xMin() - v_track_offset + v_track_pitch / 2) / v_track_pitch;
    aligned_x_start = v_track_offset + idx * v_track_pitch;
  }
  // Align y-start to h_layer tracks (horizontal wire y-positions)
  int aligned_y_start = bbox.yMin();
  if (h_track_pitch > 0) {
    int idx
        = (bbox.yMin() - h_track_offset + h_track_pitch / 2) / h_track_pitch;
    aligned_y_start = h_track_offset + idx * h_track_pitch;
  }
  // Keep node rows/cols -- and the proxy BTerms placed on them -- INSIDE the
  // core. Track alignment can snap the start just past the core edge into the
  // BPR-free margin (observed: bottom mesh row at y=1120 < core.yMin=1152).
  // A BTerm in that margin lets the router climb a front<->back via stack:
  // outside the core there is no BPR to block the M0<->BPR<->BM crossing, so
  // detail-route pin access pulls the block-I/O BTerm up to top metal. Inside
  // the core every row has BPR, which blocks the climb -- which is why the 62
  // in-core crossings stay on BM1/BM2 and only the 2 below the core climbed.
  // Bump the start inward by whole tracks and pull the far edge in so every
  // node clears the core boundary by at least one wire width.
  const odb::Rect core_area = block_->getCoreArea();
  const int x_margin = mesh_v_layer_ ? mesh_v_layer_->getWidth() : 0;
  const int y_margin = mesh_h_layer_ ? mesh_h_layer_->getWidth() : 0;
  while (v_track_pitch > 0 && aligned_x_start < core_area.xMin() + x_margin) {
    aligned_x_start += v_track_pitch;
  }
  while (h_track_pitch > 0 && aligned_y_start < core_area.yMin() + y_margin) {
    aligned_y_start += h_track_pitch;
  }
  const int aligned_x_end = std::min(bbox.xMax(), core_area.xMax() - x_margin);
  const int aligned_y_end = std::min(bbox.yMax(), core_area.yMax() - y_margin);
  odb::Rect aligned_bbox(
      aligned_x_start, aligned_y_start, aligned_x_end, aligned_y_end);
  logger_->info(CMS,
                122,
                "Grid clamped inside core [{},{}]-[{},{}] (margins x={} y={})",
                aligned_bbox.xMin(),
                aligned_bbox.yMin(),
                aligned_bbox.xMax(),
                aligned_bbox.yMax(),
                x_margin,
                y_margin);

  // Log grid size
  double dbu_per_um = tech->getDbUnitsPerMicron();
  int h_wire_count
      = (aligned_pitch > 0) ? (aligned_bbox.dy() / aligned_pitch + 1) : 0;
  int v_wire_count
      = (aligned_pitch > 0) ? (aligned_bbox.dx() / aligned_pitch + 1) : 0;
  logger_->info(CMS,
                121,
                "Clock mesh parameters: pitch={:.3f}um, expected {} H-lines x "
                "{} V-lines, "
                "bbox {:.3f}x{:.3f}um",
                aligned_pitch / dbu_per_um,
                h_wire_count,
                v_wire_count,
                aligned_bbox.dx() / dbu_per_um,
                aligned_bbox.dy() / dbu_per_um);

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(CMS, 116, "Failed to find clock net for '{}'", clock_name);
    return;
  }

  std::vector<MeshWire> h_wires;
  if (h_layer) {
    createHorizontalWires(
        mesh_net, h_layer, aligned_bbox, aligned_pitch, h_wires);
  }

  std::vector<MeshWire> v_wires;
  if (v_layer) {
    createVerticalWires(
        mesh_net, v_layer, aligned_bbox, aligned_pitch, v_wires);
  }

  // Drop horizontal segments in regions that have no vertical clock wire:
  // such a region can host no buffer/TSV, so its horizontal metal is dangling.
  pruneOrphanHSegments(h_wires, v_wires);

  std::vector<MeshVia> vias;
  if (!h_wires.empty() && !v_wires.empty()) {
    createViasAtIntersections(h_wires, v_wires, vias);
  }

  if (!h_wires.empty()) {
    writeWiresToDb(h_wires);
  }
  if (!v_wires.empty()) {
    writeWiresToDb(v_wires);
  }
  if (!vias.empty()) {
    writeViasToDb(vias);
  }

  if (!h_wires.empty() && !v_wires.empty()) {
    grid_intersections_.clear();
    for (size_t hi = 0; hi < h_wires.size(); ++hi) {
      const MeshWire& h_wire = h_wires[hi];
      int h_y = (h_wire.rect.yMin() + h_wire.rect.yMax()) / 2;
      for (size_t vi = 0; vi < v_wires.size(); ++vi) {
        const MeshWire& v_wire = v_wires[vi];
        int v_x = (v_wire.rect.xMin() + v_wire.rect.xMax()) / 2;
        // Skip intersections where clipped wires don't actually cover the point
        if (v_x < h_wire.rect.xMin() || v_x > h_wire.rect.xMax()) {
          continue;
        }
        if (h_y < v_wire.rect.yMin() || h_y > v_wire.rect.yMax()) {
          continue;
        }
        // Safety net — should not fire if clipping is correct
        if (isBlocked(v_x, h_y)) {
          continue;
        }
        odb::dbTechLayer* buf_layer
            = selectBufferLayer(h_wire.layer, v_wire.layer);
        grid_intersections_.emplace_back(v_x, h_y, buf_layer);
        grid_intersections_.back().row = static_cast<int>(hi);
        grid_intersections_.back().col = static_cast<int>(vi);
        // Store intersection as connection point on both mesh layers
        // so convertSWireToWire breaks mesh wires at via locations
        if (h_wire.layer) {
          mesh_connection_points_.insert(
              std::make_tuple(v_x, h_y, h_wire.layer->getRoutingLevel()));
        }
        if (v_wire.layer) {
          mesh_connection_points_.insert(
              std::make_tuple(v_x, h_y, v_wire.layer->getRoutingLevel()));
        }
      }
    }
  }

  mesh_wires_.clear();
  mesh_wires_.insert(mesh_wires_.end(), h_wires.begin(), h_wires.end());
  mesh_wires_.insert(mesh_wires_.end(), v_wires.begin(), v_wires.end());

  int buffers_placed = 0;
  if (!buffer_list.empty() && !grid_intersections_.empty()) {
    // Dispatch by mesh_strategy:
    //   "uniform"   — single master at every intersection (legacy path)
    //   "adaptive"  — clocksyn buffer_entire_grid=true (buffer everywhere,
    //                 sized per intersection load)
    //   "set_cover" — clocksyn buffer_entire_grid=false (greedy set-cover,
    //                 some intersections left empty)
    // Empty strategy → fall back to historical heuristic (list size).
    std::string strat = mesh_strategy;
    if (strat.empty()) {
      strat = (buffer_list.size() > 1) ? "adaptive" : "uniform";
    }
    if (checkerboard_buffers_ && strat != "uniform") {
      logger_->warn(CMS,
                    735,
                    "-checkerboard_buffers only applies to the uniform "
                    "strategy; '{}' places by its own policy (flag ignored)",
                    strat);
    }
    if (strat == "set_cover") {
      placeBuffersSetCover(block_,
                           network_,
                           logger_,
                           mesh_h_layer_,
                           mesh_v_layer_,
                           mesh_wires_,
                           clockToSinks_[clock_name],
                           grid_intersections_,
                           buffer_list);
    } else if (strat == "adaptive") {
      placeBuffersLoadAdaptive(block_,
                               network_,
                               logger_,
                               mesh_h_layer_,
                               mesh_v_layer_,
                               mesh_wires_,
                               clockToSinks_[clock_name],
                               grid_intersections_,
                               buffer_list);
    } else {
      placeBuffersAtIntersections(buffer_list[0], mesh_net);
    }
    connectBuffersToNets(mesh_net, clock_name);
    // Count what was actually placed (set_cover may leave intersections empty).
    buffers_placed = 0;
    for (const GridIntersection& g : grid_intersections_) {
      if (g.has_buffer) {
        buffers_placed++;
      }
    }

    // Detach the flop clock pins from the clock net BEFORE running CTS so
    // TritonCTS trees ONLY the mesh-buffer inputs (the intended sinks) and
    // does not also build a register tree (clk_regs). Left attached, that
    // register tree is later orphaned by connect_sinks_to_mesh /
    // create_sink_taps (both move each flop iterm onto the mesh) and its dead
    // nets break detailed routing for any non-trivial CTS buffer library.
    // Both sink paths disconnect()+connect() their iterms, so detaching here
    // is a safe no-op for their later reconnection.
    int sinks_detached = 0;
    for (const ClockSink& s : clockToSinks_[clock_name]) {
      if (s.iterm) {
        s.iterm->disconnect();
        ++sinks_detached;
      }
    }
    logger_->info(CMS,
                  709,
                  "Detached {} flop sinks from '{}' before CTS "
                  "(mesh buffers are the only CTS sinks)",
                  sinks_detached,
                  clock_name);

    std::string cts_net_name
        = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
    buildCtsTreeToBuffers(cts_net_name, cts_buffer_list);
  }

  mesh_generated_ = true;

  // Count distinct grid lines (a blockage-clipped line becomes multiple
  // segments)
  std::set<int> h_positions, v_positions;
  for (const MeshWire& w : h_wires) {
    h_positions.insert((w.rect.yMin() + w.rect.yMax()) / 2);
  }
  for (const MeshWire& w : v_wires) {
    v_positions.insert((w.rect.xMin() + w.rect.xMax()) / 2);
  }

  logger_->info(CMS,
                120,
                "Created clock mesh: {} horizontal lines x {} vertical lines "
                "({} H-segments, {} V-segments), "
                "{} intersections, {} vias, {} buffers placed",
                h_positions.size(),
                v_positions.size(),
                h_wires.size(),
                v_wires.size(),
                grid_intersections_.size(),
                vias.size(),
                buffers_placed);
}

// Creates BTerm pins for sinks at grid intersections for router connections
void ClockMesh::connectSinksViaRouter(const std::string& clock_name,
                                      odb::dbTechLayer* proxy_layer)
{
  if (!mesh_generated_) {
    logger_->error(
        CMS, 500, "Mesh not generated. Call create_clock_mesh first.");
    return;
  }

  if (!proxy_layer) {
    logger_->error(CMS, 502, "Proxy layer not specified for sink BTerms");
    return;
  }

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(
        CMS, 501, "Could not find mesh net for clock '{}'", clock_name);
    return;
  }

  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    logger_->warn(CMS, 503, "No sinks found for clock '{}'", clock_name);
    return;
  }

  std::vector<MeshWire> h_wires;
  std::vector<MeshWire> v_wires;
  for (const MeshWire& wire : mesh_wires_) {
    if (wire.is_horizontal) {
      h_wires.push_back(wire);
    } else {
      v_wires.push_back(wire);
    }
  }

  const std::vector<ClockSink>& sinks = clockToSinks_[clock_name];

  int bterms_created = 0;

  for (const auto& sink : sinks) {
    if (!sink.iterm) {
      continue;
    }

    int sink_x, sink_y;
    computeITermPosition(sink.iterm, sink_x, sink_y);
    odb::Point sink_point(sink_x, sink_y);
    odb::dbTechLayer* grid_layer = nullptr;
    odb::Point grid_point
        = findNearestGridWire(sink_point, h_wires, v_wires, &grid_layer);

    if (!grid_layer) {
      logger_->warn(CMS, 504, "No grid point found for sink {}", sink.name);
      continue;
    }

    // Safety net: snap point must not land inside a blockage
    if (isBlocked(grid_point.x(), grid_point.y())) {
      logger_->warn(
          CMS,
          511,
          "Sink {} snap point ({}, {}) is inside a blockage, skipping",
          sink.name,
          grid_point.x(),
          grid_point.y());
      continue;
    }

    // Place BTERM on the mesh grid layer for direct connectivity
    int min_width = grid_layer->getWidth();
    int half_width = min_width / 2;

    int grid_x = grid_point.x();
    int grid_y = grid_point.y();

    // If another pin already exists at this position, offset along the wire
    bool is_h_wire = (grid_layer == mesh_h_layer_);
    auto pos_key
        = std::make_tuple(grid_x, grid_y, grid_layer->getRoutingLevel());
    int count = 0;
    while (mesh_connection_points_.count(pos_key)) {
      count++;
      int offset = min_width * count;
      if (is_h_wire) {
        grid_x = grid_point.x() + ((count % 2 == 0) ? offset : -offset);
      } else {
        grid_y = grid_point.y() + ((count % 2 == 0) ? offset : -offset);
      }
      pos_key = std::make_tuple(grid_x, grid_y, grid_layer->getRoutingLevel());
    }

    sink_bterm_counter_++;
    std::string net_name = "sink_" + std::to_string(sink_bterm_counter_);
    odb::dbNet* sink_net = odb::dbNet::create(block_, net_name.c_str());
    if (!sink_net) {
      logger_->warn(CMS,
                    505,
                    "Failed to create net '{}' for sink {}",
                    net_name,
                    sink.name);
      continue;
    }
    sink_net->setSigType(odb::dbSigType::CLOCK);

    std::string bterm_name
        = "sink_bterm_" + std::to_string(sink_bterm_counter_);
    odb::dbBTerm* bterm = odb::dbBTerm::create(sink_net, bterm_name.c_str());
    if (!bterm) {
      logger_->warn(CMS,
                    506,
                    "Failed to create BTerm '{}' for sink {}",
                    bterm_name,
                    sink.name);
      continue;
    }
    bterm->setIoType(odb::dbIoType::INPUT);
    bterm->setSigType(odb::dbSigType::CLOCK);

    odb::dbBPin* bpin = odb::dbBPin::create(bterm);
    if (bpin) {
      odb::dbBox::create(bpin,
                         grid_layer,
                         grid_x - half_width,
                         grid_y - half_width,
                         grid_x + half_width,
                         grid_y + half_width);
      bpin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    // Store connection point so convertSWireToWire can break mesh wire here
    mesh_connection_points_.insert(
        std::make_tuple(grid_x, grid_y, grid_layer->getRoutingLevel()));

    // Remember the sink bterm name at this mesh-net coord on BOTH mesh
    // layers so cross-layer mesh-stripe junctions at the intersection merge
    // electrically with the sub-net stub in the SPICE deck.
    proxy_alias_[std::make_tuple(
        grid_x, grid_y, mesh_h_layer_->getRoutingLevel())]
        = bterm_name;
    if (mesh_v_layer_) {
      proxy_alias_[std::make_tuple(
          grid_x, grid_y, mesh_v_layer_->getRoutingLevel())]
          = bterm_name;
    }

    sink.iterm->disconnect();
    sink.iterm->connect(sink_net);
    bterms_created++;
  }

  logger_->info(CMS, 510, "Created {} sink BTerms for routing", bterms_created);
}

// Finds the nearest grid intersection to a point.
// Grid intersections are guaranteed to be on connected mesh segments
// (they're where an H-wire and V-wire both cover the point after clipping).
// Returns false if grid_intersections_ is empty.
bool ClockMesh::findNearestGridIntersection(const odb::Point& loc,
                                            odb::Point& out_point,
                                            odb::dbTechLayer** out_layer) const
{
  int64_t best_dist2 = std::numeric_limits<int64_t>::max();
  const GridIntersection* best = nullptr;
  for (const GridIntersection& inter : grid_intersections_) {
    int64_t dx = inter.x - loc.x();
    int64_t dy = inter.y - loc.y();
    int64_t d2 = dx * dx + dy * dy;
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best = &inter;
    }
  }
  if (!best) {
    return false;
  }
  out_point = odb::Point(best->x, best->y);
  if (out_layer) {
    *out_layer = best->layer;
  }
  return true;
}

// Finds the nearest mesh grid wire to a given point.
odb::Point ClockMesh::findNearestGridWire(const odb::Point& loc,
                                          const std::vector<MeshWire>& h_wires,
                                          const std::vector<MeshWire>& v_wires,
                                          odb::dbTechLayer** out_grid_layer)
{
  int best_x = loc.x();
  int best_y = loc.y();
  int min_dist_h = std::numeric_limits<int>::max();
  int min_dist_v = std::numeric_limits<int>::max();
  odb::dbTechLayer* nearest_h_layer = nullptr;
  odb::dbTechLayer* nearest_v_layer = nullptr;

  // Only consider wires whose clipped extent actually reaches the sink's coord
  for (const auto& wire : h_wires) {
    if (loc.x() < wire.rect.xMin() || loc.x() > wire.rect.xMax()) {
      continue;
    }
    int wire_y = (wire.rect.yMin() + wire.rect.yMax()) / 2;
    int dist = std::abs(wire_y - loc.y());
    if (dist < min_dist_h) {
      min_dist_h = dist;
      best_y = wire_y;
      nearest_h_layer = wire.layer;
    }
  }
  for (const auto& wire : v_wires) {
    if (loc.y() < wire.rect.yMin() || loc.y() > wire.rect.yMax()) {
      continue;
    }
    int wire_x = (wire.rect.xMin() + wire.rect.xMax()) / 2;
    int dist = std::abs(wire_x - loc.x());
    if (dist < min_dist_v) {
      min_dist_v = dist;
      best_x = wire_x;
      nearest_v_layer = wire.layer;
    }
  }

  odb::Point grid_point;
  if (out_grid_layer) {
    if (nearest_h_layer && nearest_v_layer) {
      if (min_dist_h < min_dist_v) {
        *out_grid_layer = nearest_h_layer;
        grid_point = odb::Point(loc.x(), best_y);
      } else {
        *out_grid_layer = nearest_v_layer;
        grid_point = odb::Point(best_x, loc.y());
      }
    } else if (nearest_h_layer) {
      *out_grid_layer = nearest_h_layer;
      grid_point = odb::Point(loc.x(), best_y);
    } else {
      *out_grid_layer = nearest_v_layer;
      grid_point = odb::Point(best_x, loc.y());
    }
  } else {
    grid_point = (min_dist_h < min_dist_v) ? odb::Point(loc.x(), best_y)
                                           : odb::Point(best_x, loc.y());
  }

  return grid_point;
}

// Creates a via stack between two layers at a given location
void ClockMesh::createViaStackAtPoint(const odb::Point& location,
                                      odb::dbTechLayer* from_layer,
                                      odb::dbTechLayer* to_layer,
                                      odb::dbNet* net)
{
  if (!from_layer || !to_layer || !net) {
    return;
  }

  odb::dbTech* tech = db_->getTech();
  if (!tech) {
    return;
  }

  int from_level = from_layer->getRoutingLevel();
  int to_level = to_layer->getRoutingLevel();

  if (from_level == to_level) {
    return;
  }
  if (from_level > to_level) {
    std::swap(from_layer, to_layer);
    std::swap(from_level, to_level);
  }

  for (int level = from_level; level < to_level; level++) {
    odb::dbTechLayer* lower = tech->findRoutingLayer(level);
    odb::dbTechLayer* upper = tech->findRoutingLayer(level + 1);
    if (!lower || !upper) {
      continue;
    }
    odb::Rect via_area(location.x(), location.y(), location.x(), location.y());
    connection_vias_.emplace_back(lower, upper, net, via_area);
  }
}

// Selects the lower routing layer for buffer placement
odb::dbTechLayer* ClockMesh::selectBufferLayer(odb::dbTechLayer* h_layer,
                                               odb::dbTechLayer* v_layer)
{
  if (!h_layer) {
    return v_layer;
  }
  if (!v_layer) {
    return h_layer;
  }

  int h_level = h_layer->getRoutingLevel();
  int v_level = v_layer->getRoutingLevel();

  return (h_level < v_level) ? h_layer : v_layer;
}

// Returns the output ITerm of a buffer instance
odb::dbITerm* ClockMesh::getBufferOutputPin(odb::dbInst* buffer)
{
  if (!buffer) {
    return nullptr;
  }
  for (odb::dbITerm* iterm : buffer->getITerms()) {
    odb::dbMTerm* mterm = iterm->getMTerm();
    if (mterm && mterm->getIoType() == odb::dbIoType::OUTPUT) {
      return iterm;
    }
  }
  return nullptr;
}

// Returns the input ITerm of a buffer instance
odb::dbITerm* ClockMesh::getBufferInputPin(odb::dbInst* buffer)
{
  if (!buffer) {
    return nullptr;
  }

  for (odb::dbITerm* iterm : buffer->getITerms()) {
    odb::dbMTerm* mterm = iterm->getMTerm();
    if (mterm && mterm->getIoType() == odb::dbIoType::INPUT) {
      return iterm;
    }
  }
  return nullptr;
}

// Places buffer cells at each mesh grid intersection
// Places a gt2_6t_TSV front<->back crossing cell at (x,y). Returns nullptr if
// the master is missing (caller falls back to a direct buffer->BTerm net).
odb::dbInst* ClockMesh::placeTsvCell(odb::dbMaster* master,
                                     const std::string& name,
                                     int x,
                                     int y,
                                     odb::dbOrientType orient)
{
  if (!master) {
    return nullptr;
  }
  odb::dbInst* inst = odb::dbInst::create(block_, master, name.c_str());
  if (!inst) {
    return nullptr;
  }
  inst->setOrient(orient);
  inst->setLocation(x, y);
  // FIRM so detailed_placement cannot drag the TSV back onto a power strap.
  inst->setPlacementStatus(odb::dbPlacementStatus::FIRM);
  return inst;
}

// Finds the placement row whose origin-y is nearest to y.
bool ClockMesh::nearestRow(int y, int& row_y, odb::dbOrientType& orient) const
{
  odb::dbRow* best = nullptr;
  int best_d = INT_MAX;
  for (odb::dbRow* row : block_->getRows()) {
    int d = row->getOrigin().y() - y;
    if (d < 0) {
      d = -d;
    }
    if (d < best_d) {
      best_d = d;
      best = row;
    }
  }
  if (!best) {
    return false;
  }
  row_y = best->getOrigin().y();
  orient = best->getOrient();
  return true;
}

void ClockMesh::placeBuffersAtIntersections(const std::string& buffer_master,
                                            odb::dbNet* mesh_net)
{
  if (grid_intersections_.empty()) {
    logger_->warn(CMS, 307, "No grid intersections available");
    return;
  }
  odb::dbMaster* master = db_->findMaster(buffer_master.c_str());
  if (!master) {
    logger_->error(CMS, 308, "Buffer master '{}' not found", buffer_master);
    return;
  }

  // Front<->back crossing cell, placed next to each mesh buffer. If the master
  // isn't loaded, we skip TSV insertion and connect buffers straight to the
  // mesh BTerm (original behavior).
  odb::dbMaster* tsv_master = db_->findMaster("gt2_6t_TSV");
  if (!tsv_master) {
    logger_->warn(CMS,
                  730,
                  "gt2_6t_TSV master not found; building mesh without "
                  "front<->back TSV crossings");
  }

  // Fixed cells (LOCKED tap cells, macros, COVER) that detailed_placement will
  // NOT move. A FIRM TSV must not overlap these — it may overlap movable design
  // cells (dpl legalizes those around it), but not locked taps/macros. Also
  // precompute the TSV's backside Y-pad x-range and the site width for nudging.
  std::vector<odb::Rect> fixed_rects;
  for (odb::dbInst* fi : block_->getInsts()) {
    const auto st = fi->getPlacementStatus();
    if (st == odb::dbPlacementStatus::LOCKED
        || st == odb::dbPlacementStatus::FIRM
        || st == odb::dbPlacementStatus::COVER) {
      fixed_rects.push_back(fi->getBBox()->getBox());
    }
  }
  const int spacing
      = std::max(mesh_v_layer_ ? mesh_v_layer_->getSpacing() : 0, 1);
  int tsv_w = 0, tsv_h = 0, ypad_lo = 0, ypad_hi = 0, ypad_cx = 0, site_w = 1;
  if (tsv_master) {
    tsv_w = static_cast<int>(tsv_master->getWidth());
    tsv_h = static_cast<int>(tsv_master->getHeight());
    ypad_lo = 0;
    ypad_hi = tsv_w;
    ypad_cx = tsv_w / 2;
    if (odb::dbMTerm* yt = tsv_master->findMTerm("Y")) {
      const odb::Rect yb = yt->getBBox();
      ypad_lo = yb.xMin();
      ypad_hi = yb.xMax();
      ypad_cx = (ypad_lo + ypad_hi) / 2;
    }
    if (odb::dbSite* s = master->getSite()) {
      site_w = std::max(static_cast<int>(s->getWidth()), 1);
    }
  }
  int tsv_nudged = 0, tsv_dropped = 0, driver_removed = 0;
  int checker_skipped = 0;

  // Prefer TSV x where the backside Y pad center lands ON a BM1 routing track
  // (the grid stub then sits exactly on its track, no widening).
  std::vector<int> drv_vtracks;
  if (block_ && mesh_v_layer_) {
    if (odb::dbTrackGrid* tg = block_->findTrackGrid(mesh_v_layer_)) {
      tg->getGridX(drv_vtracks);
    }
  }
  auto pad_track_dist = [&](int x) {
    if (drv_vtracks.empty()) {
      return 0;
    }
    const int pc = x + ypad_cx;
    auto it = std::lower_bound(drv_vtracks.begin(), drv_vtracks.end(), pc);
    int best = INT_MAX;
    if (it != drv_vtracks.end()) {
      best = std::min(best, *it - pc);
    }
    if (it != drv_vtracks.begin()) {
      best = std::min(best, pc - *std::prev(it));
    }
    return best;
  };

  for (GridIntersection& inter : grid_intersections_) {
    // Checkerboard pattern: drivers only at (row+col)-even nodes, so no driver
    // has a driven orthogonal neighbor. Halves the driver/TSV count; the mesh
    // WIRE grid (and sink-tap candidate sites) is untouched.
    if (checkerboard_buffers_ && inter.row >= 0 && inter.col >= 0
        && ((inter.row + inter.col) & 1)) {
      ++checker_skipped;
      continue;  // no TSV, no buffer at this node
    }
    // Snap to the nearest row so the row-tall cells sit between that row's BPR
    // rails (the rails the Stage-2 break removes at the TSV).
    int row_y = inter.y;
    odb::dbOrientType orient;
    const bool snapped = nearestRow(inter.y, row_y, orient);

    // Place the TSV so its backside Y pad sits ON the mesh node (inter.x = the
    // vertical clock stripe). The Y->mesh connection then becomes a
    // via/abutment at the node instead of a routed backside stub (backside
    // routing is the scarce resource). The stripe is already >=0.056 clear of
    // straps, and the TSV's backside metal lives within the Y-pad region, so it
    // stays clear too.
    int tsv_x = inter.x;
    if (tsv_master) {
      const int want = inter.x - ypad_cx;  // Y-pad center on the mesh node
      // Opt-in (-remove_colliding_wires): if this node's drive-TSV sits on a
      // power strap, SKIP the whole driver (TSV + mesh buffer) instead of
      // nudging it off. Removes mesh drivers at colliding nodes to cut the
      // mesh-driver power tier; the mesh WIRE grid is untouched, so mesh
      // connectivity and sink-tap candidate sites are preserved.
      if (remove_colliding_) {
        const int rlo = want + ypad_lo - spacing;
        const int rhi = want + ypad_hi + spacing;
        bool on_strap = false;
        for (const auto& b : pdn_vstrap_x_) {
          if (rlo < b.second && b.first < rhi) {
            on_strap = true;
            break;
          }
        }
        if (on_strap) {
          ++driver_removed;
          continue;  // no TSV, no buffer at this node
        }
      }
      // Accept x only if the backside Y-pad region stays >= spacing from every
      // vertical strap AND the full footprint overlaps no fixed cell
      // (tap/macro).
      auto fits = [&](int x) {
        const int rlo = x + ypad_lo - spacing;
        const int rhi = x + ypad_hi + spacing;
        for (const auto& b : pdn_vstrap_x_) {
          if (rlo < b.second && b.first < rhi) {
            return false;
          }
        }
        const int tlo = x, thi = x + tsv_w;
        const int blo = row_y, bhi = row_y + tsv_h;
        for (const odb::Rect& r : fixed_rects) {
          if (tlo < r.xMax() && r.xMin() < thi && blo < r.yMax()
              && r.yMin() < bhi) {
            return false;
          }
        }
        return true;
      };
      // Prefer the on-node position; if it overlaps a tap/strap, nudge outward
      // in site steps to the nearest legal spot.
      int chosen = INT_MIN;
      const int max_nudge = 24 * site_w;
      // BEST-fit: nearest fitting spot whose Y pad lands EXACTLY on a track
      int best_td = INT_MAX;
      for (int d = 0; d <= max_nudge && best_td > 0; d += site_w) {
        for (int s = 1; s >= -1; s -= 2) {
          if (d == 0 && s < 0) {
            continue;
          }
          const int cand = want + s * d;
          if (!fits(cand)) {
            continue;
          }
          const int td = pad_track_dist(cand);
          if (td < best_td) {
            best_td = td;
            chosen = cand;
          }
        }
      }
      if (chosen == INT_MIN) {
        ++tsv_dropped;
        logger_->warn(CMS,
                      731,
                      "No tap-clear/PDN-clear spot for TSV near ({}, {}); "
                      "skipping its front<->back crossing",
                      inter.x,
                      inter.y);
      } else {
        if (chosen != want) {
          ++tsv_nudged;
        }
        tsv_x = chosen;
        inter.tsv_inst = placeTsvCell(tsv_master,
                                      "tsv_buf_" + std::to_string(inter.x) + "_"
                                          + std::to_string(inter.y),
                                      tsv_x,
                                      row_y,
                                      orient);
        // future TSVs must avoid this one too
        fixed_rects.emplace_back(tsv_x, row_y, tsv_x + tsv_w, row_y + tsv_h);
      }
    }

    // Buffer goes beside the TSV (to its right). It's a frontside cell, so its
    // output reaches TSV.A on the frontside and it needs no backside room.
    const int buf_x = tsv_master
                          ? (tsv_x + static_cast<int>(tsv_master->getWidth()))
                          : inter.x;
    std::string buf_name
        = "mesh_buf_" + std::to_string(inter.x) + "_" + std::to_string(inter.y);
    odb::dbInst* buf_inst
        = odb::dbInst::create(block_, master, buf_name.c_str());
    if (buf_inst) {
      if (snapped) {
        buf_inst->setOrient(orient);
      }
      buf_inst->setLocation(buf_x, row_y);
      buf_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      inter.buffer_inst = buf_inst;
      inter.has_buffer = true;
    }
  }
  if (tsv_master) {
    logger_->info(CMS,
                  732,
                  "TSV placement: {} nudged off fixed cells (taps/macros), "
                  "{} dropped (no clear spot)",
                  tsv_nudged,
                  tsv_dropped);
  }
  if (driver_removed > 0) {
    logger_->info(CMS,
                  733,
                  "remove_colliding_wires: dropped {} mesh drivers on power "
                  "straps (mesh wire grid preserved)",
                  driver_removed);
  }
  if (checkerboard_buffers_) {
    logger_->info(CMS,
                  734,
                  "checkerboard_buffers: skipped {} of {} intersections "
                  "(drivers at (row+col)-even nodes only; wire grid preserved)",
                  checker_skipped,
                  static_cast<int>(grid_intersections_.size()));
  }
}

// Connects buffer inputs to the original clock net
void ClockMesh::connectBuffersToNets(odb::dbNet* mesh_net,
                                     const std::string& clock_name)
{
  if (!mesh_net) {
    logger_->error(CMS, 118, "Mesh net not provided");
    return;
  }

  std::string base_name = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
  odb::dbNet* orig_clock_net = block_->findNet(base_name.c_str());
  if (!orig_clock_net) {
    logger_->error(CMS, 119, "Original clock net '{}' not found", base_name);
    return;
  }

  for (const GridIntersection& intersection : grid_intersections_) {
    if (!intersection.buffer_inst || !intersection.has_buffer) {
      continue;
    }
    odb::dbITerm* input_pin = getBufferInputPin(intersection.buffer_inst);
    if (input_pin) {
      input_pin->connect(orig_clock_net);
    }
  }
}

// Creates proxy BTerm pins at intersections for buffer output routing
void ClockMesh::setupProxyBTerms(const std::string& clock_name,
                                 odb::dbTechLayer* proxy_layer)
{
  if (!mesh_generated_) {
    logger_->error(
        CMS, 600, "Mesh not generated. Call create_clock_mesh first.");
    return;
  }

  if (!proxy_layer) {
    logger_->error(CMS, 601, "Proxy layer not specified");
    return;
  }

  proxy_layer_ = proxy_layer;

  if (bterm_layer_ && proxy_layer != bterm_layer_) {
    logger_->warn(CMS,
                  603,
                  "Proxy layer {} differs from auto-computed BTERM layer {}",
                  proxy_layer->getName(),
                  bterm_layer_->getName());
  }

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(
        CMS, 602, "Could not find mesh net for clock '{}'", clock_name);
    return;
  }

  int created = createProxyBTermsWithSeparateNets(mesh_net, proxy_layer_);
  logger_->info(
      CMS, 610, "Created {} proxy BTERMs for buffer outputs", created);
}

// Creates separate nets and BTerm pins for each buffer at intersections
int ClockMesh::createProxyBTermsWithSeparateNets(
    odb::dbNet* mesh_net,
    odb::dbTechLayer* /* proxy_layer */)
{
  if (!mesh_net) {
    return 0;
  }
  // Place buffer BTERMs on the bottom mesh layer for direct connectivity
  int h_level = mesh_h_layer_ ? mesh_h_layer_->getRoutingLevel() : 0;
  int v_level = mesh_v_layer_ ? mesh_v_layer_->getRoutingLevel() : 0;
  odb::dbTechLayer* buf_bterm_layer
      = (h_level <= v_level) ? mesh_h_layer_ : mesh_v_layer_;
  if (!buf_bterm_layer) {
    return 0;
  }
  int min_width = buf_bterm_layer->getWidth();
  int half_width = min_width / 2;
  int created_count = 0;
  std::string base_name = mesh_net_name_.empty() ? "clk" : mesh_net_name_;

  for (GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.buffer_inst) {
      continue;
    }
    std::string net_name = base_name + "_buf_" + std::to_string(inter.x) + "_"
                           + std::to_string(inter.y);
    odb::dbITerm* output_pin = getBufferOutputPin(inter.buffer_inst);

    // Net the mesh BTerm lives on. With a TSV crossing this is the BACKSIDE
    // net (TSV.Y + BTerm) and the buffer output goes on a separate FRONTSIDE
    // net (buffer out + TSV.A). Without a TSV the buffer output connects to the
    // BTerm net directly (original mesh behavior).
    odb::dbNet* bterm_net = nullptr;

    if (inter.tsv_inst) {
      odb::dbITerm* tsvA = inter.tsv_inst->findITerm("A");  // frontside (M1)
      odb::dbITerm* tsvY = inter.tsv_inst->findITerm("Y");  // backside  (BM1)

      // FRONTSIDE: buffer output + TSV.A (router routes this on the frontside).
      odb::dbNet* fnet = odb::dbNet::create(block_, net_name.c_str());
      if (!fnet) {
        logger_->warn(CMS, 607, "Failed to create net '{}'", net_name);
        continue;
      }
      fnet->setSigType(odb::dbSigType::CLOCK);
      if (output_pin) {
        output_pin->disconnect();
        output_pin->connect(fnet);
      }
      if (tsvA) {
        tsvA->connect(fnet);
      }

      // BACKSIDE: TSV.Y + the mesh BTerm.
      std::string bnet_name = "b_" + net_name;
      odb::dbNet* bnet = odb::dbNet::create(block_, bnet_name.c_str());
      if (!bnet) {
        logger_->warn(CMS, 608, "Failed to create net '{}'", bnet_name);
        continue;
      }
      bnet->setSigType(odb::dbSigType::CLOCK);
      if (tsvY) {
        tsvY->connect(bnet);
        // Straight track-aligned SPECIAL stub from the Y pad to the grid
        // intersection (the closest grid point -- Y sits on the V wire), so
        // the router never has to route this net.
        drawTsvGridStub(bnet, tsvY->getBBox(), inter.y);
      }
      bterm_net = bnet;
    } else {
      odb::dbNet* buf_net = odb::dbNet::create(block_, net_name.c_str());
      if (!buf_net) {
        logger_->warn(CMS, 607, "Failed to create net '{}'", net_name);
        continue;
      }
      buf_net->setSigType(odb::dbSigType::CLOCK);
      if (output_pin) {
        output_pin->disconnect();
        output_pin->connect(buf_net);
      }
      bterm_net = buf_net;
    }

    std::string bterm_name = (inter.tsv_inst ? "b_proxy_" : "proxy_")
                             + std::to_string(inter.x) + "_"
                             + std::to_string(inter.y);
    odb::dbBTerm* bterm = odb::dbBTerm::create(bterm_net, bterm_name.c_str());
    if (!bterm) {
      continue;
    }
    bterm->setIoType(odb::dbIoType::INPUT);
    bterm->setSigType(odb::dbSigType::CLOCK);

    odb::dbBPin* bpin = odb::dbBPin::create(bterm);
    if (bpin) {
      odb::dbBox::create(bpin,
                         buf_bterm_layer,
                         inter.x - half_width,
                         inter.y - half_width,
                         inter.x + half_width,
                         inter.y + half_width);
      bpin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    // Store connection point so convertSWireToWire can break mesh wire here
    mesh_connection_points_.insert(
        std::make_tuple(inter.x, inter.y, buf_bterm_layer->getRoutingLevel()));

    // Remember the proxy bterm name at this mesh-net coord on BOTH mesh
    // layers (H and V). The mesh-stripe path on each layer has its own
    // junction at this coord; aliasing both to the same bterm name lets the
    // SPICE deck merge them electrically across layers at the intersection.
    proxy_alias_[std::make_tuple(
        inter.x, inter.y, mesh_h_layer_->getRoutingLevel())]
        = bterm_name;
    if (mesh_v_layer_) {
      proxy_alias_[std::make_tuple(
          inter.x, inter.y, mesh_v_layer_->getRoutingLevel())]
          = bterm_name;
    }

    inter.proxy_bterm = bterm;
    created_count++;
  }
  return created_count;
}

// Straight, track-aligned SPECIAL route from a TSV Y pad to the mesh H wire.
// Replaces the signal-router Y->BTerm path: deterministic, strap-free (the TSV
// site is already strap-clear), and invisible to GRT/DRT (net marked special),
// which removes that whole family of routing violations.
int ClockMesh::drawTsvGridStub(odb::dbNet* net,
                               const odb::Rect& ypad,
                               int target_y)
{
  if (!net || !block_ || !mesh_v_layer_ || !mesh_h_layer_) {
    return INT_MIN;
  }
  // nearest V-layer routing track to the Y pad center
  const int pc = (ypad.xMin() + ypad.xMax()) / 2;
  int tx = pc;
  if (odb::dbTrackGrid* tg = block_->findTrackGrid(mesh_v_layer_)) {
    std::vector<int> xs;
    tg->getGridX(xs);
    int best = INT_MAX;
    for (int x : xs) {
      const int d = std::abs(x - pc);
      if (d < best) {
        best = d;
        tx = x;
      }
    }
  }
  odb::dbSWire* swire = nullptr;
  auto swires = net->getSWires();
  if (!swires.empty()) {
    swire = *swires.begin();
  } else {
    swire = odb::dbSWire::create(net, odb::dbWireType::ROUTED);
  }
  if (!swire) {
    return INT_MIN;
  }
  const int vhw = mesh_v_layer_->getWidth() / 2;
  const int hhw = mesh_h_layer_->getWidth() / 2;
  // vertical stub: from the Y pad up/down through the via point on the H wire.
  // BM1 is the VERTICAL layer -- keep the stub strictly vertical: if the
  // snapped track misses the pad laterally, widen this single vertical box to
  // cover the pad; NEVER draw a horizontal BM1 jog.
  const int y0 = std::min(ypad.yMin(), target_y - hhw);
  const int y1 = std::max(ypad.yMax(), target_y + hhw);
  int x0 = tx - vhw;
  int x1 = tx + vhw;
  if (x1 < ypad.xMin()) {
    x1 = ypad.xMax();
  } else if (x0 > ypad.xMax()) {
    x0 = ypad.xMin();
  }
  odb::dbSBox::create(
      swire, mesh_v_layer_, x0, y0, x1, y1, odb::dbWireShapeType::STRIPE);
  // H<->V tech via at the grid point (same lookup the mesh vias use)
  odb::dbTechLayer* lower
      = (mesh_h_layer_->getRoutingLevel() < mesh_v_layer_->getRoutingLevel())
            ? mesh_h_layer_
            : mesh_v_layer_;
  odb::dbTechLayer* upper
      = (lower == mesh_h_layer_) ? mesh_v_layer_ : mesh_h_layer_;
  for (odb::dbTechVia* tv : db_->getTech()->getVias()) {
    if (tv->getBottomLayer() == lower && tv->getTopLayer() == upper) {
      odb::dbSBox::create(swire, tv, tx, target_y, odb::dbWireShapeType::NONE);
      break;
    }
  }
  net->setSpecial();
  return tx;
}

// SINK SIDE (mirror of the drive side). For each gap between adjacent vertical
// mesh wires on each horizontal mesh wire, a candidate sink-tap sits on the
// H-wire midpoint. Every FF clock pin is assigned to its NEAREST tap with load
// < capacity (spill to next-nearest); only taps that win >=1 FF are placed.
// Each placed tap = a sink-TSV (FIRM, on the row nearest the H-wire) whose Y
// goes on its own backside net with a proxy BTerm on the mesh stripe (router
// routes Y->BTerm, BPin overlaps the stripe = the tie to the grid), TSV.A feeds
// a sink-buffer placed just OUTSIDE the TSV keepout (movable -> legalized by a
// later detailed_placement on a powered row), and the buffer output drives the
// tap's assigned FFs (moved off the original clock net). The TSV keepout +
// BPR break are left to break_bpr_at_tsvs(), which sweeps all TSVs at once.
void ClockMesh::createSinkTaps(odb::dbTechLayer* h_layer,
                               odb::dbTechLayer* v_layer,
                               const std::string& tsv_master,
                               const std::string& sink_buffer_master,
                               int capacity,
                               int halo_dbu)
{
  if (!block_ || !h_layer || !v_layer) {
    logger_->error(CMS, 700, "create_sink_taps: missing block or mesh layers");
    return;
  }
  // FS mode (empty tsv_master): no TSV, no backside stub, no BPR surgery --
  // the LCB input is routed by GRT/DRT to a proxy BTerm pinned directly on the
  // nearest mesh wire (same mechanism as the mesh-driver proxies).
  const bool fs_mode = tsv_master.empty();
  odb::dbMaster* tsv_obj
      = fs_mode ? nullptr : db_->findMaster(tsv_master.c_str());
  odb::dbMaster* buf_obj = db_->findMaster(sink_buffer_master.c_str());
  if ((!fs_mode && !tsv_obj) || !buf_obj) {
    logger_->error(
        CMS, 701, "create_sink_taps: TSV or sink-buffer master not found");
    return;
  }

  // Locate the mesh net (stripes on h_layer). createMeshGrid records the BASE
  // clock name in mesh_net_name_; the stripe net is "<base>_mesh". Fall back to
  // scanning clock nets for special wires on h_layer.
  const std::string mesh_str
      = mesh_net_name_.empty() ? "" : mesh_net_name_ + "_mesh";
  odb::dbNet* mesh_net
      = mesh_str.empty() ? nullptr : block_->findNet(mesh_str.c_str());
  if (!mesh_net) {
    for (odb::dbNet* n : block_->getNets()) {
      if (n->getSigType() != odb::dbSigType::CLOCK) {
        continue;
      }
      bool has = false;
      for (odb::dbSWire* sw : n->getSWires()) {
        for (odb::dbSBox* b : sw->getWires()) {
          if (!b->isVia() && b->getTechLayer() == h_layer) {
            has = true;
            break;
          }
        }
        if (has) {
          break;
        }
      }
      if (has) {
        mesh_net = n;
        break;
      }
    }
  }
  if (!mesh_net) {
    logger_->error(CMS, 702, "create_sink_taps: clock mesh net not found");
    return;
  }

  // Mesh geometry: vertical-wire x-centers + horizontal segments (y, xlo, xhi).
  std::vector<int> vxs;
  std::vector<std::tuple<int, int, int>> hsegs;
  for (odb::dbSWire* sw : mesh_net->getSWires()) {
    for (odb::dbSBox* b : sw->getWires()) {
      if (b->isVia()) {
        continue;
      }
      odb::dbTechLayer* L = b->getTechLayer();
      const int dx = b->xMax() - b->xMin();
      const int dy = b->yMax() - b->yMin();
      if (L == v_layer && dy > dx) {
        vxs.push_back((b->xMin() + b->xMax()) / 2);
      } else if (L == h_layer && dx > dy) {
        hsegs.emplace_back((b->yMin() + b->yMax()) / 2, b->xMin(), b->xMax());
      }
    }
  }
  std::sort(vxs.begin(), vxs.end());
  vxs.erase(std::unique(vxs.begin(), vxs.end()), vxs.end());

  // Candidate taps: per H-segment, MULTIPLE per gap between consecutive
  // in-segment vertical wires, PLUS the edge spans between the outermost V
  // wires and the H-segment ends (core-boundary side) -- legal tap space that
  // on a sparse mesh can be wider than an interior gap. A gap can legally hold
  // more than one sink TSV, spaced so their BPR-cut halos (0.224um) don't
  // overlap. This multiplies tap sites WITHOUT a finer mesh pitch (which adds
  // mesh drivers and breaks legalization). n=1 -> the gap midpoint (the old
  // behavior). All candidates land ON the H mesh wire (y=hy), so each still
  // connects to the mesh; the placement search below keeps every placed TSV
  // strap-clear AND non-colliding.
  struct SinkTap
  {
    int mx;
    int ry;
    int hy;
    odb::dbOrientType orient;
  };
  const int tsv_w = fs_mode ? 0 : tsv_obj->getWidth();
  // min legal center-to-center: TSV footprint + its BPR-cut halo on each side,
  // so adjacent taps' blockages don't overlap. halo_dbu is the caller's TSV
  // halo; fall back to the TSV width if it wasn't supplied. FS taps are just
  // BTerm points -- space them ~0.5um so a gap yields several candidates.
  const int tap_stride
      = fs_mode
            ? std::max(halo_dbu * 2, (int) block_->getDbUnitsPerMicron() / 2)
            : tsv_w + 2 * (halo_dbu > 0 ? halo_dbu : tsv_w / 2);
  std::vector<SinkTap> taps;
  for (const auto& [hy, xlo, xhi] : hsegs) {
    std::vector<int> inseg;
    for (int vx : vxs) {
      if (vx >= xlo && vx <= xhi) {
        inseg.push_back(vx);
      }
    }
    int ry;
    odb::dbOrientType orient;
    if (!nearestRow(hy, ry, orient)) {
      continue;
    }
    // Gap bounds: the in-segment V wires, plus the segment ends (inset by the
    // TSV width so a tap's Y pad stays on the H wire). An edge bound is only
    // added when it leaves >= tsv_w of room to the nearest V wire, so the
    // edge-gap midpoint can't land on a mesh-drive TSV at the intersection.
    std::vector<int> bounds;
    const int elo = xlo + tsv_w;
    const int ehi = xhi - tsv_w;
    if (!inseg.empty()) {
      if (elo + tsv_w <= inseg.front()) {
        bounds.push_back(elo);
      }
      bounds.insert(bounds.end(), inseg.begin(), inseg.end());
      if (ehi - tsv_w >= inseg.back()) {
        bounds.push_back(ehi);
      }
    } else if (elo + tsv_w <= ehi) {
      // H segment with no V wire crossing it at all: previously produced zero
      // candidates; now its full span is usable.
      bounds.push_back(elo);
      bounds.push_back(ehi);
    }
    for (size_t i = 1; i < bounds.size(); i++) {
      const int lo = bounds[i - 1];
      const int gap = bounds[i] - lo;
      int n = (tap_stride > 0) ? gap / tap_stride : 1;
      if (n < 1) {
        n = 1;  // always at least the midpoint
      }
      for (int k = 0; k < n; k++) {
        // evenly spaced centers; n==1 gives the gap midpoint (old behavior)
        const int mx = lo + (int) ((2L * k + 1) * gap / (2L * n));
        taps.push_back({mx, ry, hy, orient});
      }
    }
  }

  // Pre-filter: drop candidates that sit on a mesh-drive TSV (placed at the
  // grid intersections by createMeshGrid). Multi-per-gap generation can put a
  // candidate on top of one; if the cluster->tap mapping then picks it,
  // placement can't legally use the spot and would skip it -> dropped FFs.
  // Removing occupied sites here means the mapping only ever sees legal sites,
  // so no cluster is assigned-then-skipped (0 dropped FFs).
  if (!fs_mode) {
    const int tsv_h = tsv_obj->getHeight();
    const int clr = (halo_dbu > 0) ? halo_dbu : tsv_w / 2;
    std::vector<odb::Rect> mesh_tsv;
    for (odb::dbInst* inst : block_->getInsts()) {
      if (inst->getMaster() == tsv_obj) {  // only mesh-drive TSVs exist yet
        mesh_tsv.push_back(inst->getBBox()->getBox());
      }
    }
    auto hits_mesh_tsv = [&](const SinkTap& tp) {
      // Conservative footprint: the placed cell origin is mx - ypoff (site-
      // snapped), and ypoff in [0, tsv_w], so the cell spans at most +/- tsv_w
      // around mx. Bloat by tsv_w each side (+ clr) so a surviving candidate's
      // real footprint is guaranteed clear of mesh TSVs regardless of ypoff.
      const odb::Rect c(tp.mx - tsv_w - clr,
                        tp.ry - clr,
                        tp.mx + tsv_w + clr,
                        tp.ry + tsv_h + clr);
      for (const odb::Rect& r : mesh_tsv) {
        if (c.xMin() < r.xMax() && r.xMin() < c.xMax() && c.yMin() < r.yMax()
            && r.yMin() < c.yMax()) {
          return true;
        }
      }
      return false;
    };
    const size_t before = taps.size();
    taps.erase(std::remove_if(taps.begin(), taps.end(), hits_mesh_tsv),
               taps.end());
    logger_->info(CMS,
                  704,
                  "create_sink_taps: {} candidate sites ({} removed for "
                  "overlapping mesh-drive TSVs)",
                  taps.size(),
                  before - taps.size());
  }

  // Sinks: every FF clock input pin.
  struct SinkPin
  {
    int x;
    int y;
    odb::dbITerm* iterm;
  };
  std::vector<SinkPin> sinks;
  for (odb::dbInst* inst : block_->getInsts()) {
    for (odb::dbITerm* it : inst->getITerms()) {
      odb::dbMTerm* mt = it->getMTerm();
      if (!mt || mt->getIoType() != odb::dbIoType::INPUT) {
        continue;
      }
      const bool clk_pin = mt->getSigType() == odb::dbSigType::CLOCK
                           || mt->getName() == "CLK" || mt->getName() == "CK";
      if (!clk_pin) {
        continue;
      }
      int x, y;
      computeITermPosition(it, x, y);
      sinks.push_back({x, y, it});
    }
  }

  // ---- radius-capped K-means clustering of the sinks ----
  // Replaces nearest-grid-tap + capacity. Cluster FFs by location; the sink
  // BUFFER goes at each cluster CENTROID (short leaves), while the sink TSV
  // stays on the mesh grid (where it can own its BPR-break blockage) and is
  // reached from the buffer via the sink_tap pin escape (M1->M6/M5). Two caps:
  //   count  <= F_max  (=capacity; binds in DENSE FF regions)
  //   radius <= R_max  (binds in SPARSE regions; = radius that holds ~F_max FFs
  //                     at the FF density, so the leaf RC keeps sink slew in
  //                     budget). F_max is set by a slew-budget SPICE sweep
  //                     (distributed leaf) -- ~80 for x4 at a 10%-period slew.
  const int F_max = capacity > 0 ? capacity : 80;
  const double dbu_d = block_->getDbUnitsPerMicron();
  const odb::Rect core_r0 = block_->getCoreArea();
  const double core_um2 = (core_r0.dx() / dbu_d) * (core_r0.dy() / dbu_d);
  const double dens = sinks.empty() ? 1.0 : sinks.size() / core_um2;
  const int R_max = (int) (std::sqrt(F_max / (dens * M_PI)) * dbu_d);

  std::vector<std::pair<double, double>> cent;
  const int K0 = std::max(1, (int) std::ceil(sinks.size() / (double) F_max));
  {
    std::vector<int> ord(sinks.size());
    for (size_t i = 0; i < ord.size(); i++) {
      ord[i] = (int) i;
    }
    std::sort(ord.begin(), ord.end(), [&](int a, int b) {
      return sinks[a].x != sinks[b].x ? sinks[a].x < sinks[b].x
                                      : sinks[a].y < sinks[b].y;
    });
    for (int k = 0; k < K0; k++) {
      const int id = ord[(long) k * ord.size() / K0];
      cent.emplace_back((double) sinks[id].x, (double) sinks[id].y);
    }
  }
  std::vector<int> asn(sinks.size(), 0);
  for (int iter = 0; iter < 60; iter++) {
    for (size_t i = 0; i < sinks.size(); i++) {
      double best = 1e30;
      int bk = 0;
      for (size_t k = 0; k < cent.size(); k++) {
        const double dx = sinks[i].x - cent[k].first;
        const double dy = sinks[i].y - cent[k].second;
        const double d = dx * dx + dy * dy;
        if (d < best) {
          best = d;
          bk = (int) k;
        }
      }
      asn[i] = bk;
    }
    std::vector<double> sx(cent.size(), 0), sy(cent.size(), 0);
    std::vector<int> cnt(cent.size(), 0);
    for (size_t i = 0; i < sinks.size(); i++) {
      const int k = asn[i];
      sx[k] += sinks[i].x;
      sy[k] += sinks[i].y;
      cnt[k]++;
    }
    for (size_t k = 0; k < cent.size(); k++) {
      if (cnt[k]) {
        cent[k] = {sx[k] / cnt[k], sy[k] / cnt[k]};
      }
    }
    // Split the worst cap-violator (count>F_max or radius>R_max) at its
    // farthest member; re-converge. One split per pass.
    int split_far = -1;
    for (size_t k = 0; k < cent.size(); k++) {
      if (!cnt[k]) {
        continue;
      }
      double rad = 0;
      int far = -1;
      for (size_t i = 0; i < sinks.size(); i++) {
        if (asn[i] == (int) k) {
          const double dx = sinks[i].x - cent[k].first;
          const double dy = sinks[i].y - cent[k].second;
          const double d = std::sqrt(dx * dx + dy * dy);
          if (d > rad) {
            rad = d;
            far = (int) i;
          }
        }
      }
      if (cnt[k] > F_max || rad > R_max) {
        split_far = far;
        break;
      }
    }
    if (split_far >= 0) {
      cent.emplace_back((double) sinks[split_far].x,
                        (double) sinks[split_far].y);
    } else if (iter > 0) {
      break;
    }
  }

  // Map each cluster -> nearest UNUSED grid tap (that tap hosts the cluster's
  // sink TSV). tap_ffs[t] = the cluster's FFs; tap_cent[t] = centroid where the
  // sink BUFFER is placed.
  std::vector<std::vector<odb::dbITerm*>> tap_ffs(taps.size());
  std::vector<std::pair<int, int>> tap_cent(taps.size(), {INT_MIN, INT_MIN});
  std::vector<char> tap_used(taps.size(), 0);
  std::vector<std::vector<odb::dbITerm*>> cl_ffs(cent.size());
  for (size_t i = 0; i < sinks.size(); i++) {
    cl_ffs[asn[i]].push_back(sinks[i].iterm);
  }
  int spilled = 0;
  for (size_t k = 0; k < cent.size(); k++) {
    if (cl_ffs[k].empty()) {
      continue;
    }
    long best = LONG_MAX;
    int bt = -1;
    for (size_t t = 0; t < taps.size(); t++) {
      if (tap_used[t]) {
        continue;
      }
      const long d = std::labs((long) cent[k].first - taps[t].mx)
                     + std::labs((long) cent[k].second - taps[t].hy);
      if (d < best) {
        best = d;
        bt = (int) t;
      }
    }
    if (bt < 0) {  // no free grid tap -> this cluster's FFs are unplaced
      spilled += (int) cl_ffs[k].size();
      continue;
    }
    tap_used[bt] = 1;
    tap_ffs[bt] = cl_ffs[k];
    tap_cent[bt] = {(int) cent[k].first, (int) cent[k].second};
  }

  // Placement geometry.
  const int cxmin = block_->getCoreArea().xMin();
  const int tw = fs_mode ? 0 : tsv_obj->getWidth();
  int sitew = tw;
  for (odb::dbRow* row : block_->getRows()) {
    if (row->getSite()) {
      sitew = row->getSite()->getWidth();
      break;
    }
  }
  int ypoff = tw / 2;
  if (odb::dbMTerm* ymt = fs_mode ? nullptr : tsv_obj->findMTerm("Y")) {
    odb::Rect yb;
    bool first = true;
    for (odb::dbMPin* mp : ymt->getMPins()) {
      for (odb::dbBox* bx : mp->getGeometry()) {
        odb::Rect r = bx->getBox();
        if (first) {
          yb = r;
          first = false;
        } else {
          yb.merge(r);
        }
      }
    }
    if (!first) {
      ypoff = (yb.xMin() + yb.xMax()) / 2;
    }
  }
  odb::dbTechLayer* bterm_layer
      = (h_layer->getRoutingLevel() <= v_layer->getRoutingLevel()) ? h_layer
                                                                   : v_layer;
  const int half_width = bterm_layer->getWidth() / 2;

  // The sink-TSV sits at the MIDPOINT between two clock V wires -- and the BM1
  // power straps live exactly between clock wires (the wires shifted off them),
  // so a midpoint can land ON a strap. The TSV cell must not overlap a strap
  // (its Y pad is BM1): shift the cell by site steps to the nearest clear spot.
  // The proxy BTerm stays at (mx,hy) on the mesh wire; the router absorbs the
  // small offset in the Y->BTerm route.
  collectPdnVStraps();
  const int strap_clear = 2
                          * (v_layer->getSpacing() > 0 ? v_layer->getSpacing()
                                                       : v_layer->getWidth());
  auto cell_clear = [&](int x) {
    for (const auto& band : pdn_vstrap_x_) {
      if (x - strap_clear < band.second && band.first < x + tw + strap_clear) {
        return false;
      }
    }
    return true;
  };
  // Prefer sites where the Y pad center lands ON a BM1 routing track (site
  // 0.084 vs track 0.112 align every 4th site, so a nearby site exists) --
  // then the grid stub needs no widening and sits exactly on its track.
  std::vector<int> vtracks;
  if (odb::dbTrackGrid* tg = block_->findTrackGrid(v_layer)) {
    tg->getGridX(vtracks);
  }
  auto pad_track_dist = [&](int x) {
    if (vtracks.empty()) {
      return 0;
    }
    const int pc = x + ypoff;
    auto it = std::lower_bound(vtracks.begin(), vtracks.end(), pc);
    int best = INT_MAX;
    if (it != vtracks.end()) {
      best = std::min(best, *it - pc);
    }
    if (it != vtracks.begin()) {
      best = std::min(best, pc - *std::prev(it));
    }
    return best;
  };
  int strap_shifted = 0;

  // Legality guard: with multiple candidate TSVs per gap, keep each placed sink
  // TSV from overlapping ANY already-placed TSV -- both the mesh-drive TSVs
  // (tsv_buf_*, placed by createMeshGrid at the grid intersections) AND earlier
  // sink TSVs. Seed with the existing TSV footprints, then reject any candidate
  // whose footprint (+ halo clearance) intersects one. Geometric (not
  // row-keyed) because a TSV cell can span several placement rows.
  const int tsv_h = fs_mode ? 0 : tsv_obj->getHeight();
  const int clr = (halo_dbu > 0) ? halo_dbu : tsv_w / 2;
  std::vector<odb::Rect> placed_tsv;
  for (odb::dbInst* inst : block_->getInsts()) {
    if (inst->getMaster() == tsv_obj) {
      placed_tsv.push_back(inst->getBBox()->getBox());
    }
  }
  auto occupied_near = [&](int x, int ry) {
    // candidate footprint at (x, ry), bloated by clearance on all sides
    const odb::Rect cand(x - clr, ry - clr, x + tsv_w + clr, ry + tsv_h + clr);
    for (const odb::Rect& r : placed_tsv) {
      if (cand.xMin() < r.xMax() && r.xMin() < cand.xMax()
          && cand.yMin() < r.yMax() && r.yMin() < cand.yMax()) {
        return true;
      }
    }
    return false;
  };

  int sn = 0;
  int assigned = 0;
  for (size_t t = 0; t < taps.size(); t++) {
    if (tap_ffs[t].empty()) {
      continue;
    }
    const SinkTap& tp = taps[t];
    if (fs_mode) {
      // FRONTSIDE tap: LCB at the cluster centroid; tap net = LCB input ->
      // proxy BTerm ON the mesh H wire at (mx, hy). Routed later by GRT/DRT
      // (connect_sink_taps -use_router); convert_mesh_swire / the SPICE
      // writer merge the BTerm node with the mesh via the alias below.
      const std::string idx = std::to_string(sn);
      odb::dbInst* bi
          = odb::dbInst::create(block_, buf_obj, ("sink_buf_" + idx).c_str());
      if (bi) {
        int bry;
        odb::dbOrientType borient;
        if (!nearestRow(tap_cent[t].second, bry, borient)) {
          bry = tp.ry;
          borient = tp.orient;
        }
        const int bbx
            = cxmin + ((tap_cent[t].first - cxmin + sitew / 2) / sitew) * sitew;
        bi->setOrient(borient);
        bi->setLocation(bbx, bry);
        bi->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      }
      odb::dbNet* tapn
          = odb::dbNet::create(block_, ("sink_tap_" + idx).c_str());
      tapn->setSigType(odb::dbSigType::CLOCK);
      if (odb::dbITerm* bin = bi ? getBufferInputPin(bi) : nullptr) {
        bin->connect(tapn);
      }
      odb::dbBTerm* bt
          = odb::dbBTerm::create(tapn, ("sink_prx_" + idx).c_str());
      if (bt) {
        bt->setIoType(odb::dbIoType::INPUT);
        bt->setSigType(odb::dbSigType::CLOCK);
        if (odb::dbBPin* bp = odb::dbBPin::create(bt)) {
          // Pin on the H layer: the tap point lies ON the mesh H wire (between
          // V wires) -- a pin on the V layer would float off-metal and can
          // track-snap onto a driver proxy pin (GRT-0031/0080).
          const int hw2 = h_layer->getWidth() / 2;
          odb::dbBox::create(
              bp, h_layer, tp.mx - hw2, tp.hy - hw2, tp.mx + hw2, tp.hy + hw2);
          bp->setPlacementStatus(odb::dbPlacementStatus::PLACED);
        }
        mesh_connection_points_.insert(
            std::make_tuple(tp.mx, tp.hy, h_layer->getRoutingLevel()));
        proxy_alias_[std::make_tuple(tp.mx, tp.hy, h_layer->getRoutingLevel())]
            = "sink_prx_" + idx;
        proxy_alias_[std::make_tuple(tp.mx, tp.hy, v_layer->getRoutingLevel())]
            = "sink_prx_" + idx;
      }
      odb::dbNet* drvn
          = odb::dbNet::create(block_, ("sink_drv_" + idx).c_str());
      drvn->setSigType(odb::dbSigType::CLOCK);
      if (bi) {
        if (odb::dbITerm* bout = getBufferOutputPin(bi)) {
          bout->connect(drvn);
        }
      }
      for (odb::dbITerm* ff : tap_ffs[t]) {
        ff->disconnect();
        ff->connect(drvn);
        assigned++;
      }
      sn++;
      continue;
    }
    const int ox0
        = cxmin + ((tp.mx - ypoff - cxmin + sitew / 2) / sitew) * sitew;
    // BEST-fit: among strap-clear sites nearest the midpoint, take the one
    // whose Y pad center lands EXACTLY on a BM1 track (dist 0 is always
    // reachable: gcd(site, track pitch) divides the pad/track residue).
    int ox = INT_MIN;
    int best_d = INT_MAX;
    for (int k = 0; k <= 24 && best_d > 0; k++) {
      for (int s = 1; s >= -1; s -= 2) {
        if (k == 0 && s < 0) {
          continue;
        }
        const int cand = ox0 + s * k * sitew;
        if (!cell_clear(cand)) {
          continue;
        }
        if (occupied_near(cand, tp.ry)) {
          continue;  // another sink TSV already sits here on this row
        }
        const int d = pad_track_dist(cand);
        if (d < best_d) {
          best_d = d;
          ox = cand;
        }
      }
    }
    if (ox == INT_MIN) {
      // No strap-clear, track-aligned, collision-free site found in the search.
      // Fall back to ox0: the candidate survived the pre-filter, so its
      // footprint is already clear of mesh-drive TSVs, and candidate spacing
      // (tap_stride) keeps it clear of sibling sink TSVs -- so ox0 is legal.
      // Never skip (that would drop the cluster's FFs); ox0 just may sit on a
      // power strap.
      ox = ox0;
    }
    if (ox != ox0) {
      strap_shifted++;
    }
    const std::string idx = std::to_string(sn);
    odb::dbInst* ti
        = placeTsvCell(tsv_obj, "sink_tsv_" + idx, ox, tp.ry, tp.orient);
    if (!ti) {
      continue;
    }
    placed_tsv.push_back(ti->getBBox()->getBox());  // reserve this footprint
    odb::dbInst* bi
        = odb::dbInst::create(block_, buf_obj, ("sink_buf_" + idx).c_str());
    if (bi) {
      // Sink BUFFER at the cluster CENTROID (short leaves to its FFs), NOT next
      // to the TSV. The TSV stays on the grid with its blockage; the buffer
      // reaches it via the sink_tap pin escape, so the buffer can sit anywhere.
      // Snap the centroid to the nearest placement row + site;
      // detailed_placement legalizes it.
      int bry;
      odb::dbOrientType borient;
      if (!nearestRow(tap_cent[t].second, bry, borient)) {
        bry = tp.ry;
        borient = tp.orient;
      }
      const int bbx
          = cxmin + ((tap_cent[t].first - cxmin + sitew / 2) / sitew) * sitew;
      bi->setOrient(borient);
      bi->setLocation(bbx, bry);
      bi->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    // Backside: TSV.Y on its own net, tied to the mesh by a straight
    // track-aligned SPECIAL stub up to the CLOSEST grid point -- the H wire at
    // the TSV's own x (already strap-clear) -- with the proxy BTerm there.
    odb::dbNet* bnet = odb::dbNet::create(block_, ("b_sink_" + idx).c_str());
    bnet->setSigType(odb::dbSigType::CLOCK);
    int btx = tp.mx;
    if (odb::dbITerm* yit = ti->findITerm("Y")) {
      yit->connect(bnet);
      const odb::Rect ypad = yit->getBBox();
      const int tx = drawTsvGridStub(bnet, ypad, tp.hy);
      if (tx != INT_MIN) {
        btx = tx;
      }
    }
    odb::dbBTerm* bt = odb::dbBTerm::create(bnet, ("b_sink_" + idx).c_str());
    if (bt) {
      bt->setIoType(odb::dbIoType::INPUT);
      bt->setSigType(odb::dbSigType::CLOCK);
      if (odb::dbBPin* bp = odb::dbBPin::create(bt)) {
        odb::dbBox::create(bp,
                           bterm_layer,
                           btx - half_width,
                           tp.hy - half_width,
                           btx + half_width,
                           tp.hy + half_width);
        bp->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      }
      // Register the tie point (like the drive proxy BTerms do) so
      // convert_mesh_swire breaks the mesh stripe here and the SPICE writer
      // aliases the mesh node at this coord to the b_sink BTerm name --
      // electrically merging the sink stub with the mesh in the deck.
      mesh_connection_points_.insert(
          std::make_tuple(btx, tp.hy, bterm_layer->getRoutingLevel()));
      proxy_alias_[std::make_tuple(btx, tp.hy, h_layer->getRoutingLevel())]
          = "b_sink_" + idx;
      proxy_alias_[std::make_tuple(btx, tp.hy, v_layer->getRoutingLevel())]
          = "b_sink_" + idx;
    }

    // Frontside: TSV.A -> sink-buffer input.
    odb::dbNet* tapn = odb::dbNet::create(block_, ("sink_tap_" + idx).c_str());
    tapn->setSigType(odb::dbSigType::CLOCK);
    odb::dbITerm* ait = ti->findITerm("A");
    odb::dbITerm* bin = bi ? getBufferInputPin(bi) : nullptr;
    if (ait) {
      ait->connect(tapn);
    }
    if (bin) {
      bin->connect(tapn);
    }
    // NOTE: the sink_tap SWire (buffer.A -> TSV.A) is authored LATER, by
    // connectSinkTaps(), which the flow calls AFTER the post-break
    // detailed_placement. Authoring it here would draw to the buffer's
    // pre-legalization position -- the subsequent detailed_placement moves the
    // (esp. wide) sink buffer, leaving the wire dangling at the old spot. So we
    // only make the logical connection now; the physical special wire waits for
    // the buffer's final position.

    // Sink-buffer output -> assigned FFs (moved off the original clock net).
    odb::dbNet* drvn = odb::dbNet::create(block_, ("sink_drv_" + idx).c_str());
    drvn->setSigType(odb::dbSigType::CLOCK);
    if (bi) {
      if (odb::dbITerm* bout = getBufferOutputPin(bi)) {
        bout->connect(drvn);
      }
    }
    for (odb::dbITerm* ff : tap_ffs[t]) {
      ff->disconnect();
      ff->connect(drvn);
      assigned++;
    }
    sn++;
  }

  logger_->info(CMS,
                703,
                "create_sink_taps: placed {} sink-taps (of {} candidates), "
                "{} FFs assigned ({} spilled to non-nearest), "
                "{} TSVs shifted off power straps",
                sn,
                taps.size(),
                assigned,
                spilled,
                strap_shifted);
}

// BPR surgery for every front<->back TSV. GT2N BPR power followpins sit inside
// the clock routing range and run THROUGH the TSV footprint, so each TSV must
// have its two bounding rails broken. For each TSV: (1) a cut window (footprint
// +halo) plus a placement blockage spanning relocate_rows above+below (the
// rails are shared with adjacent rows, so stranded cells must relocate too);
// (2) trim the POWER/GROUND BPR rails over the cut windows, keeping the
// surviving spans; (3) drop BPR segments left with no via feed (electrically
// floating); (4) delete tap cells stranded in the blockages. Caller runs
// detailed_placement after.
void ClockMesh::breakBprAtTsvs(odb::dbTechLayer* bpr_layer,
                               const std::string& tsv_master,
                               const std::string& tap_master,
                               int halo_dbu,
                               int relocate_rows)
{
  if (!block_ || !bpr_layer) {
    logger_->error(CMS, 710, "break_bpr_at_tsvs: missing block or BPR layer");
    return;
  }

  // Row height (relocation band = relocate_rows above + below).
  std::vector<int> rys;
  for (odb::dbRow* r : block_->getRows()) {
    rys.push_back(r->getOrigin().y());
  }
  std::sort(rys.begin(), rys.end());
  rys.erase(std::unique(rys.begin(), rys.end()), rys.end());
  const int rh = rys.size() >= 2 ? rys[1] - rys[0] : 288;
  const int band = relocate_rows * rh;

  auto overlaps = [](int alo, int ahi, int blo, int bhi) {
    return alo < bhi && blo < ahi;
  };

  // (1) per-TSV cut window + relocation blockage.
  struct Cut
  {
    int x0, y0, x1, y1;
  };
  std::vector<Cut> cuts;
  for (odb::dbInst* inst : block_->getInsts()) {
    if (inst->getMaster()->getName() != tsv_master) {
      continue;
    }
    const odb::Rect bb = inst->getBBox()->getBox();
    const int wx0 = bb.xMin() - halo_dbu;
    const int wx1 = bb.xMax() + halo_dbu;
    cuts.push_back({wx0, bb.yMin(), wx1, bb.yMax()});
    odb::dbBlockage::create(
        block_, wx0, bb.yMin() - band, wx1, bb.yMax() + band);
  }

  // (2) trim BPR rails over the cut windows.
  int trimmed = 0;
  for (odb::dbNet* net : block_->getNets()) {
    const odb::dbSigType st = net->getSigType();
    if (st != odb::dbSigType::POWER && st != odb::dbSigType::GROUND) {
      continue;
    }
    for (odb::dbSWire* swire : net->getSWires()) {
      std::vector<odb::dbSBox*> boxes;
      for (odb::dbSBox* sb : swire->getWires()) {
        boxes.push_back(sb);
      }
      for (odb::dbSBox* sb : boxes) {
        if (sb->isVia() || sb->getTechLayer() != bpr_layer) {
          continue;
        }
        const int rxl = sb->xMin(), ryl = sb->yMin();
        const int rxh = sb->xMax(), ryh = sb->yMax();
        std::vector<std::pair<int, int>> rem;
        for (const Cut& c : cuts) {
          if (!overlaps(c.y0, c.y1, ryl, ryh)) {
            continue;
          }
          const int lo = std::max(c.x0, rxl);
          const int hi = std::min(c.x1, rxh);
          if (lo < hi) {
            rem.emplace_back(lo, hi);
          }
        }
        if (rem.empty()) {
          continue;
        }
        std::sort(rem.begin(), rem.end());
        std::vector<std::pair<int, int>> merged;
        for (const auto& iv : rem) {
          if (!merged.empty() && iv.first <= merged.back().second) {
            merged.back().second = std::max(merged.back().second, iv.second);
          } else {
            merged.push_back(iv);
          }
        }
        std::vector<std::pair<int, int>> surv;
        int cur = rxl;
        for (const auto& iv : merged) {
          if (iv.first > cur) {
            surv.emplace_back(cur, iv.first);
          }
          if (iv.second > cur) {
            cur = iv.second;
          }
        }
        if (cur < rxh) {
          surv.emplace_back(cur, rxh);
        }
        const odb::dbWireShapeType wst = sb->getWireShapeType();
        for (const auto& s : surv) {
          odb::dbSBox::create(
              swire, bpr_layer, s.first, ryl, s.second, ryh, wst);
        }
        odb::dbSBox::destroy(sb);
        trimmed++;
      }
    }
  }

  // (3) drop BPR segments with no via feed (electrically floating).
  int nfloat = 0;
  for (odb::dbNet* net : block_->getNets()) {
    const odb::dbSigType st = net->getSigType();
    if (st != odb::dbSigType::POWER && st != odb::dbSigType::GROUND) {
      continue;
    }
    std::vector<odb::Rect> vias;
    for (odb::dbSWire* sw : net->getSWires()) {
      for (odb::dbSBox* sb : sw->getWires()) {
        if (sb->isVia()) {
          vias.emplace_back(sb->xMin(), sb->yMin(), sb->xMax(), sb->yMax());
        }
      }
    }
    std::vector<odb::dbSBox*> floaters;
    for (odb::dbSWire* sw : net->getSWires()) {
      for (odb::dbSBox* sb : sw->getWires()) {
        if (sb->isVia() || sb->getTechLayer() != bpr_layer) {
          continue;
        }
        bool fed = false;
        for (const odb::Rect& v : vias) {
          if (overlaps(sb->xMin(), sb->xMax(), v.xMin(), v.xMax())
              && overlaps(sb->yMin(), sb->yMax(), v.yMin(), v.yMax())) {
            fed = true;
            break;
          }
        }
        if (!fed) {
          floaters.push_back(sb);
        }
      }
    }
    for (odb::dbSBox* sb : floaters) {
      odb::dbBlockage::create(
          block_, sb->xMin(), sb->yMin() - rh, sb->xMax(), sb->yMax() + rh);
      odb::dbSBox::destroy(sb);
      nfloat++;
    }
  }

  // (4) delete tap/endcap cells stranded by the surgery. Two criteria:
  //   (a) the cell overlaps a relocation blockage (in the way of the keepout);
  //   (b) a PG pin of the cell no longer touches any surviving BPR rail --
  //       these are FIXED physical-only cells that detailed_placement cannot
  //       relocate, so an unfed one would sit there permanently unpowered
  //       (endcaps placed by tapcell use the same tap master).
  // Surviving BPR rails per PG net, for the unfed check:
  std::map<odb::dbNet*, std::vector<odb::Rect>, odb::ODBPtrLess> rails;
  for (odb::dbNet* net : block_->getNets()) {
    const odb::dbSigType st = net->getSigType();
    if (st != odb::dbSigType::POWER && st != odb::dbSigType::GROUND) {
      continue;
    }
    for (odb::dbSWire* sw : net->getSWires()) {
      for (odb::dbSBox* sb : sw->getWires()) {
        if (!sb->isVia() && sb->getTechLayer() == bpr_layer) {
          rails[net].push_back(sb->getBox());
        }
      }
    }
  }
  auto rect_overlaps = [&](const odb::Rect& a, const odb::Rect& b) {
    return overlaps(a.xMin(), a.xMax(), b.xMin(), b.xMax())
           && overlaps(a.yMin(), a.yMax(), b.yMin(), b.yMax());
  };
  std::vector<odb::dbInst*> dead;
  int unfed = 0;
  for (odb::dbInst* inst : block_->getInsts()) {
    if (inst->getMaster()->getName() != tap_master) {
      continue;
    }
    // (a) overlaps a blockage
    bool doomed = false;
    const odb::Rect ib = inst->getBBox()->getBox();
    for (odb::dbBlockage* b : block_->getBlockages()) {
      if (rect_overlaps(ib, b->getBBox()->getBox())) {
        doomed = true;
        break;
      }
    }
    // (b) a PG pin with BPR geometry that no surviving rail touches
    if (!doomed) {
      const odb::dbTransform xf = inst->getTransform();
      for (odb::dbITerm* it : inst->getITerms()) {
        odb::dbNet* net = it->getNet();
        if (!net || rails.find(net) == rails.end()) {
          continue;
        }
        bool has_bpr = false;
        bool fed = false;
        for (odb::dbMPin* mp : it->getMTerm()->getMPins()) {
          for (odb::dbBox* g : mp->getGeometry()) {
            if (g->getTechLayer() != bpr_layer) {
              continue;
            }
            has_bpr = true;
            odb::Rect r = g->getBox();
            xf.apply(r);
            for (const odb::Rect& q : rails[net]) {
              if (rect_overlaps(r, q)) {
                fed = true;
                break;
              }
            }
            if (fed) {
              break;
            }
          }
          if (fed) {
            break;
          }
        }
        if (has_bpr && !fed) {
          doomed = true;
          unfed++;
          break;
        }
      }
    }
    if (doomed) {
      dead.push_back(inst);
    }
  }
  for (odb::dbInst* inst : dead) {
    odb::dbInst::destroy(inst);
  }

  logger_->info(CMS,
                711,
                "break_bpr_at_tsvs: {} cut windows, {} BPR rails trimmed, "
                "{} floating stubs dropped, {} stranded taps deleted "
                "({} of them unpowered fixed cells)",
                cuts.size(),
                trimmed,
                nfloat,
                dead.size(),
                unfed);
}

// Author each sink_tap net (buffer.A -> TSV.A) as a SPECIAL wire, AFTER the
// post-break detailed_placement has finalized the sink-buffer positions. The
// flow calls this between detailed_placement and global_route. Doing it here
// (not in createSinkTaps) draws the wire to the buffer's FINAL location -- a
// wide sink buffer legalizes away from its grid TSV, and a wire authored
// earlier dangles at the pre-move spot. The router never touches SWires, so no
// DRT-0206 at any buffer width/distance; convert_mesh_swire later turns these
// into regular Wires for extraction (like the b_sink backside stub).
// Route (LCB tap on the M5-M9 window, off the congested lower metals):
//   buffer.A(M1) -[V1..V5 stack]-> M6 (horizontal, dx) -V5-> M5 (vertical, dy)
//   -[V4..V1 stack]-> TSV.A(M1).
// With use_router=true, no wire is authored at all: the sink_tap nets stay
// ordinary 2-pin nets and GRT/DRT route them (DRC/congestion-aware); the
// SPICE-prep re-author is also skipped so the deck extracts the router's wire.
// (The special-wire path predates the router-mod revert that caused DRT-0206.)
void ClockMesh::connectSinkTaps(bool use_router)
{
  if (!block_) {
    return;
  }
  taps_use_router_ = use_router;
  if (use_router) {
    int nr = 0;
    for (odb::dbNet* net : block_->getNets()) {
      if (net->getName().rfind("sink_tap_", 0) == 0) {
        net->clearSpecial();  // ensure GRT/DRT pick the net up
        nr++;
      }
    }
    logger_->info(CMS,
                  713,
                  "connect_sink_taps: {} sink-tap nets left to the router "
                  "(-use_router)",
                  nr);
    return;
  }
  odb::dbTech* tech = db_->getTech();
  // M1..M6: pins sit on M1; the tap RUNS on M6 (H) / M5 (V).
  odb::dbTechLayer* lyr[6];
  const char* lyr_names[6] = {"M1", "M2", "M3", "M4", "M5", "M6"};
  bool lyr_ok = true;
  for (int i = 0; i < 6; i++) {
    lyr[i] = tech->findLayer(lyr_names[i]);
    lyr_ok = lyr_ok && lyr[i];
  }
  auto find_via
      = [&](odb::dbTechLayer* a, odb::dbTechLayer* b) -> odb::dbTechVia* {
    for (odb::dbTechVia* tv : tech->getVias()) {
      if ((tv->getBottomLayer() == a && tv->getTopLayer() == b)
          || (tv->getBottomLayer() == b && tv->getTopLayer() == a)) {
        return tv;
      }
    }
    return nullptr;
  };
  odb::dbTechVia* vst[5];  // vst[i] = via between M(i+1) and M(i+2)
  for (int i = 0; i < 5; i++) {
    vst[i] = lyr_ok ? find_via(lyr[i], lyr[i + 1]) : nullptr;
    lyr_ok = lyr_ok && vst[i];
  }
  odb::dbTechLayer* l1 = lyr_ok ? lyr[0] : nullptr;
  if (!lyr_ok) {
    logger_->warn(CMS, 707, "connect_sink_taps: M1..M6 layers or vias missing");
    return;
  }
  int n = 0;
  for (odb::dbNet* net : block_->getNets()) {
    if (net->getName().rfind("sink_tap_", 0) != 0) {
      continue;
    }
    odb::dbITerm* bin = nullptr;  // sink-buffer input
    odb::dbITerm* ait = nullptr;  // TSV.A
    for (odb::dbITerm* it : net->getITerms()) {
      if (std::string(it->getInst()->getConstName()).rfind("sink_tsv_", 0)
          == 0) {
        ait = it;
      } else {
        bin = it;
      }
    }
    if (!bin || !ait) {
      continue;
    }
    // Author a CONNECTED regular Wire (not fragmented SBoxes) through BOTH
    // pins, so the extraction walk traces it as REAL per-segment R/C -- no
    // 1mohm island bridge. Drawn at the buffers' FINAL positions; a fully
    // pre-routed net is left untouched by the router (no DRT-0206). Route:
    //   buffer.A(M1) -[V1..V5]-> M6 (H, dx) -V5-> M5 (V, dy) -[V4..V1]->
    //   TSV.A(M1).
    if (odb::dbWire* stale = net->getWire()) {
      odb::dbWire::destroy(stale);
    }
    const odb::Rect bpad = bin->getBBox();
    const odb::Rect tpad = ait->getBBox();
    const int bx = (bpad.xMin() + bpad.xMax()) / 2;
    const int by = (bpad.yMin() + bpad.yMax()) / 2;
    const int tx = (tpad.xMin() + tpad.xMax()) / 2;
    const int ty = (tpad.yMin() + tpad.yMax()) / 2;
    odb::dbWire* w = odb::dbWire::create(net);
    odb::dbWireEncoder enc;
    enc.begin(w);
    enc.newPath(l1, odb::dbWireType::ROUTED);
    enc.addPoint(bx, by);
    enc.addITerm(bin);  // buffer.A
    for (int i = 0; i < 5; i++) {
      enc.addTechVia(vst[i]);  // M1 -> M6 stack
    }
    enc.addPoint(tx, by);    // M6 horizontal (dx)
    enc.addTechVia(vst[4]);  // M6 -> M5
    enc.addPoint(tx, ty);    // M5 vertical (dy)
    for (int i = 3; i >= 0; i--) {
      enc.addTechVia(vst[i]);  // M5 -> M1 stack
    }
    enc.addITerm(ait);  // TSV.A
    enc.end();
    // Mark special so the router leaves it entirely: DRT won't generate guides
    // or try to reroute it (no DRT-0206 even on the longest wires), and OTHER
    // nets treat it as an obstruction (the leaf sink_drv routes around it, so
    // no sink_tap<->sink_drv short). convert_mesh_swire clears this flag so the
    // SPICE walk still extracts the wire's real R/C.
    net->setSpecial();
    n++;
  }
  logger_->info(
      CMS, 706, "connect_sink_taps: authored {} sink-tap special wires", n);
}

// Re-map each FF to its NEAREST sink buffer using FINAL positions. The original
// clustering (createSinkTaps) assigns FFs BEFORE break_bpr_at_tsvs + the final
// detailed_placement, which then displaces cells (worse for a denser mesh: more
// BPR breaks -> more movement). This pass repairs the stale assignments: for
// each FF clock pin, find the nearest sink buffer (by final pin position),
// capacity-capped at F_max, and reconnect if it changed.
void ClockMesh::reassignSinkFFs(int capacity)
{
  if (!block_) {
    return;
  }
  const int f_max = capacity > 0 ? capacity : 80;
  // Sink buffers: one per sink_drv_* net; position = its output-pin location.
  struct Buf
  {
    odb::dbNet* net;
    int x;
    int y;
    int cnt;
  };
  std::vector<Buf> bufs;
  for (odb::dbNet* net : block_->getNets()) {
    if (net->getName().rfind("sink_drv_", 0) != 0) {
      continue;
    }
    int bx = INT_MIN, by = INT_MIN;
    for (odb::dbITerm* it : net->getITerms()) {
      if (it->getIoType() == odb::dbIoType::OUTPUT) {
        const odb::Rect r = it->getBBox();
        bx = (r.xMin() + r.xMax()) / 2;
        by = (r.yMin() + r.yMax()) / 2;
        break;
      }
    }
    if (bx == INT_MIN) {
      continue;
    }
    bufs.push_back({net, bx, by, 0});
  }
  if (bufs.empty()) {
    return;
  }
  // FFs: input pins currently on any sink_drv_* net, with final pin position.
  struct FF
  {
    odb::dbITerm* it;
    odb::dbNet* cur;
    int x;
    int y;
  };
  std::vector<FF> ffs;
  for (Buf& b : bufs) {
    for (odb::dbITerm* it : b.net->getITerms()) {
      if (it->getIoType() != odb::dbIoType::INPUT) {
        continue;
      }
      const odb::Rect r = it->getBBox();
      ffs.push_back(
          {it, b.net, (r.xMin() + r.xMax()) / 2, (r.yMin() + r.yMax()) / 2});
    }
  }
  // Greedy nearest-with-capacity; if all buffers full, take the nearest anyway.
  int moved = 0;
  for (FF& f : ffs) {
    long long best = LLONG_MAX, best_any = LLONG_MAX;
    int bi = -1, bi_any = -1;
    for (int i = 0; i < (int) bufs.size(); i++) {
      const long long dx = bufs[i].x - f.x, dy = bufs[i].y - f.y;
      const long long d = dx * dx + dy * dy;
      if (d < best_any) {
        best_any = d;
        bi_any = i;
      }
      if (bufs[i].cnt < f_max && d < best) {
        best = d;
        bi = i;
      }
    }
    if (bi < 0) {
      bi = bi_any;  // all full -> overflow into nearest
    }
    bufs[bi].cnt++;
    if (bufs[bi].net != f.cur) {
      f.it->disconnect();
      f.it->connect(bufs[bi].net);
      moved++;
    }
  }
  logger_->info(CMS,
                712,
                "reassign_sink_ffs: re-mapped {} of {} FFs to nearest sink "
                "buffer (post-placement, {} buffers, cap {})",
                moved,
                ffs.size(),
                bufs.size(),
                f_max);
}

// BTERMs are placed directly on mesh layers — no via stacks needed
void ClockMesh::connectProxyBTermsToMesh(const std::string& clock_name)
{
  int buffer_bterms = 0;
  for (const GridIntersection& inter : grid_intersections_) {
    if (inter.has_buffer && inter.proxy_bterm) {
      buffer_bterms++;
    }
  }

  int sink_bterms = 0;
  for (odb::dbNet* net : block_->getNets()) {
    std::string net_name = net->getName();
    if (net_name.rfind("sink_", 0) == 0) {
      sink_bterms++;
    }
  }

  logger_->info(
      CMS,
      625,
      "BTERMs on mesh layers: {} buffer, {} sink (no via stacks needed)",
      buffer_bterms,
      sink_bterms);
}

// Runs TritonCTS to build clock tree to buffer inputs
void ClockMesh::buildCtsTreeToBuffers(
    const std::string& tree_net_name,
    const std::vector<std::string>& buffer_list)
{
  cts::TritonCTS* triton_cts = openroad_->getTritonCts();
  if (!triton_cts) {
    logger_->error(CMS, 400, "TritonCTS not available");
    return;
  }

  int result = triton_cts->setClockNets(tree_net_name.c_str());
  if (result != 0) {
    logger_->warn(
        CMS, 401, "Failed to set clock net '{}' for CTS", tree_net_name);
    return;
  }

  // Build the buffer list string. Empty string triggers TritonCTS
  // auto-inference.
  std::string clkbuf_list;
  for (const auto& buf : buffer_list) {
    if (!clkbuf_list.empty()) {
      clkbuf_list += " ";
    }
    clkbuf_list += buf;
  }
  triton_cts->setBufferList(clkbuf_list.c_str());

  triton_cts->runTritonCts();
}

// Re-encodes wire segments from source net onto the mesh net's wire
void ClockMesh::reencodeWireToMesh(odb::dbWire* src_wire,
                                   odb::dbWireEncoder& encoder)
{
  odb::dbWireDecoder decoder;
  decoder.begin(src_wire);
  std::map<int, int> jct_map;

  odb::dbWireDecoder::OpCode opcode;
  while ((opcode = decoder.next()) != odb::dbWireDecoder::END_DECODE) {
    switch (opcode) {
      case odb::dbWireDecoder::PATH: {
        odb::dbTechLayer* layer = decoder.getLayer();
        odb::dbWireType wire_type = decoder.getWireType();
        if (decoder.peek() == odb::dbWireDecoder::RULE) {
          decoder.next();
          encoder.newPath(layer, wire_type, decoder.getRule());
        } else {
          encoder.newPath(layer, wire_type);
        }
        break;
      }
      case odb::dbWireDecoder::JUNCTION: {
        int src_jct = decoder.getJunctionValue();
        odb::dbWireType wire_type = decoder.getWireType();
        auto it = jct_map.find(src_jct);
        if (it != jct_map.end()) {
          if (decoder.peek() == odb::dbWireDecoder::RULE) {
            decoder.next();
            encoder.newPath(it->second, wire_type, decoder.getRule());
          } else {
            encoder.newPath(it->second, wire_type);
          }
        } else {
          odb::dbTechLayer* layer = decoder.getLayer();
          logger_->warn(CMS,
                        873,
                        "Junction ID {} not found in source wire, "
                        "starting disjoint path on layer {}",
                        src_jct,
                        layer->getName());
          encoder.newPath(layer, wire_type);
        }
        break;
      }
      case odb::dbWireDecoder::SHORT: {
        int src_jct = decoder.getJunctionValue();
        odb::dbTechLayer* layer = decoder.getLayer();
        odb::dbWireType wire_type = decoder.getWireType();
        auto it = jct_map.find(src_jct);
        if (it != jct_map.end()) {
          if (decoder.peek() == odb::dbWireDecoder::RULE) {
            decoder.next();
            encoder.newPathShort(
                it->second, layer, wire_type, decoder.getRule());
          } else {
            encoder.newPathShort(it->second, layer, wire_type);
          }
        } else {
          logger_->warn(CMS,
                        874,
                        "Short junction ID {} not found, "
                        "starting disjoint path on layer {}",
                        src_jct,
                        layer->getName());
          encoder.newPath(layer, wire_type);
        }
        break;
      }
      case odb::dbWireDecoder::VWIRE: {
        int src_jct = decoder.getJunctionValue();
        odb::dbTechLayer* layer = decoder.getLayer();
        odb::dbWireType wire_type = decoder.getWireType();
        auto it = jct_map.find(src_jct);
        if (it != jct_map.end()) {
          encoder.newPathVirtualWire(it->second, layer, wire_type);
        } else {
          logger_->warn(CMS,
                        875,
                        "VWire junction ID {} not found, "
                        "starting disjoint path on layer {}",
                        src_jct,
                        layer->getName());
          encoder.newPath(layer, wire_type);
        }
        break;
      }
      case odb::dbWireDecoder::POINT: {
        int x, y;
        decoder.getPoint(x, y);
        int new_jct = encoder.addPoint(x, y);
        jct_map[decoder.getJunctionId()] = new_jct;
        break;
      }
      case odb::dbWireDecoder::POINT_EXT: {
        int x, y, ext;
        decoder.getPoint(x, y, ext);
        int new_jct = encoder.addPoint(x, y, ext);
        jct_map[decoder.getJunctionId()] = new_jct;
        break;
      }
      case odb::dbWireDecoder::VIA: {
        odb::dbVia* via = decoder.getVia();
        int new_jct = encoder.addVia(via);
        jct_map[decoder.getJunctionId()] = new_jct;
        break;
      }
      case odb::dbWireDecoder::TECH_VIA: {
        odb::dbTechVia* via = decoder.getTechVia();
        int new_jct = encoder.addTechVia(via);
        jct_map[decoder.getJunctionId()] = new_jct;
        break;
      }
      case odb::dbWireDecoder::RECT: {
        int dx1, dy1, dx2, dy2;
        decoder.getRect(dx1, dy1, dx2, dy2);
        encoder.addRect(dx1, dy1, dx2, dy2);
        break;
      }
      case odb::dbWireDecoder::RULE:
        break;
      case odb::dbWireDecoder::ITERM:
      case odb::dbWireDecoder::BTERM:
        break;
      default:
        break;
    }
  }
}

// Converts mesh grid SWires to regular dbWire so OpenRCX can extract parasitics
void ClockMesh::convertSWireToWire(const std::string& clock_name)
{
  std::string base_name = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
  std::string mesh_net_name = base_name + "_mesh";

  odb::dbNet* mesh_net = block_->findNet(mesh_net_name.c_str());
  if (!mesh_net) {
    logger_->error(CMS, 850, "Mesh net '{}' not found", mesh_net_name);
    return;
  }

  auto swires = mesh_net->getSWires();
  if (swires.empty()) {
    logger_->warn(CMS, 851, "No SWires found on net '{}'", mesh_net_name);
    return;
  }

  // Collect wire rectangles and vias from SBoxes
  struct WireRect
  {
    odb::dbTechLayer* layer;
    odb::Rect rect;
    bool is_horizontal;
  };
  struct ViaEntry
  {
    odb::dbTechVia* tech_via;
    int x;
    int y;
  };

  std::vector<WireRect> wire_rects;
  std::vector<ViaEntry> via_entries;

  for (odb::dbSWire* swire : swires) {
    for (odb::dbSBox* sbox : swire->getWires()) {
      if (sbox->isVia()) {
        odb::dbTechVia* tv = sbox->getTechVia();
        if (tv) {
          int cx = (sbox->xMin() + sbox->xMax()) / 2;
          int cy = (sbox->yMin() + sbox->yMax()) / 2;
          via_entries.push_back({tv, cx, cy});
        }
      } else {
        odb::dbTechLayer* layer = sbox->getTechLayer();
        if (!layer) {
          continue;
        }
        odb::Rect rect(sbox->xMin(), sbox->yMin(), sbox->xMax(), sbox->yMax());
        bool is_horiz = (rect.dx() > rect.dy());
        wire_rects.push_back({layer, rect, is_horiz});
      }
    }
  }

  // Append mesh geometry to the existing dbWire (which already holds the
  // routed clk_buf_* and sink_* paths after mergeNetsToMesh).  We rely on
  // OpenRCX's same-layer geometric coincidence merging to tie routed
  // wires to the mesh: each mesh wire is one continuous newPath; vias are
  // standalone newPaths; H-wire / V-wire / via shapes physically overlap
  // at every intersection on both layers, and OpenRCX collapses overlapping
  // wire shapes on the same layer into one RC node during extraction.
  //
  // This is the simplest encoding that worked on dense aes_block (0 fails).
  // The connectivity-aware sink filter in connectSinksViaRouter handles the
  // sparse case where the mesh topology itself splits into sub-meshes.
  odb::dbWire* wire = mesh_net->getWire();
  if (!wire) {
    wire = odb::dbWire::create(mesh_net);
  }

  odb::dbWireEncoder encoder;
  encoder.append(wire);

  for (const auto& wr : wire_rects) {
    int cx_start, cy_start, cx_end, cy_end;
    if (wr.is_horizontal) {
      int cy = (wr.rect.yMin() + wr.rect.yMax()) / 2;
      cx_start = wr.rect.xMin();
      cx_end = wr.rect.xMax();
      cy_start = cy_end = cy;
    } else {
      int cx = (wr.rect.xMin() + wr.rect.xMax()) / 2;
      cx_start = cx_end = cx;
      cy_start = wr.rect.yMin();
      cy_end = wr.rect.yMax();
    }

    // Insert intermediate POINTs at every planned connection point on this
    // wire so OpenRCX has explicit RC nodes there (proxy BTerms / sink BTerms
    // sit on top of these).
    int level = wr.layer->getRoutingLevel();
    std::vector<int> break_coords;
    for (const auto& [px, py, plevel] : mesh_connection_points_) {
      if (plevel != level) {
        continue;
      }
      if (wr.is_horizontal) {
        if (py == cy_start && px > cx_start && px < cx_end) {
          break_coords.push_back(px);
        }
      } else {
        if (px == cx_start && py > cy_start && py < cy_end) {
          break_coords.push_back(py);
        }
      }
    }
    std::sort(break_coords.begin(), break_coords.end());

    encoder.newPath(wr.layer, odb::dbWireType::ROUTED);
    encoder.addPoint(cx_start, cy_start);
    for (int c : break_coords) {
      if (wr.is_horizontal) {
        encoder.addPoint(c, cy_start);
      } else {
        encoder.addPoint(cx_start, c);
      }
    }
    encoder.addPoint(cx_end, cy_end);
  }

  // Vias as standalone paths.  OpenRCX merges the via's bottom-layer point
  // with the H-wire's bottom-layer rectangle and the via's top-layer point
  // with the V-wire's top-layer rectangle (both physically overlap at the
  // grid intersection coord on the same layer).
  for (const auto& ve : via_entries) {
    odb::dbTechLayer* bot_layer = ve.tech_via->getBottomLayer();
    encoder.newPath(bot_layer, odb::dbWireType::ROUTED);
    encoder.addPoint(ve.x, ve.y);
    encoder.addTechVia(ve.tech_via);
  }

  encoder.end();

  // Delete the SWires after conversion
  std::vector<odb::dbSWire*> swires_to_delete;
  for (odb::dbSWire* swire : mesh_net->getSWires()) {
    swires_to_delete.push_back(swire);
  }
  for (odb::dbSWire* swire : swires_to_delete) {
    odb::dbSWire::destroy(swire);
  }

  // Clear the special flag so OpenRCX treats it as a regular net
  mesh_net->clearSpecial();

  logger_->info(
      CMS,
      852,
      "Converted {} wire segments and {} vias from SWire to Wire on '{}'",
      wire_rects.size(),
      via_entries.size(),
      mesh_net_name);

  // Also convert the backside TSV stub nets (b_<clk>_buf_*, b_sink_*): each is
  // one BM1 stripe + a BV1 via drawn by drawTsvGridStub as special wire.
  // Converting them (and clearing the special flag) lets OpenRCX extract their
  // parasitics exactly like the mesh.
  const std::string bbuf_prefix = "b_" + base_name + "_buf_";
  int stub_nets = 0;
  for (odb::dbNet* n : block_->getNets()) {
    const std::string nm = n->getName();
    if (nm.rfind(bbuf_prefix, 0) != 0 && nm.rfind("b_sink_", 0) != 0) {
      continue;
    }
    std::vector<odb::dbSWire*> nsw;
    for (odb::dbSWire* sw : n->getSWires()) {
      nsw.push_back(sw);
    }
    if (nsw.empty()) {
      continue;
    }
    odb::dbWire* w = n->getWire();
    if (!w) {
      w = odb::dbWire::create(n);
    }
    odb::dbWireEncoder enc;
    enc.begin(w);
    for (odb::dbSWire* sw : nsw) {
      for (odb::dbSBox* sb : sw->getWires()) {
        if (sb->isVia()) {
          if (odb::dbTechVia* tv = sb->getTechVia()) {
            enc.newPath(tv->getBottomLayer(), odb::dbWireType::ROUTED);
            enc.addPoint((sb->xMin() + sb->xMax()) / 2,
                         (sb->yMin() + sb->yMax()) / 2);
            enc.addTechVia(tv);
          }
        } else {
          odb::dbTechLayer* L = sb->getTechLayer();
          const int hw = L->getWidth() / 2;
          enc.newPath(L, odb::dbWireType::ROUTED);
          if (sb->yMax() - sb->yMin() >= sb->xMax() - sb->xMin()) {
            const int cx = (sb->xMin() + sb->xMax()) / 2;
            enc.addPoint(cx, sb->yMin() + hw);
            enc.addPoint(cx, sb->yMax() - hw);
          } else {
            const int cy = (sb->yMin() + sb->yMax()) / 2;
            enc.addPoint(sb->xMin() + hw, cy);
            enc.addPoint(sb->xMax() - hw, cy);
          }
        }
      }
    }
    enc.end();
    for (odb::dbSWire* sw : nsw) {
      odb::dbSWire::destroy(sw);
    }
    n->clearSpecial();
    stub_nets++;
  }
  if (stub_nets > 0) {
    logger_->info(CMS,
                  896,
                  "Converted {} backside stub nets to regular wire for "
                  "extraction",
                  stub_nets);
  }

  // (Re)author the sink_tap buffer.A -> TSV.A wire HERE, after estimate_
  // parasitics (which wipes the pre-route connect_sink_taps drew before
  // routing). This is the copy that survives into the deck. A single CONNECTED
  // path through both iterms so the extraction walk traces real per-segment
  // R/C (no island bridge). Route (both pins M1; tap runs on M5-M9 window):
  //   buffer.A -[V1..V5]-> M6 (H, dx) -V5-> M5 (V, dy) -[V4..V1]-> TSV.A.
  // Skipped entirely with connect_sink_taps -use_router: the taps were detail-
  // routed like any net, and destroying the router's wire here would replace a
  // DRC-clean route with a hand-drawn one.
  odb::dbTech* tap_tech = db_->getTech();
  odb::dbTechLayer* lyr[6];
  const char* lyr_names[6] = {"M1", "M2", "M3", "M4", "M5", "M6"};
  bool lyr_ok = true;
  for (int i = 0; i < 6; i++) {
    lyr[i] = tap_tech->findLayer(lyr_names[i]);
    lyr_ok = lyr_ok && lyr[i];
  }
  auto find_via
      = [&](odb::dbTechLayer* a, odb::dbTechLayer* b) -> odb::dbTechVia* {
    for (odb::dbTechVia* tv : tap_tech->getVias()) {
      if ((tv->getBottomLayer() == a && tv->getTopLayer() == b)
          || (tv->getBottomLayer() == b && tv->getTopLayer() == a)) {
        return tv;
      }
    }
    return nullptr;
  };
  odb::dbTechVia* vst[5];  // vst[i] = via between M(i+1) and M(i+2)
  for (int i = 0; i < 5; i++) {
    vst[i] = lyr_ok ? find_via(lyr[i], lyr[i + 1]) : nullptr;
    lyr_ok = lyr_ok && vst[i];
  }
  odb::dbTechLayer* l1 = lyr_ok ? lyr[0] : nullptr;
  int tap_wires = 0;
  if (lyr_ok && !taps_use_router_) {
    for (odb::dbNet* n : block_->getNets()) {
      if (n->getName().rfind("sink_tap_", 0) != 0) {
        continue;
      }
      n->clearSpecial();
      odb::dbITerm* bin = nullptr;
      odb::dbITerm* ait = nullptr;
      for (odb::dbITerm* it : n->getITerms()) {
        if (std::string(it->getInst()->getConstName()).rfind("sink_tsv_", 0)
            == 0) {
          ait = it;
        } else {
          bin = it;
        }
      }
      if (!bin || !ait) {
        continue;
      }
      for (odb::dbSWire* sw : std::vector<odb::dbSWire*>(
               n->getSWires().begin(), n->getSWires().end())) {
        odb::dbSWire::destroy(sw);
      }
      if (odb::dbWire* stale = n->getWire()) {
        odb::dbWire::destroy(stale);
      }
      const odb::Rect bp = bin->getBBox();
      const odb::Rect tp = ait->getBBox();
      const int bx = (bp.xMin() + bp.xMax()) / 2;
      const int by = (bp.yMin() + bp.yMax()) / 2;
      const int tx = (tp.xMin() + tp.xMax()) / 2;
      const int ty = (tp.yMin() + tp.yMax()) / 2;
      odb::dbWire* w = odb::dbWire::create(n);
      odb::dbWireEncoder enc;
      enc.begin(w);
      enc.newPath(l1, odb::dbWireType::ROUTED);
      enc.addPoint(bx, by);
      enc.addITerm(bin);
      for (int i = 0; i < 5; i++) {
        enc.addTechVia(vst[i]);  // M1 -> M6 stack
      }
      enc.addPoint(tx, by);    // M6 horizontal (dx)
      enc.addTechVia(vst[4]);  // M6 -> M5
      enc.addPoint(tx, ty);    // M5 vertical (dy)
      for (int i = 3; i >= 0; i--) {
        enc.addTechVia(vst[i]);  // M5 -> M1 stack
      }
      enc.addITerm(ait);
      enc.end();
      tap_wires++;
    }
  }
  if (tap_wires > 0) {
    logger_->info(
        CMS, 708, "Re-authored {} sink_tap wires (real R/C)", tap_wires);
  }
}

void ClockMesh::captureLeafArrivals(const std::string& clock_name)
{
  leaf_arrivals_ns_.clear();
  leaf_slews_ns_.clear();

  // Ensure STA timing is up-to-date after CTS modifications
  sta_->updateTiming(false);

  for (const GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.buffer_inst) {
      continue;
    }
    odb::dbITerm* input_iterm = getBufferInputPin(inter.buffer_inst);
    if (!input_iterm || !input_iterm->getNet()) {
      continue;
    }
    // Key by input pin so writeMeshSpice can match Vclk source → buffer input
    std::string leaf_key
        = std::string(inter.buffer_inst->getConstName()) + "/"
          + std::string(input_iterm->getMTerm()->getConstName());
    if (leaf_arrivals_ns_.count(leaf_key)) {
      continue;
    }
    // Capture arrival at the INPUT pin (excludes the buffer's input→output
    // cell-delay arc). The per-buffer Vclk source then represents "when the
    // CTS signal reaches the mesh buffer input"; the real buffer subcircuit
    // in the SPICE netlist propagates the cell delay, so capturing at the
    // output would double-count it.
    sta::Pin* arrival_pin = network_->dbToSta(input_iterm);
    const char* arrival_pin_name = input_iterm->getMTerm()->getConstName();
    if (arrival_pin) {
      float arrival_sec = sta_->arrival(
          arrival_pin, sta::RiseFallBoth::rise(), sta::MinMax::max());
      leaf_arrivals_ns_[leaf_key] = arrival_sec * 1e9;
      // Per-buffer rise slew at the input pin — what the CTS feeder
      // delivers to this buffer. The fall slew is typically symmetric;
      // we cache rise only and reuse it for fall.
      sta::Vertex* vertex = sta_->graph()->pinLoadVertex(arrival_pin);
      float slew_sec = 0.0f;
      if (vertex) {
        slew_sec = sta_->slew(vertex,
                              sta::RiseFallBoth::rise(),
                              sta_->scenes(),
                              sta::MinMax::max());
      }
      leaf_slews_ns_[leaf_key] = slew_sec * 1e9;
      logger_->info(
          CMS,
          873,
          "Buffer {} arrival at pin {} on net '{}': {:.6g} ns, slew {:.6g} ns",
          inter.buffer_inst->getConstName(),
          arrival_pin_name,
          input_iterm->getNet()->getConstName(),
          arrival_sec * 1e9,
          slew_sec * 1e9);
    } else {
      leaf_arrivals_ns_[leaf_key] = 0.0;
      leaf_slews_ns_[leaf_key] = 0.0;
    }
  }

  // float min_arr = std::numeric_limits<float>::max();
  // float max_arr = std::numeric_limits<float>::lowest();
  // for (const auto& [name, arr] : leaf_arrivals_ns_) {
  //   min_arr = std::min(min_arr, arr);
  //   max_arr = std::max(max_arr, arr);
  // }
  // if (!leaf_arrivals_ns_.empty()) {
  //   logger_->info(CMS, 867,
  //                 "Captured {} CTS leaf arrivals, range: [{:.4g}, {:.4g}] ns,
  //                 " "CTS skew: {:.4g} ns", leaf_arrivals_ns_.size(), min_arr,
  //                 max_arr, max_arr - min_arr);
  // } else {
  //   logger_->warn(CMS, 869, "No CTS leaf arrivals captured");
  // }
}

// Reads extracted parasitics from DB and writes SPICE netlist
void ClockMesh::writeMeshSpice(const std::string& clock_name,
                               const std::string& spice_file,
                               float vdd_voltage,
                               float rise_time_ns,
                               float fall_time_ns,
                               const std::vector<std::string>& spice_models,
                               bool zero_delay,
                               bool full_tree,
                               bool finfet,
                               float tsv_res)
{
  std::string base_name = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
  std::string mesh_net_name = base_name + "_mesh";

  odb::dbNet* mesh_net = block_->findNet(mesh_net_name.c_str());
  if (!mesh_net) {
    logger_->error(CMS, 860, "Mesh net '{}' not found", mesh_net_name);
    return;
  }

  std::ofstream out(spice_file);
  if (!out.is_open()) {
    logger_->error(CMS, 861, "Cannot open SPICE file: {}", spice_file);
    return;
  }

  // Auto-detect VDD voltage from Liberty operating conditions
  float vdd = vdd_voltage;
  if (vdd == 0.0) {
    sta::LibertyLibrary* lib = network_->defaultLibertyLibrary();
    if (lib) {
      const sta::OperatingConditions* op_cond
          = lib->defaultOperatingConditions();
      if (op_cond) {
        vdd = op_cond->voltage();
      }
    }
    if (vdd == 0.0) {
      vdd = 1.8;
      logger_->warn(CMS,
                    865,
                    "Could not detect VDD from Liberty, using default {}V",
                    vdd);
    }
  }

  // Get clock period from SDC (STA stores time in seconds)
  float period_ns = 10.0;  // fallback default
  sta::Sdc* sdc = sta_->cmdSdc();
  if (sdc) {
    for (auto clk : sdc->clocks()) {
      if (std::string(clk->name()) == clock_name) {
        period_ns = clk->period() * 1e9;  // seconds → nanoseconds
        break;
      }
    }
  }
  float pw_ns = period_ns / 2.0;
  float rise_ns = (rise_time_ns > 0.0) ? rise_time_ns : period_ns / 100.0;
  float fall_ns = (fall_time_ns > 0.0) ? fall_time_ns : period_ns / 100.0;

  logger_->info(CMS,
                866,
                "SPICE parameters: VDD={:.3g}V, period={:.4g}ns, "
                "rise={:.4g}ns, fall={:.4g}ns",
                vdd,
                period_ns,
                rise_ns,
                fall_ns);

  // Characters like $, [, ], . are invalid in SPICE node names
  auto sanitize = [](const std::string& name) -> std::string {
    std::string s = name;
    for (char& c : s) {
      if (c == '$' || c == '[' || c == ']' || c == '.' || c == '/' || c == '\\'
          || c == ' ') {
        c = '_';
      }
    }
    return s;
  };

  // Build junction-id → (x, y, layer-level) map for the mesh net's dbWire.
  // Used to alias mesh-net internal nodes at proxy-bterm positions to the
  // bterm names so sub-net stubs electrically merge with mesh stripes when
  // merge_mesh_nets was skipped.
  std::map<int, std::tuple<int, int, int>> mesh_jid_to_coord;
  if (odb::dbWire* mw = mesh_net->getWire()) {
    odb::dbWireDecoder mdec;
    mdec.begin(mw);
    odb::dbTechLayer* cur_layer = nullptr;
    odb::dbWireDecoder::OpCode op;
    while ((op = mdec.next()) != odb::dbWireDecoder::END_DECODE) {
      if (op == odb::dbWireDecoder::PATH || op == odb::dbWireDecoder::JUNCTION
          || op == odb::dbWireDecoder::SHORT) {
        cur_layer = mdec.getLayer();
      } else if (op == odb::dbWireDecoder::POINT
                 || op == odb::dbWireDecoder::POINT_EXT) {
        int x, y;
        if (op == odb::dbWireDecoder::POINT) {
          mdec.getPoint(x, y);
        } else {
          int ext;
          mdec.getPoint(x, y, ext);
        }
        if (cur_layer) {
          mesh_jid_to_coord[mdec.getJunctionId()]
              = std::make_tuple(x, y, cur_layer->getRoutingLevel());
        }
      }
    }
  }

  auto node_name = [&](odb::dbCapNode* cap_node) -> std::string {
    if (cap_node->isITerm()) {
      odb::dbITerm* iterm = odb::dbITerm::getITerm(block_, cap_node->getNode());
      if (iterm) {
        return sanitize(std::string(iterm->getInst()->getConstName()) + "_"
                        + std::string(iterm->getMTerm()->getConstName()));
      }
    }
    if (cap_node->isBTerm()) {
      odb::dbBTerm* bterm = odb::dbBTerm::getBTerm(block_, cap_node->getNode());
      if (bterm) {
        return sanitize(std::string(bterm->getConstName()));
      }
    }
    // If this CapNode belongs to the mesh net AND its junction sits at a
    // proxy-bterm position, return the bterm name so we alias to sub-nets.
    if (cap_node->getNet() == mesh_net) {
      auto coord_it = mesh_jid_to_coord.find(cap_node->getNode());
      if (coord_it != mesh_jid_to_coord.end()) {
        auto alias_it = proxy_alias_.find(coord_it->second);
        if (alias_it != proxy_alias_.end()) {
          return sanitize(alias_it->second);
        }
      }
    }
    uint32_t nid = cap_node->getNet()->getId();
    return "n_" + std::to_string(nid) + "_"
           + std::to_string(cap_node->getNode());
  };

  // Parse CDL/SPICE model files for subcircuit pin order
  std::map<std::string, std::vector<std::string>> subckt_pins;
  for (const auto& model_file : spice_models) {
    std::ifstream model_in(model_file);
    if (!model_in.is_open()) {
      logger_->warn(CMS, 872, "Cannot open SPICE model file: {}", model_file);
      continue;
    }
    std::string line;
    while (std::getline(model_in, line)) {
      // Match .SUBCKT or .subckt
      if (line.size() > 7
          && (line.compare(0, 7, ".SUBCKT") == 0
              || line.compare(0, 7, ".subckt") == 0)) {
        std::istringstream iss(line);
        std::string token, subckt_name;
        iss >> token >> subckt_name;  // skip ".SUBCKT", get name
        std::vector<std::string> pins;
        while (iss >> token) {
          pins.push_back(token);
        }
        subckt_pins[subckt_name] = pins;
      }
    }
    model_in.close();
  }

  // Header
  out << "* SPICE netlist for clock mesh net: " << mesh_net_name << "\n";
  out << "* Generated by OpenROAD ClockMesh (VLSIDA Lab UCSC) \n";
  out << "*\n";

  // Simulator options for convergence with extracted RC networks
  out << ".option rshunt=1e12\n";
  out << ".option method=gear\n";
  out << ".option abstol=1e-10 reltol=0.003 vntol=1e-4\n";
  out << ".option delmax=10p\n";
  out << ".option autostop\n";

  // Disable Monte Carlo mismatch for deterministic simulation
  if (!spice_models.empty()) {
    out << ".param mc_mm_switch=0\n";
    out << ".param mc_pr_switch=0\n";
  }

  // Include SPICE model/subcircuit files
  for (const auto& model_file : spice_models) {
    if (model_file.size() > 5
        && model_file.compare(model_file.size() - 5, 5, ".osdi") == 0) {
      out << ".pre_osdi " << model_file << "\n";
    } else {
      out << ".include " << model_file << "\n";
    }
  }

  // Power supplies (GND = node 0, no separate Vgnd source needed)
  out << "\n* Power supplies\n";
  out << "Vvdd VDD 0 " << vdd << "\n";

  // Ensure every mesh buffer has an arrival entry. Captured arrivals come
  // from capture_mesh_arrivals; any buffer missing from that map (partial
  // capture, multi-clock domains, post-capture buffer additions) gets a
  // zero-delay fallback so its SPICE input source is still written.
  const bool capture_was_empty = leaf_arrivals_ns_.empty();
  int filled_zero = 0;
  for (const GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.buffer_inst) {
      continue;
    }
    odb::dbITerm* input_iterm = getBufferInputPin(inter.buffer_inst);
    if (input_iterm && input_iterm->getNet()) {
      std::string leaf_key
          = std::string(inter.buffer_inst->getConstName()) + "/"
            + std::string(input_iterm->getMTerm()->getConstName());
      if (!leaf_arrivals_ns_.count(leaf_key)) {
        leaf_arrivals_ns_[leaf_key] = 0.0;
        filled_zero++;
      }
    }
  }
  if (capture_was_empty) {
    logger_->warn(CMS,
                  870,
                  "No cached CTS leaf arrivals. Call capture_mesh_arrivals "
                  "before merge_mesh_nets for accurate skew analysis. Using "
                  "zero delays for all {} buffers.",
                  filled_zero);
  } else if (filled_zero > 0) {
    logger_->warn(CMS,
                  871,
                  "Partial CTS leaf arrivals: {} buffers were missing from the "
                  "captured map and assigned zero delay. Skew numbers for "
                  "those buffers' downstream sinks will be optimistic.",
                  filled_zero);
  }

  // Walk back through the upstream CTS clock tree when full_tree=true.
  // This collects driver nets / driver instances / root bterms that need to
  // be in the SPICE deck so the upstream tree is fully simulated (root PWL
  // -> root inverter -> ... -> mesh buffer inputs) rather than abstracted
  // as per-buffer Vclk PULSE sources.
  std::set<odb::dbNet*, odb::ODBPtrLess> tree_nets;
  std::set<odb::dbInst*, odb::ODBPtrLess> tree_buffers;
  std::set<odb::dbBTerm*, odb::ODBPtrLess> root_bterms;
  if (full_tree) {
    std::vector<odb::dbInst*> worklist;
    for (const GridIntersection& inter : grid_intersections_) {
      if (!inter.has_buffer || !inter.buffer_inst) {
        continue;
      }
      worklist.push_back(inter.buffer_inst);
    }
    while (!worklist.empty()) {
      odb::dbInst* inst = worklist.back();
      worklist.pop_back();
      for (odb::dbITerm* it : inst->getITerms()) {
        if (it->getIoType() != odb::dbIoType::INPUT) {
          continue;
        }
        // Check the NET's sig type (cell pin MTerm is often SIGNAL even for
        // clock cells), and exclude power/ground nets.
        odb::dbNet* drv_net = it->getNet();
        if (!drv_net || drv_net == mesh_net) {
          continue;
        }
        odb::dbSigType nst = drv_net->getSigType();
        if (nst == odb::dbSigType::POWER || nst == odb::dbSigType::GROUND) {
          continue;
        }
        if (!tree_nets.insert(drv_net).second) {
          continue;
        }
        for (odb::dbITerm* dit : drv_net->getITerms()) {
          if (dit->getIoType() == odb::dbIoType::OUTPUT) {
            odb::dbInst* drv_inst = dit->getInst();
            if (drv_inst && tree_buffers.insert(drv_inst).second) {
              worklist.push_back(drv_inst);
            }
          }
        }
        for (odb::dbBTerm* bt : drv_net->getBTerms()) {
          if (bt->getIoType() == odb::dbIoType::INPUT) {
            root_bterms.insert(bt);
          }
        }
      }
    }
    logger_->info(CMS,
                  893,
                  "full_tree: walked back {} tree nets, {} tree buffer "
                  "instances, {} root bterm(s)",
                  tree_nets.size(),
                  tree_buffers.size(),
                  root_bterms.size());
  }

  // If zero_delay requested, build a zeroed copy of arrivals (leaves original
  // intact)
  std::map<std::string, float> arrivals_for_spice;
  if (zero_delay) {
    for (const auto& [k, v] : leaf_arrivals_ns_) {
      arrivals_for_spice[k] = 0.0f;
    }
    logger_->info(CMS,
                  872,
                  "zero_delay mode: all {} mesh buffer arrival times set to 0.",
                  arrivals_for_spice.size());
    out << "\n* CTS buffer clock sources (zero-delay mode: all arrivals forced "
           "to 0)\n";
  } else {
    arrivals_for_spice = leaf_arrivals_ns_;
    out << "\n* CTS buffer clock sources (with per-pin STA arrival delays)\n";
  }

  // Write clock source(s).
  // - full_tree=false: per-buffer Vclk PULSE at each mesh-buffer A pin with
  //   STA-captured arrival + slew baked in.
  // - full_tree=true:  one Vclk PULSE at the root clock bterm, no per-buffer
  //   sources; the tree propagates the signal to mesh buffer inputs.
  const bool user_set_slew = (rise_time_ns > 0.0f) || (fall_time_ns > 0.0f);
  std::map<uint32_t, std::string> buf_input_spice_node;
  // Mesh-buffer input nodes (feeder side) — for a 10%-90% feeder-slew measure,
  // measured the SAME way as the sink slew so the two are directly comparable
  // (shows whether the mesh regenerates a degraded feeder edge).
  std::vector<std::string> feeder_input_nodes;
  int vclk_count = 0;
  if (full_tree) {
    // Single root Vclk
    for (odb::dbBTerm* bt : root_bterms) {
      std::string root_node = sanitize(bt->getConstName());
      out << "Vclk_root_" << vclk_count << " " << root_node << " 0 PULSE(0 "
          << vdd << " 0n " << rise_ns << "n " << fall_ns << "n " << pw_ns
          << "n " << period_ns << "n)" << " $ root bterm=" << bt->getConstName()
          << "\n";
      vclk_count++;
    }
    // No per-buffer Vclk; buffer input pins are driven by the tree.
    // Register mesh-driver input nodes for the feeder-slew measures below.
    // In full_tree mode these pins are internal tree-net nodes, named by the
    // iterm convention (<inst>_<pin>) used when emitting the buffer X-calls.
    for (const GridIntersection& inter : grid_intersections_) {
      if (!inter.has_buffer || !inter.buffer_inst) {
        continue;
      }
      odb::dbITerm* in_pin = getBufferInputPin(inter.buffer_inst);
      if (in_pin) {
        feeder_input_nodes.push_back(
            sanitize(std::string(inter.buffer_inst->getConstName()) + "_"
                     + in_pin->getMTerm()->getConstName()));
      }
    }
  } else {
    for (const auto& [leaf_key, arrival] : arrivals_for_spice) {
      std::string spice_node = "clk_in_" + sanitize(leaf_key);
      feeder_input_nodes.push_back(spice_node);
      // Pick slew: user override > STA-captured leaf slew > deck default.
      float vclk_rise = rise_ns;
      float vclk_fall = fall_ns;
      if (!user_set_slew) {
        auto it = leaf_slews_ns_.find(leaf_key);
        if (it != leaf_slews_ns_.end() && it->second > 0.0f) {
          vclk_rise = it->second;
          vclk_fall = it->second;
        }
      }
      out << "Vclk_" << vclk_count << " " << spice_node << " 0 PULSE(0 " << vdd
          << " " << arrival << "n " << vclk_rise << "n " << vclk_fall << "n "
          << pw_ns << "n " << period_ns << "n)" << " $ " << leaf_key
          << " arrival=" << arrival << "ns slew=" << vclk_rise << "ns\n";
      // Extract inst name from key "inst/pin" and find the iterm ID
      size_t slash_pos = leaf_key.find('/');
      if (slash_pos != std::string::npos) {
        std::string inst_name = leaf_key.substr(0, slash_pos);
        odb::dbInst* inst = block_->findInst(inst_name.c_str());
        if (inst) {
          odb::dbITerm* input_iterm = getBufferInputPin(inst);
          if (input_iterm) {
            buf_input_spice_node[input_iterm->getId()] = spice_node;
          }
        }
      }
      vclk_count++;
    }
  }  // end !full_tree branch

  // Buffer instances (mesh buffers always; tree buffers when full_tree)
  out << "\n* Buffer instances\n";
  if (full_tree) {
    // Emit upstream tree buffer X-subckt calls. Use the iterm-CapNode naming
    // convention (inst_pin sanitized) so they connect via OpenRCX-extracted
    // wire parasitics on the clknet_leaf_* nets (already in relevant_nets).
    for (odb::dbInst* tinst : tree_buffers) {
      out << "X" << sanitize(tinst->getConstName());
      std::string master_name_t = tinst->getMaster()->getConstName();
      // pin_name -> iterm map for CDL pin order
      std::map<std::string, odb::dbITerm*> pin_to_iterm_t;
      for (odb::dbITerm* it : tinst->getITerms()) {
        pin_to_iterm_t[it->getMTerm()->getConstName()] = it;
      }
      auto write_t_pin = [&](odb::dbITerm* it) {
        odb::dbSigType st = it->getSigType();
        if (st == odb::dbSigType::POWER) {
          out << " VDD";
          return;
        }
        if (st == odb::dbSigType::GROUND) {
          out << " 0";
          return;
        }
        odb::dbNet* nn = it->getNet();
        if (!nn) {
          out << " 0";
          return;
        }
        // For root-bterm-driven net: use the bterm name (matches the Vclk).
        for (odb::dbBTerm* bt : nn->getBTerms()) {
          if (root_bterms.count(bt)) {
            out << " " << sanitize(bt->getConstName());
            return;
          }
        }
        // Otherwise use inst_pin name (matches the iterm CapNode in the net's
        // extracted parasitics).
        out << " "
            << sanitize(std::string(it->getInst()->getConstName()) + "_"
                        + std::string(it->getMTerm()->getConstName()));
      };
      auto it_subckt = subckt_pins.find(master_name_t);
      if (it_subckt != subckt_pins.end()) {
        for (const std::string& cdl_pin : it_subckt->second) {
          auto pit = pin_to_iterm_t.find(cdl_pin);
          if (pit != pin_to_iterm_t.end()) {
            write_t_pin(pit->second);
          } else {
            out << " " << cdl_pin;
          }
        }
      } else {
        for (odb::dbITerm* it : tinst->getITerms()) {
          write_t_pin(it);
        }
      }
      out << " " << master_name_t << "\n";
    }
  }
  for (const GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.buffer_inst) {
      continue;
    }
    odb::dbInst* inst = inter.buffer_inst;
    std::string inst_name = inst->getConstName();
    std::string master_name = inst->getMaster()->getConstName();
    out << "X" << inst_name;

    // Build a map of pin_name → ITerm for this instance
    std::map<std::string, odb::dbITerm*> pin_to_iterm;
    for (odb::dbITerm* iterm : inst->getITerms()) {
      pin_to_iterm[iterm->getMTerm()->getConstName()] = iterm;
    }

    // Lambda to write a single pin's net connection
    auto write_pin = [&](odb::dbITerm* iterm) {
      odb::dbSigType sig_type = iterm->getSigType();
      if (sig_type == odb::dbSigType::GROUND) {
        out << " 0";
        return;
      }
      if (sig_type == odb::dbSigType::POWER) {
        out << " VDD";
        return;
      }
      // Use per-buffer Vclk node for buffer input pins
      auto buf_it = buf_input_spice_node.find(iterm->getId());
      if (buf_it != buf_input_spice_node.end()) {
        out << " " << buf_it->second;
        return;
      }
      odb::dbNet* iterm_net = iterm->getNet();
      std::string net_name_str;
      if (iterm_net) {
        // Search the iterm's actual net (could be a sub-net when
        // merge_mesh_nets has not been called yet).
        bool found = false;
        for (odb::dbCapNode* cn : iterm_net->getCapNodes()) {
          if (cn->isITerm() && cn->getNode() == iterm->getId()) {
            net_name_str = node_name(cn);
            found = true;
            break;
          }
        }
        if (!found) {
          // no extraction data on this net: use the inst_pin convention shared
          // by the analytic-RC emitter, tree buffers, sink buffers and TSVs
          net_name_str
              = sanitize(std::string(iterm->getInst()->getConstName()) + "_"
                         + std::string(iterm->getMTerm()->getConstName()));
        }
      } else {
        net_name_str = iterm->getMTerm()->getConstName();
      }
      out << " " << net_name_str;
    };

    // Use CDL pin order if available, otherwise DB order
    auto it = subckt_pins.find(master_name);
    if (it != subckt_pins.end()) {
      for (const std::string& cdl_pin : it->second) {
        auto pit = pin_to_iterm.find(cdl_pin);
        if (pit != pin_to_iterm.end()) {
          write_pin(pit->second);
        } else {
          // Pin in CDL but not in DB — write name as-is
          out << " " << cdl_pin;
        }
      }
    } else {
      // No CDL info — fall back to DB order
      for (odb::dbITerm* iterm : inst->getITerms()) {
        write_pin(iterm);
      }
    }
    out << " " << master_name << "\n";
  }

  // Sink-tap buffers (backside sink side: mesh -> b_sink stub -> TSV -> A,
  // Y -> sink_drv -> FF pins). Not mesh intersection buffers, so emit them
  // here with iterm-CapNode node naming (inst_pin), like tree buffers.
  int sink_buf_count = 0;
  for (odb::dbInst* sinst : block_->getInsts()) {
    const std::string sname = sinst->getConstName();
    if (sname.rfind("sink_buf_", 0) != 0) {
      continue;
    }
    const std::string smaster = sinst->getMaster()->getConstName();
    out << "X" << sanitize(sname);
    std::map<std::string, odb::dbITerm*> spin_map;
    for (odb::dbITerm* it : sinst->getITerms()) {
      spin_map[it->getMTerm()->getConstName()] = it;
    }
    auto write_s_pin = [&](odb::dbITerm* it) {
      const odb::dbSigType st = it->getSigType();
      if (st == odb::dbSigType::POWER) {
        out << " VDD";
        return;
      }
      if (st == odb::dbSigType::GROUND) {
        out << " 0";
        return;
      }
      odb::dbNet* nn = it->getNet();
      if (!nn) {
        out << " 0";
        return;
      }
      std::string node
          = sanitize(std::string(it->getInst()->getConstName()) + "_"
                     + std::string(it->getMTerm()->getConstName()));
      for (odb::dbCapNode* cn : nn->getCapNodes()) {
        if (cn->isITerm() && cn->getNode() == it->getId()) {
          node = node_name(cn);
          break;
        }
      }
      out << " " << node;
    };
    auto sit = subckt_pins.find(smaster);
    if (sit != subckt_pins.end()) {
      for (const std::string& cdl_pin : sit->second) {
        auto pit = spin_map.find(cdl_pin);
        if (pit != spin_map.end()) {
          write_s_pin(pit->second);
        } else {
          out << " " << cdl_pin;
        }
      }
    } else {
      for (odb::dbITerm* it : sinst->getITerms()) {
        write_s_pin(it);
      }
    }
    out << " " << smaster << "\n";
    sink_buf_count++;
  }
  if (sink_buf_count > 0) {
    logger_->info(
        CMS, 897, "Emitted {} sink-tap buffer instances", sink_buf_count);
  }

  // Collect all relevant nets: mesh_net + sub-nets (clk_buf_* and sink_*).
  // When merge_mesh_nets is skipped, parasitics of the sub-nets stay on
  // their original nets. We emit parasitics from each — node names for
  // proxy bterms naturally serve as shared SPICE aliases that electrically
  // tie sub-net stubs to the mesh stripes at the intersection point.
  //
  // When full_tree=true, also walk back from each mesh-buffer A pin through
  // the upstream CTS clock tree (driver instances + driving nets) and add
  // those nets too. This builds a complete root-to-sink SPICE deck where
  // the upstream tree is fully simulated instead of being abstracted as
  // per-buffer Vclk PULSE sources.
  std::vector<odb::dbNet*> relevant_nets;
  std::set<odb::dbNet*, odb::ODBPtrLess> relevant_set;
  auto add_net = [&](odb::dbNet* n) {
    if (n && relevant_set.insert(n).second) {
      relevant_nets.push_back(n);
    }
  };
  add_net(mesh_net);
  std::string buf_prefix = base_name + "_buf_";
  const std::string bbuf_prefix = "b_" + buf_prefix;  // backside TSV.Y nets
  for (odb::dbNet* n : block_->getNets()) {
    if (n == mesh_net) {
      continue;
    }
    std::string nm = n->getName();
    if (nm.rfind(buf_prefix, 0) == 0) {
      add_net(n);
    } else if (nm.rfind(bbuf_prefix, 0) == 0 || nm.rfind("b_sink_", 0) == 0) {
      // backside stub nets (drive b_<clk>_buf_* and sink b_sink_*): TSV.Y +
      // proxy BTerm + the special BM1 stub tying it to the mesh
      add_net(n);
    } else if (nm.rfind("sink_", 0) == 0
               && nm.find("bterm") == std::string::npos) {
      add_net(n);
    }
  }
  // Also include upstream tree nets when full_tree=true (already collected).
  for (odb::dbNet* tn : tree_nets) {
    add_net(tn);
  }
  logger_->info(CMS,
                891,
                "Emitting SPICE parasitics from {} nets "
                "(mesh + sub-nets when merge_mesh_nets is skipped)",
                relevant_nets.size());

  // ---- ANALYTIC RC (no OpenRCX) ----
  // For every relevant net that has NO extracted parasitics (no CapNodes),
  // walk its dbWire and emit RC analytically:
  //   R_seg = (RPERSQ / width) * length      (tech-layer values from setRC)
  //   C_seg = (CPERSQDIST * width) * length  (half to each end node)
  //   vias  = tech-via resistance
  // Node names: iterm -> inst_pin, bterm -> bterm name, mesh junctions at
  // proxy-BTerm coords -> the BTerm name (proxy_alias_), else <net>_j<id>.
  // This bypasses OpenRCX entirely (no gt2n rules file exists; foreign-model
  // extraction segfaults on the 21-routing-layer stack).
  {
    // Same preprocessing OpenRCX does before extraction: (re)order each net's
    // dbWire and annotate the paths with their ITERM/BTERM endpoints. Without
    // this the walk sees only anonymous junctions and the buffer/FF pin nodes
    // never appear in the RC network (floating instances).
    odb::orderWires(logger_, block_);
    const double dbu_um = block_->getDbUnitsPerMicron();
    out << "\n* Analytic RC (nets without extracted parasitics)\n";
    int an_nets = 0, an_r = 0, an_c = 0, an_bridge = 0;
    for (odb::dbNet* net : relevant_nets) {
      if (net->getCapNodes().begin() != net->getCapNodes().end()) {
        continue;  // extracted net: emitted by the RSeg/CapNode loops below
      }
      odb::dbWire* wire = net->getWire();
      if (!wire) {
        continue;
      }
      const std::string net_san = sanitize(net->getConstName());
      std::map<std::string, double> node_cap;
      // Record every emitted R edge so we can detect (and bridge) RC islands:
      // dbWirePathItr traverses ENCODED junctions only, but detailed_route
      // connects some branches by geometric overlap/abutment -> the walk splits
      // one routed-connected net into disconnected components (buffer in one,
      // some sink pins in another) and those pins float. See island-bridging.
      std::vector<std::pair<std::string, std::string>> net_edges;
      auto node_of = [&](odb::dbITerm* it,
                         odb::dbBTerm* bt,
                         const odb::Point& pt,
                         odb::dbTechLayer* layer,
                         int jid) -> std::string {
        if (it) {
          return sanitize(std::string(it->getInst()->getConstName()) + "_"
                          + it->getMTerm()->getConstName());
        }
        if (bt) {
          return sanitize(bt->getConstName());
        }
        if (layer) {
          auto al = proxy_alias_.find(
              std::make_tuple(pt.getX(), pt.getY(), layer->getRoutingLevel()));
          if (al != proxy_alias_.end()) {
            return sanitize(al->second);
          }
        }
        return net_san + "_j" + std::to_string(jid);
      };
      odb::dbWirePathItr pitr;
      odb::dbWirePath path;
      odb::dbWirePathShape pshape;
      pitr.begin(wire);
      int ridx = 0;
      while (pitr.getNextPath(path)) {
        std::string prev_node = node_of(
            path.iterm, path.bterm, path.point, path.layer, path.junction_id);
        odb::Point prev_pt = path.point;
        while (pitr.getNextShape(pshape)) {
          odb::dbTechLayer* slayer
              = pshape.shape.isVia() ? nullptr : pshape.shape.getTechLayer();
          const std::string node = node_of(pshape.iterm,
                                           pshape.bterm,
                                           pshape.point,
                                           slayer,
                                           pshape.junction_id);
          double r_ohm = 0.0;
          if (pshape.shape.isVia()) {
            if (odb::dbTechVia* tv = pshape.shape.getTechVia()) {
              r_ohm = tv->getResistance();
            }
          } else if (slayer) {
            const double w_um = slayer->getWidth() / dbu_um;
            const double len_um
                = (std::abs(pshape.point.getX() - prev_pt.getX())
                   + std::abs(pshape.point.getY() - prev_pt.getY()))
                  / dbu_um;
            if (w_um > 0) {
              r_ohm = (slayer->getResistance() / w_um) * len_um;
              const double c_pf = (slayer->getCapacitance() * w_um) * len_um;
              node_cap[prev_node] += c_pf / 2.0;
              node_cap[node] += c_pf / 2.0;
            }
          }
          if (node != prev_node) {
            if (r_ohm < 1e-4) {
              r_ohm = 1e-4;  // keep the graph connected (no 0-ohm elements)
            }
            out << "R" << net_san << "_" << ridx++ << " " << prev_node << " "
                << node << " " << r_ohm << "\n";
            net_edges.emplace_back(prev_node, node);
            an_r++;
          }
          prev_node = node;
          prev_pt = pshape.point;
        }
      }
      int cidx = 0;
      for (const auto& [nname, cap_pf] : node_cap) {
        if (cap_pf <= 0) {
          continue;
        }
        out << "C" << net_san << "_" << cidx++ << " " << nname << " 0 "
            << cap_pf * 1e-12 << "\n";
        an_c++;
      }

      // ---- island-bridging ----
      // Union-find over the emitted R edges, then reconnect any net pin that
      // landed in a different component than the net driver. The pins are
      // physically coincident with the driver's net (detailed_route connected
      // them by overlap, which dbWirePathItr's junction walk does not follow),
      // so a near-zero series R is the correct tie. Without this, overlap-
      // connected sink pins float at 0 V and their .measure fails.
      std::map<std::string, std::string> par;
      auto ufind = [&](std::string x) -> std::string {
        par.emplace(x, x);
        while (par[x] != x) {
          par[x] = par[par[x]];
          x = par[x];
        }
        return x;
      };
      auto uni = [&](const std::string& a, const std::string& b) {
        par.emplace(a, a);
        par.emplace(b, b);
        par[ufind(a)] = ufind(b);
      };
      for (const auto& e : net_edges) {
        uni(e.first, e.second);
      }
      // Collect pin nodes (same naming as node_of's iterm/bterm branches) and
      // pick a driver: the net's OUTPUT iterm, else any BTerm, else any pin.
      std::vector<std::string> pin_nodes;
      std::string driver_node;
      for (odb::dbITerm* it : net->getITerms()) {
        std::string pn = sanitize(std::string(it->getInst()->getConstName())
                                  + "_" + it->getMTerm()->getConstName());
        pin_nodes.push_back(pn);
        par.emplace(pn, pn);
        if (it->getIoType() == odb::dbIoType::OUTPUT) {
          driver_node = pn;
        }
      }
      for (odb::dbBTerm* bt : net->getBTerms()) {
        std::string pn = sanitize(bt->getConstName());
        pin_nodes.push_back(pn);
        par.emplace(pn, pn);
        if (driver_node.empty()) {
          driver_node = pn;
        }
      }
      if (driver_node.empty() && !pin_nodes.empty()) {
        driver_node = pin_nodes.front();
      }
      if (!driver_node.empty()) {
        int bidx = 0;
        for (const std::string& pn : pin_nodes) {
          if (ufind(pn) != ufind(driver_node)) {
            out << "Rbridge" << net_san << "_" << bidx++ << " " << pn << " "
                << driver_node << " 1e-3\n";
            uni(pn, driver_node);
            an_r++;
            an_bridge++;
          }
        }
      }
      an_nets++;
    }
    if (an_nets > 0) {
      logger_->info(CMS,
                    898,
                    "Analytic RC: {} nets, {} resistors ({} island bridges), "
                    "{} node caps (no OpenRCX)",
                    an_nets,
                    an_r,
                    an_bridge,
                    an_c);
    }
  }

  // FinFET mode: emit clocksyn-style lumped cin/cout per mesh buffer to
  // match clocksyn's SPICE deck (segment.C:594-609). Tech-file constants
  // for ASAP7 (asap7_only_x4.tech):
  //   GATE_INCAP  = 0.619928 fF (INVx1 input cap)
  //   GATE_OUTCAP = 2.85    fF (INVx1 output drain cap)
  //   nfin_per_unit = 3, buf_fixed_gain = 3
  // Closed form: cin  = 0.207  fF × N,  cout = 2.85 fF × N  for BUFxN.
  if (finfet) {
    out << "\n* FinFET lumped cin/cout per mesh buffer (clocksyn-style)\n";
    int n_finfet = 0;
    for (const GridIntersection& inter : grid_intersections_) {
      if (!inter.has_buffer || !inter.buffer_inst) {
        continue;
      }
      std::string master = inter.buffer_inst->getMaster()->getConstName();
      // Parse "BUFxN_..." → extract N (also handles BUFx16f-style suffixes)
      size_t pos = master.find("BUFx");
      if (pos == std::string::npos) {
        continue;
      }
      size_t start = pos + 4;
      size_t end = start;
      while (end < master.size()
             && (master[end] >= '0' && master[end] <= '9')) {
        end++;
      }
      if (end == start) {
        continue;
      }
      int N = std::stoi(master.substr(start, end - start));
      double cin_fF = 0.207 * N;
      double cout_fF = 2.85 * N;

      // Input/output node names match how the buffer X-instance was emitted.
      // Input: clk_in_<inst>_<inputpin>  (when !full_tree) —
      // buf_input_spice_node has it. Output: <inst>_Y (sanitized inst_name +
      // "_Y")
      odb::dbITerm* in_pin = getBufferInputPin(inter.buffer_inst);
      odb::dbITerm* out_pin = getBufferOutputPin(inter.buffer_inst);
      std::string in_node, out_node;
      if (in_pin) {
        auto it = buf_input_spice_node.find(in_pin->getId());
        in_node = (it != buf_input_spice_node.end())
                      ? it->second
                      : sanitize(std::string(inter.buffer_inst->getConstName())
                                 + "_" + in_pin->getMTerm()->getConstName());
      }
      if (out_pin) {
        out_node = sanitize(std::string(inter.buffer_inst->getConstName()) + "_"
                            + out_pin->getMTerm()->getConstName());
      }
      if (!in_node.empty()) {
        out << "Ccin_" << inter.buffer_inst->getConstName() << " " << in_node
            << " 0 " << cin_fF << "f\n";
      }
      if (!out_node.empty()) {
        out << "Ccout_" << inter.buffer_inst->getConstName() << " " << out_node
            << " 0 " << cout_fF << "f\n";
      }
      n_finfet++;
    }
    logger_->info(CMS,
                  894,
                  "finfet mode: emitted lumped cin/cout for {} mesh buffers",
                  n_finfet);
  }

  // Resistance segments — emit from every relevant net
  out << "\n* Resistance segments\n";
  int r_count = 0;
  for (odb::dbNet* net : relevant_nets) {
    for (odb::dbRSeg* rseg : net->getRSegs()) {
      odb::dbCapNode* src_node = rseg->getSourceCapNode();
      odb::dbCapNode* tgt_node = rseg->getTargetCapNode();
      if (!src_node || !tgt_node) {
        continue;
      }
      double res = rseg->getResistance(0);
      out << "R" << r_count << " " << node_name(src_node) << " "
          << node_name(tgt_node) << " " << res << "\n";
      r_count++;
    }
  }

  // Ground capacitances — emit from every relevant net
  out << "\n* Ground capacitances\n";
  int c_count = 0;
  for (odb::dbNet* net : relevant_nets) {
    for (odb::dbCapNode* cap_node : net->getCapNodes()) {
      double cap = cap_node->getCapacitance(0);
      if (cap > 0.0) {
        out << "C" << c_count << " " << node_name(cap_node) << " 0 "
            << cap * 1e-15 << "\n";
        c_count++;
      }
    }
  }
  if (c_count == 0) {
    // Fallback: read cap from RSegs (LEF-RC mode stores cap here)
    for (odb::dbNet* net : relevant_nets) {
      for (odb::dbRSeg* rseg : net->getRSegs()) {
        double cap = rseg->getGroundCapacitance(0);
        if (cap > 0.0) {
          odb::dbCapNode* tgt_node = rseg->getTargetCapNode();
          if (!tgt_node) {
            continue;
          }
          out << "C" << c_count << " " << node_name(tgt_node) << " 0 "
              << cap * 1e-15 << "\n";
          c_count++;
        }
      }
    }
  }

  // Coupling capacitances — emit from every relevant net
  out << "\n* Coupling capacitances\n";
  int cc_count = 0;
  std::set<uint32_t> visited_cc;
  for (odb::dbNet* net : relevant_nets) {
    for (odb::dbCapNode* cap_node : net->getCapNodes()) {
      for (odb::dbCCSeg* cc : cap_node->getCCSegs()) {
        if (visited_cc.count(cc->getId())) {
          continue;
        }
        visited_cc.insert(cc->getId());
        odb::dbCapNode* src = cc->getSourceCapNode();
        odb::dbCapNode* tgt = cc->getTargetCapNode();
        double cap = cc->getCapacitance(0);
        if (cap > 0.0) {
          out << "Cc" << cc_count << " " << node_name(src) << " "
              << node_name(tgt) << " " << cap * 1e-15 << "\n";
          cc_count++;
        }
      }
    }
  }

  // Build the authoritative set of sink iterm IDs from clockToSinks_ — this
  // is the SDC-defined flop CLK pin set that CMS created sink bterms for.
  // Used to filter sink loops below so we only emit Csink/measure for the
  // real sinks (excluding any tree-buffer pins that ended up in relevant
  // nets via the full_tree walk-back).
  std::set<uint32_t> real_sink_iterm_ids;
  if (clockToSinks_.count(clock_name)) {
    for (const ClockSink& s : clockToSinks_[clock_name]) {
      if (s.iterm) {
        real_sink_iterm_ids.insert(s.iterm->getId());
      }
    }
  }

  // TSV front<->back crossings: the bridge cell is a passive via stack
  // (M1->V0->M0->VSD->SDCON->VBPR->BPR->BV0->BM1), modeled as a series
  // resistor between its A node (frontside net) and Y node (backside stub
  // net). Default 149ohm from the real GT2N ITF:
  //   V0 54.99 + VSD 36.86 + VBPR 32.0 + BV0 25.10.
  out << "\n* TSV crossings (passive front<->back via stack, series R)\n";
  int tsv_count = 0;
  for (odb::dbNet* net : relevant_nets) {
    for (odb::dbITerm* iterm : net->getITerms()) {
      // anchor on the Y pin so each TSV is emitted exactly once
      if (std::string(iterm->getMTerm()->getConstName()) != "Y") {
        continue;
      }
      odb::dbInst* inst = iterm->getInst();
      if (std::string(inst->getMaster()->getName()).find("TSV")
          == std::string::npos) {
        continue;
      }
      odb::dbITerm* a_it = inst->findITerm("A");
      if (!a_it || !a_it->getNet()) {
        continue;
      }
      auto node_for = [&](odb::dbITerm* it) {
        for (odb::dbCapNode* cn : it->getNet()->getCapNodes()) {
          if (cn->isITerm() && cn->getNode() == it->getId()) {
            return node_name(cn);
          }
        }
        return sanitize(std::string(it->getInst()->getConstName()) + "_"
                        + it->getMTerm()->getConstName());
      };
      out << "Rtsv_" << sanitize(inst->getConstName()) << " " << node_for(a_it)
          << " " << node_for(iterm) << " " << tsv_res << "\n";
      tsv_count++;
    }
  }
  if (tsv_count > 0) {
    logger_->info(CMS,
                  895,
                  "Emitted {} TSV crossings as {}ohm series R",
                  tsv_count,
                  tsv_res);
  }

  // Sink gate-cap injection. OpenRCX extracts wire parasitics only — the
  // input-pin (gate) cap of each flop is a Liberty value, not in the RSeg/
  // CapNode network. Without these, sinks are purely resistive loads and
  // transition essentially instantly, giving unrealistically small skew.
  // Iterate sink iterms across all relevant nets (mesh + sub-nets).
  out << "\n* Sink pin caps (Liberty input-pin capacitance, one per sink "
         "ITerm)\n";
  int sink_cap_count = 0;
  std::set<uint32_t> seen_sink_iterms;
  for (odb::dbNet* net : relevant_nets) {
    for (odb::dbITerm* iterm : net->getITerms()) {
      std::string inst_name = iterm->getInst()->getConstName();
      if (inst_name.find("mesh_buf_") == 0) {
        continue;
      }
      // In full_tree mode, also skip upstream CTS tree-buffer instances —
      // they're drivers, not sinks.
      if (tree_buffers.count(iterm->getInst())) {
        continue;
      }
      // Only emit for authoritative sink iterms (the SDC-defined flop CLK
      // pins from clockToSinks_). This skips CTS infrastructure pins
      // (clkbuf_*, clkload*, anonymous _NNNN_) that share the clock net
      // but aren't real sinks.
      if (!real_sink_iterm_ids.empty()
          && !real_sink_iterm_ids.count(iterm->getId())) {
        continue;
      }
      if (seen_sink_iterms.count(iterm->getId())) {
        continue;
      }
      seen_sink_iterms.insert(iterm->getId());
      std::string sink_node;
      for (odb::dbCapNode* cn : net->getCapNodes()) {
        if (cn->isITerm() && cn->getNode() == iterm->getId()) {
          sink_node = node_name(cn);
          break;
        }
      }
      if (sink_node.empty()) {
        sink_node = sanitize(inst_name + "_"
                             + std::string(iterm->getMTerm()->getConstName()));
      }
      // Liberty input-pin cap (farads) for this (master, pin).
      // Use MAX corner to match ISPD/clocksyn convention (its sink-cap
      // generator calls timing.getPortCap(..., timing.Max) — see
      // /home/wali2/mesh/ispd/save_asap7_ispd.py:143). Using nominal/default
      // capacitance() instead gave ~8.7% lower per-sink cap than clocksyn.
      sta::LibertyCell* lcell = network_->libertyCell(
          network_->dbToSta(iterm->getInst()->getMaster()));
      if (!lcell) {
        // the db->sta master mapping can miss (e.g. odb-read designs); the
        // by-name library lookup is the same path `get_lib_pins` uses
        lcell = network_->findLibertyCell(
            iterm->getInst()->getMaster()->getConstName());
      }
      double pin_cap_f = 0.0;
      if (lcell) {
        sta::LibertyPort* port
            = lcell->findLibertyPort(iterm->getMTerm()->getConstName());
        if (port) {
          pin_cap_f = port->capacitance(sta::MinMax::max());
          if (pin_cap_f <= 0.0) {
            // libs with only the scalar `capacitance` attribute (e.g. gt2n)
            // return 0 from the corner-specific query -- fall back to scalar
            pin_cap_f = port->capacitance();
          }
        }
      }
      if (pin_cap_f <= 0.0) {
        logger_->warn(CMS,
                      899,
                      "No liberty pin cap for sink {}/{} (lcell {} found)",
                      iterm->getInst()->getConstName(),
                      iterm->getMTerm()->getConstName(),
                      lcell ? "was" : "NOT");
      }
      if (pin_cap_f > 0.0) {
        out << "Csink" << sink_cap_count << " " << sink_node << " 0 "
            << pin_cap_f << "\n";
        sink_cap_count++;
      }
    }
  }
  logger_->info(CMS, 869, "Added {} sink pin-cap entries", sink_cap_count);

  // Sink arrival time measurements
  // Iterate sink iterms across all relevant nets (mesh + sub-nets).
  out << "\n* Sink arrival time measurements (50% VDD, 1st rising edge)\n";
  out << "* Post-process: skew = max(t_sink_i) - min(t_sink_i)\n";
  float half_vdd = vdd / 2.0;
  int sink_count = 0;
  std::set<uint32_t> seen_measure_iterms;
  for (odb::dbNet* net : relevant_nets) {
    for (odb::dbITerm* iterm : net->getITerms()) {
      std::string inst_name = iterm->getInst()->getConstName();
      if (inst_name.find("mesh_buf_") == 0) {
        continue;
      }
      // In full_tree mode, also skip upstream CTS tree-buffer instances —
      // they're drivers, not sinks.
      if (tree_buffers.count(iterm->getInst())) {
        continue;
      }
      // Only emit for authoritative sink iterms (the SDC-defined flop CLK
      // pins from clockToSinks_). This skips CTS infrastructure pins
      // (clkbuf_*, clkload*, anonymous _NNNN_) that share the clock net
      // but aren't real sinks.
      if (!real_sink_iterm_ids.empty()
          && !real_sink_iterm_ids.count(iterm->getId())) {
        continue;
      }
      if (seen_measure_iterms.count(iterm->getId())) {
        continue;
      }
      seen_measure_iterms.insert(iterm->getId());
      // Find the CapNode for this sink ITerm to get its SPICE node name
      std::string sink_node;
      for (odb::dbCapNode* cn : net->getCapNodes()) {
        if (cn->isITerm() && cn->getNode() == iterm->getId()) {
          sink_node = node_name(cn);
          break;
        }
      }
      if (sink_node.empty()) {
        sink_node = sanitize(inst_name + "_"
                             + std::string(iterm->getMTerm()->getConstName()));
      }
      out << ".measure tran t_sink_" << sink_count << " WHEN v(" << sink_node
          << ")=" << half_vdd << " RISE=1" << "\n* " << inst_name << "/"
          << iterm->getMTerm()->getConstName() << "\n";
      // Sink rise slew = 10%->90% transition time at the FF clock pin. This is
      // the ACTUAL clock-edge quality seen by the flop (after the mesh + sink
      // buffer regenerate the feeder edge), distinct from the feeder-input
      // slew.
      const double v10 = 0.1 * vdd;
      const double v90 = 0.9 * vdd;
      out << ".measure tran slew_sink_" << sink_count << " TRIG v(" << sink_node
          << ")=" << v10 << " RISE=1 TARG v(" << sink_node << ")=" << v90
          << " RISE=1\n";
      sink_count++;
    }
  }
  logger_->info(CMS,
                868,
                "Added {} sink .measure statements for skew analysis",
                sink_count);

  // Feeder-side slew: 10%-90% rise at each mesh-buffer INPUT node, measured the
  // same way as the sink slew. Comparing feeder vs sink slew shows whether the
  // mesh + sink buffer regenerate a degraded (sparse-mesh) feeder edge.
  {
    const double v10 = 0.1 * vdd;
    const double v90 = 0.9 * vdd;
    int fslew = 0;
    for (const std::string& fn : feeder_input_nodes) {
      out << ".measure tran slew_feed_" << fslew << " TRIG v(" << fn
          << ")=" << v10 << " RISE=1 TARG v(" << fn << ")=" << v90
          << " RISE=1\n";
      fslew++;
    }
    logger_->info(
        CMS, 870, "Added {} feeder-input slew .measure statements", fslew);
  }

  // Simulation control — step = rise_time/2.
  // Stop = max buffer arrival + 2 full periods, so we have at least one
  // settled clock period for the power measure (last period below).
  // autostop will exit early once all .measure statements complete.
  float tran_step = rise_ns / 2.0;
  float max_arrival = 0.0f;
  for (const auto& [k, v] : arrivals_for_spice) {
    max_arrival = std::max(max_arrival, v);
  }
  float tran_stop = max_arrival + 2.0f * period_ns;
  out << "\n* Simulation control\n";
  out << ".tran " << tran_step << "n " << tran_stop << "n\n";

  // Total power over the last full clock period (clocksyn-style).
  // HSPICE's reserved `power` quantity = Σ V(source_i) × I(source_i) across
  // every voltage source in the deck (Vvdd dominates; Vclk_* contribute
  // tiny gate-charge currents). Averaged over [tran_stop - period, tran_stop]
  // to skip initial settling and report a true steady-state number.
  // Post-process:  Power_mW = avg_power × 1000.
  out << ".measure tran avg_power AVG power FROM=" << (tran_stop - period_ns)
      << "n TO=" << tran_stop << "n\n";
  out << ".end\n";

  out.close();

  logger_->info(CMS,
                862,
                "Wrote SPICE netlist: {} ({} R, {} C, {} Cc, {} sinks)",
                spice_file,
                r_count,
                c_count,
                cc_count,
                sink_count);
}

// Merges buffer and sink nets into clk_mesh for parasitic extraction
void ClockMesh::mergeNetsToMesh(const std::string& clock_name)
{
  std::string base_name = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
  std::string mesh_net_name = base_name + "_mesh";

  odb::dbNet* mesh_net = block_->findNet(mesh_net_name.c_str());
  if (!mesh_net) {
    logger_->error(CMS, 800, "Mesh net '{}' not found", mesh_net_name);
    return;
  }

  int buffers_merged = 0;
  int sinks_merged = 0;

  // Collect nets to merge (can't modify net list while iterating)
  std::vector<odb::dbNet*> buf_nets;
  std::vector<odb::dbNet*> sink_nets;
  std::string buf_prefix = base_name + "_buf_";

  for (odb::dbNet* net : block_->getNets()) {
    std::string name = net->getName();
    if (name.rfind(buf_prefix, 0) == 0) {
      buf_nets.push_back(net);
    } else if (name.rfind("sink_", 0) == 0
               && name.find("bterm") == std::string::npos) {
      sink_nets.push_back(net);
    }
  }

  odb::dbWire* new_wire = odb::dbWire::create(mesh_net);
  odb::dbWireEncoder encoder;
  encoder.begin(new_wire);

  // Re-encode routing from buffer nets
  for (odb::dbNet* net : buf_nets) {
    odb::dbWire* wire = net->getWire();
    if (wire) {
      reencodeWireToMesh(wire, encoder);
    }
  }

  // Re-encode routing from sink nets
  for (odb::dbNet* net : sink_nets) {
    odb::dbWire* wire = net->getWire();
    if (wire) {
      reencodeWireToMesh(wire, encoder);
    }
  }

  encoder.end();

  for (odb::dbNet* net : buf_nets) {
    net->setDoNotTouch(false);

    std::vector<odb::dbITerm*> iterms(net->getITerms().begin(),
                                      net->getITerms().end());
    for (odb::dbITerm* iterm : iterms) {
      iterm->disconnect();
      iterm->connect(mesh_net);
    }
    odb::dbNet::destroy(net);
    buffers_merged++;
  }

  for (odb::dbNet* net : sink_nets) {
    net->setDoNotTouch(false);

    std::vector<odb::dbITerm*> iterms(net->getITerms().begin(),
                                      net->getITerms().end());
    for (odb::dbITerm* iterm : iterms) {
      iterm->disconnect();
      iterm->connect(mesh_net);
    }
    odb::dbNet::destroy(net);
    sinks_merged++;
  }

  logger_->info(CMS,
                801,
                "Merged {} buffer nets and {} sink nets into '{}'",
                buffers_merged,
                sinks_merged,
                mesh_net_name);

  // Rebuild the STA network view from the current DB state.
  // write_verilog reads from the STA network (not OpenDB directly),
  // and the BTerm destruction callbacks only disconnect pins in STA
  // but don't remove ports from the top-level cell.
  network_->readDbAfter(db_);
  logger_->info(CMS, 802, "Rebuilt STA network after merge");
}

// Writes Verilog with buffer and sink nets merged to mesh net
void ClockMesh::writeMeshVerilog(const std::string& clock_name,
                                 const std::string& input_filename,
                                 const std::string& output_filename)
{
  std::string base_name = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
  std::string mesh_net_target = base_name + "_mesh";

  std::ifstream in(input_filename);
  if (!in.is_open()) {
    logger_->error(CMS, 751, "Cannot open input file: {}", input_filename);
    return;
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  std::string content = buffer.str();
  in.close();

  std::set<std::string> nets_to_replace;
  std::string buf_net_prefix = base_name + "_buf_";

  for (odb::dbNet* net : block_->getNets()) {
    std::string name = net->getName();
    if (name.rfind(buf_net_prefix, 0) == 0) {
      nets_to_replace.insert(name);
    }
    if (name.rfind("sink_", 0) == 0
        && name.find("bterm") == std::string::npos) {
      nets_to_replace.insert(name);
    }
  }

  std::set<std::string> bterms_to_remove;
  for (odb::dbBTerm* bterm : block_->getBTerms()) {
    std::string name = bterm->getName();
    if (name.rfind("proxy_", 0) == 0 || name.rfind("sink_bterm_", 0) == 0) {
      bterms_to_remove.insert(name);
    }
  }

  std::vector<std::string> sorted_nets(nets_to_replace.begin(),
                                       nets_to_replace.end());
  std::sort(sorted_nets.begin(),
            sorted_nets.end(),
            [](const std::string& a, const std::string& b) {
              return a.length() > b.length();
            });

  for (const std::string& net_name : sorted_nets) {
    size_t pos = 0;
    while ((pos = content.find(net_name, pos)) != std::string::npos) {
      content.replace(pos, net_name.length(), mesh_net_target);
      pos += mesh_net_target.length();
    }
  }

  std::istringstream iss(content);
  std::ostringstream oss;
  std::string line;

  while (std::getline(iss, line)) {
    bool skip_line = false;
    for (const std::string& bterm_name : bterms_to_remove) {
      if (line.find(bterm_name) != std::string::npos) {
        skip_line = true;
        break;
      }
    }
    if (!skip_line) {
      oss << line << "\n";
    }
  }

  std::string result = oss.str();
  size_t pos = result.find(",\n input ");
  if (pos != std::string::npos) {
    result.replace(pos, 2, ");");
  }

  std::string final_content = "// Mesh-Merged Verilog Netlist\n";
  final_content += "// Modified by OpenROAD ClockMesh\n";
  final_content += "// " + buf_net_prefix + "* and sink_* nets merged to '"
                   + mesh_net_target + "'\n";
  final_content
      += "// CTS tree on original clock net '" + base_name + "' preserved\n";
  final_content += "// proxy_* and sink_bterm_* BTERMs removed from ports\n\n";
  final_content += result;

  std::ofstream out(output_filename);
  if (!out.is_open()) {
    logger_->error(CMS, 752, "Cannot open output file: {}", output_filename);
    return;
  }
  out << final_content;
  out.close();

  logger_->info(CMS, 755, "Wrote mesh-merged Verilog: {}", output_filename);
}

}  // namespace cms
