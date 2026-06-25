// SPDX-License-Identifier: BSD-3-Clause

#include "cms/ClockMesh.hh"
#include "cms/LoadAdaptiveBuffer.hh"
#include <algorithm>
#include <climits>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include "cts/TritonCTS.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "odb/dbShape.h"
#include "odb/dbWireCodec.h"
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
          clockToSinks_[clkName].insert(clockToSinks_[clkName].end(), sinks.begin(), sinks.end());
        }
      }
    }
  }
}

// Extracts the nets connected to clock leaf pins
void ClockMesh::findClockRoots(sta::Clock* clk, std::set<odb::dbNet*, odb::ODBPtrLess>& clockNets)
{
  for (const sta::Pin* pin : clk->leafPins()) {
    odb::dbITerm* instTerm;
    odb::dbBTerm* port;
    odb::dbModITerm* moditerm;
    network_->staToDb(pin, instTerm, port, moditerm);
    odb::dbNet* net = instTerm ? instTerm->getNet() :
                      (port ? port->getNet() : nullptr);
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
  sta::LibertyPort* inputPort =
      libertyCell->findLibertyPort(iterm->getMTerm()->getConstName());

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
        std::string name = std::string(inst->getConstName()) + "/" + std::string(mterm->getConstName());
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

// Collects macro and blockage bounding boxes, bloated by halo_dbu, into blockage_rects_
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

  logger_->info(CMS, 130,
                "Collected {} blockage rects (halo = {} DBU)",
                blockage_rects_.size(), halo_dbu);
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
// Returns 0..N surviving subsegments; segments shorter than min_segment_length dropped.
std::vector<odb::Rect> ClockMesh::clipWireByBlockages(
    const odb::Rect& wire_rect, bool is_horizontal, int min_segment_length) const
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
  int end    = is_horizontal ? wire_rect.xMax() : wire_rect.yMax();

  auto emit = [&](int seg_start, int seg_end) {
    if (seg_end - seg_start < min_segment_length) {
      return;
    }
    if (is_horizontal) {
      result.emplace_back(seg_start, wire_rect.yMin(),
                          seg_end,   wire_rect.yMax());
    } else {
      result.emplace_back(wire_rect.xMin(), seg_start,
                          wire_rect.xMax(), seg_end);
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
  if (!block_) {
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
        if (sbox->isVia()) {
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
      pdn_vstrap_x_.back().second = std::max(pdn_vstrap_x_.back().second, b.second);
    } else {
      pdn_vstrap_x_.push_back(b);
    }
  }
  const double dbu = block_->getDbUnitsPerMicron();
  std::ostringstream gaps;
  for (size_t i = 1; i < pdn_vstrap_x_.size(); ++i) {
    gaps << ' '
         << (pdn_vstrap_x_[i].first - pdn_vstrap_x_[i - 1].second) / dbu;
  }
  logger_->info(CMS, 140,
                "PDN vertical straps: {} bands, first at x={:.3f}um; "
                "inter-strap gaps (um):{}",
                pdn_vstrap_x_.size(), pdn_vstrap_x_.front().first / dbu,
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
  const int left_x = band.first - spacing - half_w;
  const int right_x = band.second + spacing + half_w;
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

// Notches a horizontal clock wire segment at every vertical PDN strap (removing
// [strap - spacing, strap + spacing]). Returns the surviving sub-segments.
std::vector<odb::Rect> ClockMesh::notchHByPdn(const odb::Rect& seg, int spacing,
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
      std::remove_if(
          h_wires.begin(), h_wires.end(),
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
    logger_->info(CMS, 142,
                  "Pruned {} orphan horizontal segments (no vertical clock "
                  "wire in that region)",
                  dropped);
  }
}

// Creates horizontal mesh wires at regular pitch intervals
void ClockMesh::createHorizontalWires(odb::dbNet* net, odb::dbTechLayer* layer, const odb::Rect& bbox, int pitch, std::vector<MeshWire>& wires)
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
  const int spacing = std::max(layer->getSpacing(), 1);

  for (int y_pos = bbox.yMin(); y_pos <= bbox.yMax(); y_pos += pitch) {
    const int y_min = y_pos - half_width;
    const int y_max = y_pos + half_width;
    odb::Rect full_rect(x_start, y_min, x_end, y_max);
    for (const odb::Rect& seg
         : clipWireByBlockages(full_rect, true, min_segment_length)) {
      // notch at vertical PDN straps (the via columns block BM2 there)
      for (const odb::Rect& piece
           : notchHByPdn(seg, spacing, min_segment_length)) {
        wires.emplace_back(layer, net, piece, true);
      }
    }
  }
}

// Creates vertical mesh wires at regular pitch intervals
void ClockMesh::createVerticalWires(odb::dbNet* net, odb::dbTechLayer* layer, const odb::Rect& bbox, int pitch, std::vector<MeshWire>& wires)
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
  const int spacing = std::max(layer->getSpacing(), 1);

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
    for (const odb::Rect& seg
         : clipWireByBlockages(full_rect, false, min_segment_length)) {
      wires.emplace_back(layer, net, seg, false);
    }
  }
  if (!pdn_vstrap_x_.empty()) {
    logger_->info(CMS, 141,
                  "Vertical clock wires: {} shifted clear of PDN straps, "
                  "{} dropped (no room in gap)",
                  shifted, dropped);
  }
}

// Creates vias where horizontal and vertical wires intersect
void ClockMesh::createViasAtIntersections(const std::vector<MeshWire>& h_wires, const std::vector<MeshWire>& v_wires, std::vector<MeshVia>& vias)
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
            odb::dbNet* net = bterm ? bterm->getNet()
                            : (iterm ? iterm->getNet() : nullptr);
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
      logger_->warn(CMS, 107, "Failed to get/create SWire for net {}", wire.net->getName());
      continue;
    }
    odb::dbSBox::create(swire, wire.layer, wire.rect.xMin(), wire.rect.yMin(), wire.rect.xMax(), wire.rect.yMax(), odb::dbWireShapeType::STRIPE);
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

      odb::dbSBox::create(swire, tech_via, via_x, via_y, odb::dbWireShapeType::NONE);
    }
  }
}

// Main function: creates mesh grid, places buffers, and runs CTS
void ClockMesh::createMeshGrid(const std::string& clock_name, odb::dbTechLayer* h_layer, odb::dbTechLayer* v_layer, int pitch, const std::vector<std::string>& buffer_list, int macro_halo_dbu, const std::vector<std::string>& cts_buffer_list, const std::string& mesh_strategy)
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

  collectBlockageRects(macro_halo_dbu);
  collectPdnVStraps();

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
  int h_xtrack_pitch = 0, h_xtrack_offset = 0;  // M4 X-tracks (for BTerm X access)
  int v_track_pitch = 0, v_track_offset = 0;  // M5 X-tracks
  int v_ytrack_pitch = 0, v_ytrack_offset = 0;  // M5 Y-tracks (for BTerm Y access)

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
      logger_->warn(CMS, 211, "No y-tracks found for horizontal mesh layer {}", h_layer->getName());
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
      logger_->warn(CMS, 212, "No x-tracks found for vertical mesh layer {}", v_layer->getName());
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
    while (b != 0) { int t = b; b = a % b; a = t; }
    return a;
  };
  auto lcm_int = [&](int a, int b) {
    if (a == 0) return b;
    if (b == 0) return a;
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
  logger_->info(CMS, 213,
                "Track-aligned pitch: input={}dbu, LCM of all track pitches={}dbu, "
                "aligned pitch={}dbu", pitch, combined, aligned_pitch);

  // Align x-start to v_layer tracks (vertical wire x-positions)
  int aligned_x_start = bbox.xMin();
  if (v_track_pitch > 0) {
    int idx = (bbox.xMin() - v_track_offset + v_track_pitch / 2) / v_track_pitch;
    aligned_x_start = v_track_offset + idx * v_track_pitch;
  }
  // Align y-start to h_layer tracks (horizontal wire y-positions)
  int aligned_y_start = bbox.yMin();
  if (h_track_pitch > 0) {
    int idx = (bbox.yMin() - h_track_offset + h_track_pitch / 2) / h_track_pitch;
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
  odb::Rect aligned_bbox(aligned_x_start, aligned_y_start, aligned_x_end, aligned_y_end);
  logger_->info(CMS, 122,
      "Grid clamped inside core [{},{}]-[{},{}] (margins x={} y={})",
      aligned_bbox.xMin(), aligned_bbox.yMin(),
      aligned_bbox.xMax(), aligned_bbox.yMax(), x_margin, y_margin);

  // Log grid size
  double dbu_per_um = tech->getDbUnitsPerMicron();
  int h_wire_count = (aligned_pitch > 0) ? (aligned_bbox.dy() / aligned_pitch + 1) : 0;
  int v_wire_count = (aligned_pitch > 0) ? (aligned_bbox.dx() / aligned_pitch + 1) : 0;
  logger_->info(CMS, 121,
      "Clock mesh parameters: pitch={:.3f}um, expected {} H-lines x {} V-lines, "
      "bbox {:.3f}x{:.3f}um",
      aligned_pitch / dbu_per_um,
      h_wire_count, v_wire_count,
      aligned_bbox.dx() / dbu_per_um, aligned_bbox.dy() / dbu_per_um);

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(CMS, 116, "Failed to find clock net for '{}'", clock_name);
    return;
  }

  std::vector<MeshWire> h_wires;
  if (h_layer) {
    createHorizontalWires(mesh_net, h_layer, aligned_bbox, aligned_pitch, h_wires);
  }

  std::vector<MeshWire> v_wires;
  if (v_layer) {
    createVerticalWires(mesh_net, v_layer, aligned_bbox, aligned_pitch, v_wires);
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
    for (const MeshWire& h_wire : h_wires) {
      int h_y = (h_wire.rect.yMin() + h_wire.rect.yMax()) / 2;
      for (const MeshWire& v_wire : v_wires) {
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
        odb::dbTechLayer* buf_layer = selectBufferLayer(h_wire.layer, v_wire.layer);
        grid_intersections_.emplace_back(v_x, h_y, buf_layer);
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
    if (strat == "set_cover") {
      placeBuffersSetCover(block_, network_, logger_,
                           mesh_h_layer_, mesh_v_layer_,
                           mesh_wires_, clockToSinks_[clock_name],
                           grid_intersections_, buffer_list);
    } else if (strat == "adaptive") {
      placeBuffersLoadAdaptive(block_, network_, logger_,
                               mesh_h_layer_, mesh_v_layer_,
                               mesh_wires_, clockToSinks_[clock_name],
                               grid_intersections_, buffer_list);
    } else {
      placeBuffersAtIntersections(buffer_list[0], mesh_net);
    }
    connectBuffersToNets(mesh_net, clock_name);
    // Count what was actually placed (set_cover may leave intersections empty).
    buffers_placed = 0;
    for (const GridIntersection& g : grid_intersections_) {
      if (g.has_buffer) buffers_placed++;
    }

    std::string cts_net_name = mesh_net_name_.empty() ? clock_name
                                                      : mesh_net_name_;
    buildCtsTreeToBuffers(cts_net_name, cts_buffer_list);
  }

  mesh_generated_ = true;

  // Count distinct grid lines (a blockage-clipped line becomes multiple segments)
  std::set<int> h_positions, v_positions;
  for (const MeshWire& w : h_wires)
    h_positions.insert((w.rect.yMin() + w.rect.yMax()) / 2);
  for (const MeshWire& w : v_wires)
    v_positions.insert((w.rect.xMin() + w.rect.xMax()) / 2);

  logger_->info(CMS, 120,
      "Created clock mesh: {} horizontal lines x {} vertical lines "
      "({} H-segments, {} V-segments), "
      "{} intersections, {} vias, {} buffers placed",
      h_positions.size(), v_positions.size(),
      h_wires.size(), v_wires.size(),
      grid_intersections_.size(), vias.size(), buffers_placed);
}

// Creates BTerm pins for sinks at grid intersections for router connections
void ClockMesh::connectSinksViaRouter(const std::string& clock_name, odb::dbTechLayer* proxy_layer)
{
  if (!mesh_generated_) {
    logger_->error(CMS, 500, "Mesh not generated. Call create_clock_mesh first.");
    return;
  }

  if (!proxy_layer) {
    logger_->error(CMS, 502, "Proxy layer not specified for sink BTerms");
    return;
  }

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(CMS, 501, "Could not find mesh net for clock '{}'", clock_name);
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
    odb::Point grid_point = findNearestGridWire(sink_point, h_wires, v_wires, &grid_layer);

    if (!grid_layer) {
      logger_->warn(CMS, 504, "No grid point found for sink {}", sink.name);
      continue;
    }

    // Safety net: snap point must not land inside a blockage
    if (isBlocked(grid_point.x(), grid_point.y())) {
      logger_->warn(CMS, 511,
                    "Sink {} snap point ({}, {}) is inside a blockage, skipping",
                    sink.name, grid_point.x(), grid_point.y());
      continue;
    }

    // Place BTERM on the mesh grid layer for direct connectivity
    int min_width = grid_layer->getWidth();
    int half_width = min_width / 2;

    int grid_x = grid_point.x();
    int grid_y = grid_point.y();

    // If another pin already exists at this position, offset along the wire
    bool is_h_wire = (grid_layer == mesh_h_layer_);
    auto pos_key = std::make_tuple(grid_x, grid_y, grid_layer->getRoutingLevel());
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
      logger_->warn(CMS, 505, "Failed to create net '{}' for sink {}", net_name, sink.name);
      continue;
    }
    sink_net->setSigType(odb::dbSigType::CLOCK);

    std::string bterm_name = "sink_bterm_" + std::to_string(sink_bterm_counter_);
    odb::dbBTerm* bterm = odb::dbBTerm::create(sink_net, bterm_name.c_str());
    if (!bterm) {
      logger_->warn(CMS, 506, "Failed to create BTerm '{}' for sink {}", bterm_name, sink.name);
      continue;
    }
    bterm->setIoType(odb::dbIoType::INPUT);
    bterm->setSigType(odb::dbSigType::CLOCK);

    odb::dbBPin* bpin = odb::dbBPin::create(bterm);
    if (bpin) {
      odb::dbBox::create(bpin, grid_layer,
                         grid_x - half_width, grid_y - half_width,
                         grid_x + half_width, grid_y + half_width);
      bpin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    // Store connection point so convertSWireToWire can break mesh wire here
    mesh_connection_points_.insert(
        std::make_tuple(grid_x, grid_y, grid_layer->getRoutingLevel()));

    // Remember the sink bterm name at this mesh-net coord on BOTH mesh
    // layers so cross-layer mesh-stripe junctions at the intersection merge
    // electrically with the sub-net stub in the SPICE deck.
    proxy_alias_[std::make_tuple(grid_x, grid_y, mesh_h_layer_->getRoutingLevel())]
        = bterm_name;
    if (mesh_v_layer_) {
      proxy_alias_[std::make_tuple(grid_x, grid_y, mesh_v_layer_->getRoutingLevel())]
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
    int64_t d2 = dx*dx + dy*dy;
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
odb::Point ClockMesh::findNearestGridWire(const odb::Point& loc, const std::vector<MeshWire>& h_wires, const std::vector<MeshWire>& v_wires, odb::dbTechLayer** out_grid_layer)
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
    grid_point = (min_dist_h < min_dist_v) ?
                 odb::Point(loc.x(), best_y) : odb::Point(best_x, loc.y());
  }

  return grid_point;
}

// Creates a via stack between two layers at a given location
void ClockMesh::createViaStackAtPoint(const odb::Point& location, odb::dbTechLayer* from_layer, odb::dbTechLayer* to_layer, odb::dbNet* net)
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
odb::dbTechLayer* ClockMesh::selectBufferLayer(odb::dbTechLayer* h_layer, odb::dbTechLayer* v_layer)
{
  if (!h_layer) return v_layer;
  if (!v_layer) return h_layer;

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
                                     const std::string& name, int x, int y,
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

void ClockMesh::placeBuffersAtIntersections(const std::string& buffer_master, odb::dbNet* mesh_net)
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
    logger_->warn(CMS, 730,
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
  const int spacing = std::max(mesh_v_layer_ ? mesh_v_layer_->getSpacing() : 0, 1);
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
  int tsv_nudged = 0, tsv_dropped = 0;

  for (GridIntersection& inter : grid_intersections_) {
    // Snap to the nearest row so the row-tall cells sit between that row's BPR
    // rails (the rails the Stage-2 break removes at the TSV).
    int row_y = inter.y;
    odb::dbOrientType orient;
    const bool snapped = nearestRow(inter.y, row_y, orient);

    // Place the TSV so its backside Y pad sits ON the mesh node (inter.x = the
    // vertical clock stripe). The Y->mesh connection then becomes a via/abutment
    // at the node instead of a routed backside stub (backside routing is the
    // scarce resource). The stripe is already >=0.056 clear of straps, and the
    // TSV's backside metal lives within the Y-pad region, so it stays clear too.
    int tsv_x = inter.x;
    if (tsv_master) {
      const int want = inter.x - ypad_cx;  // Y-pad center on the mesh node
      // Accept x only if the backside Y-pad region stays >= spacing from every
      // vertical strap AND the full footprint overlaps no fixed cell (tap/macro).
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
      for (int d = 0; d <= max_nudge && chosen == INT_MIN; d += site_w) {
        if (fits(want + d)) {
          chosen = want + d;
        } else if (d != 0 && fits(want - d)) {
          chosen = want - d;
        }
      }
      if (chosen == INT_MIN) {
        ++tsv_dropped;
        logger_->warn(CMS, 731,
                      "No tap-clear/PDN-clear spot for TSV near ({}, {}); "
                      "skipping its front<->back crossing",
                      inter.x, inter.y);
      } else {
        if (chosen != want) {
          ++tsv_nudged;
        }
        tsv_x = chosen;
        inter.tsv_inst = placeTsvCell(
            tsv_master,
            "tsv_buf_" + std::to_string(inter.x) + "_" + std::to_string(inter.y),
            tsv_x, row_y, orient);
        // future TSVs must avoid this one too
        fixed_rects.emplace_back(tsv_x, row_y, tsv_x + tsv_w, row_y + tsv_h);
      }
    }

    // Buffer goes beside the TSV (to its right). It's a frontside cell, so its
    // output reaches TSV.A on the frontside and it needs no backside room.
    const int buf_x =
        tsv_master ? (tsv_x + static_cast<int>(tsv_master->getWidth())) : inter.x;
    std::string buf_name = "mesh_buf_" + std::to_string(inter.x) +"_" + std::to_string(inter.y);
    odb::dbInst* buf_inst = odb::dbInst::create(block_, master, buf_name.c_str());
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
    logger_->info(CMS, 732,
                  "TSV placement: {} nudged off fixed cells (taps/macros), "
                  "{} dropped (no clear spot)",
                  tsv_nudged, tsv_dropped);
  }
}

// Connects buffer inputs to the original clock net
void ClockMesh::connectBuffersToNets(odb::dbNet* mesh_net, const std::string& clock_name)
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
void ClockMesh::setupProxyBTerms(const std::string& clock_name, odb::dbTechLayer* proxy_layer)
{
  if (!mesh_generated_) {
    logger_->error(CMS, 600, "Mesh not generated. Call create_clock_mesh first.");
    return;
  }

  if (!proxy_layer) {
    logger_->error(CMS, 601, "Proxy layer not specified");
    return;
  }

  proxy_layer_ = proxy_layer;

  if (bterm_layer_ && proxy_layer != bterm_layer_) {
    logger_->warn(CMS, 603, "Proxy layer {} differs from auto-computed BTERM layer {}", proxy_layer->getName(), bterm_layer_->getName());
  }

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(CMS, 602, "Could not find mesh net for clock '{}'", clock_name);
    return;
  }

  int created = createProxyBTermsWithSeparateNets(mesh_net, proxy_layer_);
  logger_->info(CMS, 610, "Created {} proxy BTERMs for buffer outputs", created);
}

// Creates separate nets and BTerm pins for each buffer at intersections
int ClockMesh::createProxyBTermsWithSeparateNets(odb::dbNet* mesh_net, odb::dbTechLayer* /* proxy_layer */)
{
  if (!mesh_net) {
    return 0;
  }
  // Place buffer BTERMs on the bottom mesh layer for direct connectivity
  int h_level = mesh_h_layer_ ? mesh_h_layer_->getRoutingLevel() : 0;
  int v_level = mesh_v_layer_ ? mesh_v_layer_->getRoutingLevel() : 0;
  odb::dbTechLayer* buf_bterm_layer = (h_level <= v_level) ? mesh_h_layer_ : mesh_v_layer_;
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
    std::string net_name = base_name + "_buf_" + std::to_string(inter.x) + "_" + std::to_string(inter.y);
    odb::dbITerm* output_pin = getBufferOutputPin(inter.buffer_inst);

    // Net the mesh BTerm lives on. With a TSV crossing this is the BACKSIDE
    // net (TSV.Y + BTerm) and the buffer output goes on a separate FRONTSIDE
    // net (buffer out + TSV.A). Without a TSV the buffer output connects to the
    // BTerm net directly (original mesh behavior).
    odb::dbNet* bterm_net = nullptr;

    if (inter.tsv_inst) {
      odb::dbITerm* tsvA = inter.tsv_inst->findITerm("A");   // frontside (M1)
      odb::dbITerm* tsvY = inter.tsv_inst->findITerm("Y");   // backside  (BM1)

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
                             + std::to_string(inter.x) + "_" + std::to_string(inter.y);
    odb::dbBTerm* bterm = odb::dbBTerm::create(bterm_net, bterm_name.c_str());
    if (!bterm) {
      continue;
    }
    bterm->setIoType(odb::dbIoType::INPUT);
    bterm->setSigType(odb::dbSigType::CLOCK);

    odb::dbBPin* bpin = odb::dbBPin::create(bterm);
    if (bpin) {
      odb::dbBox::create(bpin, buf_bterm_layer, inter.x - half_width, inter.y - half_width, inter.x + half_width, inter.y + half_width);
      bpin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    // Store connection point so convertSWireToWire can break mesh wire here
    mesh_connection_points_.insert(
        std::make_tuple(inter.x, inter.y, buf_bterm_layer->getRoutingLevel()));

    // Remember the proxy bterm name at this mesh-net coord on BOTH mesh
    // layers (H and V). The mesh-stripe path on each layer has its own
    // junction at this coord; aliasing both to the same bterm name lets the
    // SPICE deck merge them electrically across layers at the intersection.
    proxy_alias_[std::make_tuple(inter.x, inter.y, mesh_h_layer_->getRoutingLevel())]
        = bterm_name;
    if (mesh_v_layer_) {
      proxy_alias_[std::make_tuple(inter.x, inter.y, mesh_v_layer_->getRoutingLevel())]
          = bterm_name;
    }

    inter.proxy_bterm = bterm;
    created_count++;
  }
  return created_count;
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

  logger_->info(CMS, 625, "BTERMs on mesh layers: {} buffer, {} sink (no via stacks needed)", buffer_bterms, sink_bterms);
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
    logger_->warn(CMS, 401, "Failed to set clock net '{}' for CTS",
                  tree_net_name);
    return;
  }

  // Build the buffer list string. Empty string triggers TritonCTS auto-inference.
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
void ClockMesh::reencodeWireToMesh(odb::dbWire* src_wire, odb::dbWireEncoder& encoder)
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
          logger_->warn(CMS, 873, "Junction ID {} not found in source wire, " "starting disjoint path on layer {}", src_jct, layer->getName());
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
          logger_->warn(CMS, 874, "Short junction ID {} not found, " "starting disjoint path on layer {}", src_jct, layer->getName());
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
          logger_->warn(CMS, 875, "VWire junction ID {} not found, " "starting disjoint path on layer {}", src_jct, layer->getName());
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
  struct WireRect {
    odb::dbTechLayer* layer;
    odb::Rect rect;
    bool is_horizontal;
  };
  struct ViaEntry {
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
        odb::Rect rect(sbox->xMin(), sbox->yMin(),
                       sbox->xMax(), sbox->yMax());
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
      cx_end   = wr.rect.xMax();
      cy_start = cy_end = cy;
    } else {
      int cx = (wr.rect.xMin() + wr.rect.xMax()) / 2;
      cx_start = cx_end = cx;
      cy_start = wr.rect.yMin();
      cy_end   = wr.rect.yMax();
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

  logger_->info(CMS, 852, "Converted {} wire segments and {} vias from SWire to Wire on '{}'", wire_rects.size(), via_entries.size(), mesh_net_name);
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
    std::string leaf_key = std::string(inter.buffer_inst->getConstName())
        + "/" + std::string(input_iterm->getMTerm()->getConstName());
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
        slew_sec = sta_->slew(vertex, sta::RiseFallBoth::rise(),
                              sta_->scenes(), sta::MinMax::max());
      }
      leaf_slews_ns_[leaf_key] = slew_sec * 1e9;
      logger_->info(CMS, 873,
                    "Buffer {} arrival at pin {} on net '{}': {:.6g} ns, slew {:.6g} ns",
                    inter.buffer_inst->getConstName(),
                    arrival_pin_name,
                    input_iterm->getNet()->getConstName(),
                    arrival_sec * 1e9, slew_sec * 1e9);
    } else {
      leaf_arrivals_ns_[leaf_key] = 0.0;
      leaf_slews_ns_[leaf_key]    = 0.0;
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
  //                 "Captured {} CTS leaf arrivals, range: [{:.4g}, {:.4g}] ns, "
  //                 "CTS skew: {:.4g} ns",
  //                 leaf_arrivals_ns_.size(), min_arr, max_arr,
  //                 max_arr - min_arr);
  // } else {
  //   logger_->warn(CMS, 869, "No CTS leaf arrivals captured");
  // }
}

// Reads extracted parasitics from DB and writes SPICE netlist
void ClockMesh::writeMeshSpice(const std::string& clock_name, const std::string& spice_file, float vdd_voltage, float rise_time_ns, float fall_time_ns, const std::vector<std::string>& spice_models, bool zero_delay, bool full_tree, bool finfet)
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
      logger_->warn(CMS, 865, "Could not detect VDD from Liberty, using default {}V", vdd);
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

  logger_->info(CMS, 866,
                "SPICE parameters: VDD={:.3g}V, period={:.4g}ns, "
                "rise={:.4g}ns, fall={:.4g}ns",
                vdd, period_ns, rise_ns, fall_ns);

  // Characters like $, [, ], . are invalid in SPICE node names
  auto sanitize = [](const std::string& name) -> std::string {
    std::string s = name;
    for (char& c : s) {
      if (c == '$' || c == '[' || c == ']' || c == '.' || c == '/'
          || c == '\\' || c == ' ') {
        c = '_';
      }
    }
    return s;
  };

  // Build junction-id → (x, y, layer-level) map for the mesh net's dbWire.
  // Used to alias mesh-net internal nodes at proxy-bterm positions to the
  // bterm names so sub-net stubs electrically merge with mesh stripes when
  // merge_mesh_nets was skipped.
  std::map<int, std::tuple<int,int,int>> mesh_jid_to_coord;
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
      std::string leaf_key = std::string(inter.buffer_inst->getConstName())
          + "/" + std::string(input_iterm->getMTerm()->getConstName());
      if (!leaf_arrivals_ns_.count(leaf_key)) {
        leaf_arrivals_ns_[leaf_key] = 0.0;
        filled_zero++;
      }
    }
  }
  if (capture_was_empty) {
    logger_->warn(CMS, 870, "No cached CTS leaf arrivals. Call capture_mesh_arrivals before merge_mesh_nets for accurate skew analysis. Using zero delays for all {} buffers.", filled_zero);
  } else if (filled_zero > 0) {
    logger_->warn(CMS, 871, "Partial CTS leaf arrivals: {} buffers were missing from the captured map and assigned zero delay. Skew numbers for those buffers' downstream sinks will be optimistic.", filled_zero);
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
      if (!inter.has_buffer || !inter.buffer_inst) continue;
      worklist.push_back(inter.buffer_inst);
    }
    while (!worklist.empty()) {
      odb::dbInst* inst = worklist.back(); worklist.pop_back();
      for (odb::dbITerm* it : inst->getITerms()) {
        if (it->getIoType() != odb::dbIoType::INPUT) continue;
        // Check the NET's sig type (cell pin MTerm is often SIGNAL even for
        // clock cells), and exclude power/ground nets.
        odb::dbNet* drv_net = it->getNet();
        if (!drv_net || drv_net == mesh_net) continue;
        odb::dbSigType nst = drv_net->getSigType();
        if (nst == odb::dbSigType::POWER || nst == odb::dbSigType::GROUND) continue;
        if (!tree_nets.insert(drv_net).second) continue;
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
    logger_->info(CMS, 893,
                  "full_tree: walked back {} tree nets, {} tree buffer "
                  "instances, {} root bterm(s)",
                  tree_nets.size(), tree_buffers.size(), root_bterms.size());
  }

  // If zero_delay requested, build a zeroed copy of arrivals (leaves original intact)
  std::map<std::string, float> arrivals_for_spice;
  if (zero_delay) {
    for (const auto& [k, v] : leaf_arrivals_ns_) {
      arrivals_for_spice[k] = 0.0f;
    }
    logger_->info(CMS, 872, "zero_delay mode: all {} mesh buffer arrival times set to 0.", arrivals_for_spice.size());
    out << "\n* CTS buffer clock sources (zero-delay mode: all arrivals forced to 0)\n";
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
  int vclk_count = 0;
  if (full_tree) {
    // Single root Vclk
    for (odb::dbBTerm* bt : root_bterms) {
      std::string root_node = sanitize(bt->getConstName());
      out << "Vclk_root_" << vclk_count << " " << root_node << " 0 PULSE(0 "
          << vdd << " 0n " << rise_ns << "n " << fall_ns << "n "
          << pw_ns << "n " << period_ns << "n)"
          << " $ root bterm=" << bt->getConstName() << "\n";
      vclk_count++;
    }
    // No per-buffer Vclk; buffer input pins are driven by the tree.
  } else {
  for (const auto& [leaf_key, arrival] : arrivals_for_spice) {
    std::string spice_node = "clk_in_" + sanitize(leaf_key);
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
        << pw_ns << "n " << period_ns << "n)"
        << " $ " << leaf_key << " arrival=" << arrival
        << "ns slew=" << vclk_rise << "ns\n";
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
        if (st == odb::dbSigType::POWER) { out << " VDD"; return; }
        if (st == odb::dbSigType::GROUND) { out << " 0"; return; }
        odb::dbNet* nn = it->getNet();
        if (!nn) { out << " 0"; return; }
        // For root-bterm-driven net: use the bterm name (matches the Vclk).
        for (odb::dbBTerm* bt : nn->getBTerms()) {
          if (root_bterms.count(bt)) {
            out << " " << sanitize(bt->getConstName()); return;
          }
        }
        // Otherwise use inst_pin name (matches the iterm CapNode in the net's
        // extracted parasitics).
        out << " " << sanitize(std::string(it->getInst()->getConstName())
                               + "_"
                               + std::string(it->getMTerm()->getConstName()));
      };
      auto it_subckt = subckt_pins.find(master_name_t);
      if (it_subckt != subckt_pins.end()) {
        for (const std::string& cdl_pin : it_subckt->second) {
          auto pit = pin_to_iterm_t.find(cdl_pin);
          if (pit != pin_to_iterm_t.end()) write_t_pin(pit->second);
          else out << " " << cdl_pin;
        }
      } else {
        for (odb::dbITerm* it : tinst->getITerms()) write_t_pin(it);
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
          net_name_str = iterm_net->getConstName();
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
  for (odb::dbNet* n : block_->getNets()) {
    if (n == mesh_net) continue;
    std::string nm = n->getName();
    if (nm.rfind(buf_prefix, 0) == 0) {
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
  logger_->info(CMS, 891,
                "Emitting SPICE parasitics from {} nets "
                "(mesh + sub-nets when merge_mesh_nets is skipped)",
                relevant_nets.size());

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
      if (!inter.has_buffer || !inter.buffer_inst) continue;
      std::string master = inter.buffer_inst->getMaster()->getConstName();
      // Parse "BUFxN_..." → extract N (also handles BUFx16f-style suffixes)
      size_t pos = master.find("BUFx");
      if (pos == std::string::npos) continue;
      size_t start = pos + 4;
      size_t end = start;
      while (end < master.size() && (master[end] >= '0' && master[end] <= '9')) end++;
      if (end == start) continue;
      int N = std::stoi(master.substr(start, end - start));
      double cin_fF  = 0.207 * N;
      double cout_fF = 2.85  * N;

      // Input/output node names match how the buffer X-instance was emitted.
      // Input: clk_in_<inst>_<inputpin>  (when !full_tree) — buf_input_spice_node has it.
      // Output: <inst>_Y                  (sanitized inst_name + "_Y")
      odb::dbITerm* in_pin  = getBufferInputPin (inter.buffer_inst);
      odb::dbITerm* out_pin = getBufferOutputPin(inter.buffer_inst);
      std::string in_node, out_node;
      if (in_pin) {
        auto it = buf_input_spice_node.find(in_pin->getId());
        in_node = (it != buf_input_spice_node.end()) ? it->second
                  : sanitize(std::string(inter.buffer_inst->getConstName()) + "_"
                             + in_pin->getMTerm()->getConstName());
      }
      if (out_pin) {
        out_node = sanitize(std::string(inter.buffer_inst->getConstName()) + "_"
                            + out_pin->getMTerm()->getConstName());
      }
      if (!in_node.empty()) {
        out << "Ccin_"  << inter.buffer_inst->getConstName() << " "
            << in_node  << " 0 " << cin_fF  << "f\n";
      }
      if (!out_node.empty()) {
        out << "Ccout_" << inter.buffer_inst->getConstName() << " "
            << out_node << " 0 " << cout_fF << "f\n";
      }
      n_finfet++;
    }
    logger_->info(CMS, 894,
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
      out << "R" << r_count << " " << node_name(src_node)
          << " " << node_name(tgt_node) << " " << res << "\n";
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
        out << "C" << c_count << " " << node_name(cap_node)
            << " 0 " << cap * 1e-15 << "\n";
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
          out << "C" << c_count << " " << node_name(tgt_node)
              << " 0 " << cap * 1e-15 << "\n";
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
          out << "Cc" << cc_count << " " << node_name(src)
              << " " << node_name(tgt) << " " << cap * 1e-15 << "\n";
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
      if (s.iterm) real_sink_iterm_ids.insert(s.iterm->getId());
    }
  }

  // Sink gate-cap injection. OpenRCX extracts wire parasitics only — the
  // input-pin (gate) cap of each flop is a Liberty value, not in the RSeg/
  // CapNode network. Without these, sinks are purely resistive loads and
  // transition essentially instantly, giving unrealistically small skew.
  // Iterate sink iterms across all relevant nets (mesh + sub-nets).
  out << "\n* Sink pin caps (Liberty input-pin capacitance, one per sink ITerm)\n";
  int sink_cap_count = 0;
  std::set<uint32_t> seen_sink_iterms;
  for (odb::dbNet* net : relevant_nets) {
    for (odb::dbITerm* iterm : net->getITerms()) {
      std::string inst_name = iterm->getInst()->getConstName();
      if (inst_name.find("mesh_buf_") == 0) continue;
      // In full_tree mode, also skip upstream CTS tree-buffer instances —
      // they're drivers, not sinks.
      if (tree_buffers.count(iterm->getInst())) continue;
      // Only emit for authoritative sink iterms (the SDC-defined flop CLK
      // pins from clockToSinks_). This skips CTS infrastructure pins
      // (clkbuf_*, clkload*, anonymous _NNNN_) that share the clock net
      // but aren't real sinks.
      if (!real_sink_iterm_ids.empty()
          && !real_sink_iterm_ids.count(iterm->getId())) continue;
      if (seen_sink_iterms.count(iterm->getId())) continue;
      seen_sink_iterms.insert(iterm->getId());
      std::string sink_node;
      for (odb::dbCapNode* cn : net->getCapNodes()) {
        if (cn->isITerm() && cn->getNode() == iterm->getId()) {
          sink_node = node_name(cn);
          break;
        }
      }
      if (sink_node.empty()) {
        sink_node = sanitize(inst_name + "_" + std::string(iterm->getMTerm()->getConstName()));
      }
      // Liberty input-pin cap (farads) for this (master, pin).
      // Use MAX corner to match ISPD/clocksyn convention (its sink-cap
      // generator calls timing.getPortCap(..., timing.Max) — see
      // /home/wali2/mesh/ispd/save_asap7_ispd.py:143). Using nominal/default
      // capacitance() instead gave ~8.7% lower per-sink cap than clocksyn.
      sta::LibertyCell* lcell =
          network_->libertyCell(network_->dbToSta(iterm->getInst()->getMaster()));
      double pin_cap_f = 0.0;
      if (lcell) {
        sta::LibertyPort* port =
            lcell->findLibertyPort(iterm->getMTerm()->getConstName());
        if (port) pin_cap_f = port->capacitance(sta::MinMax::max());
      }
      if (pin_cap_f > 0.0) {
        out << "Csink" << sink_cap_count << " " << sink_node
            << " 0 " << pin_cap_f << "\n";
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
      if (inst_name.find("mesh_buf_") == 0) continue;
      // In full_tree mode, also skip upstream CTS tree-buffer instances —
      // they're drivers, not sinks.
      if (tree_buffers.count(iterm->getInst())) continue;
      // Only emit for authoritative sink iterms (the SDC-defined flop CLK
      // pins from clockToSinks_). This skips CTS infrastructure pins
      // (clkbuf_*, clkload*, anonymous _NNNN_) that share the clock net
      // but aren't real sinks.
      if (!real_sink_iterm_ids.empty()
          && !real_sink_iterm_ids.count(iterm->getId())) continue;
      if (seen_measure_iterms.count(iterm->getId())) continue;
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
        sink_node = sanitize(inst_name + "_"  + std::string(iterm->getMTerm()->getConstName()));
      }
      out << ".measure tran t_sink_" << sink_count
          << " WHEN v(" << sink_node << ")=" << half_vdd
          << " RISE=1"
          << "\n* " << inst_name << "/" << iterm->getMTerm()->getConstName()
          << "\n";
      sink_count++;
    }
  }
  logger_->info(CMS, 868,
                "Added {} sink .measure statements for skew analysis",
                sink_count);

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
  out << ".measure tran avg_power AVG power FROM="
      << (tran_stop - period_ns) << "n TO=" << tran_stop << "n\n";
  out << ".end\n";

  out.close();

  logger_->info(CMS, 862,"Wrote SPICE netlist: {} ({} R, {} C, {} Cc, {} sinks)", spice_file, r_count, c_count, cc_count, sink_count);
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

    std::vector<odb::dbITerm*> iterms(net->getITerms().begin(), net->getITerms().end());
    for (odb::dbITerm* iterm : iterms) {
      iterm->disconnect();
      iterm->connect(mesh_net);
    }
    odb::dbNet::destroy(net);
    buffers_merged++;
  }

  for (odb::dbNet* net : sink_nets) {
    net->setDoNotTouch(false);

    std::vector<odb::dbITerm*> iterms(net->getITerms().begin(), net->getITerms().end());
    for (odb::dbITerm* iterm : iterms) {
      iterm->disconnect();
      iterm->connect(mesh_net);
    }
    odb::dbNet::destroy(net);
    sinks_merged++;
  }

  logger_->info(CMS, 801,
                "Merged {} buffer nets and {} sink nets into '{}'",
                buffers_merged, sinks_merged, mesh_net_name);

  // Rebuild the STA network view from the current DB state.
  // write_verilog reads from the STA network (not OpenDB directly),
  // and the BTerm destruction callbacks only disconnect pins in STA
  // but don't remove ports from the top-level cell.
  network_->readDbAfter(db_);
  logger_->info(CMS, 802, "Rebuilt STA network after merge");
}

// Writes Verilog with buffer and sink nets merged to mesh net
void ClockMesh::writeMeshVerilog(const std::string& clock_name, const std::string& input_filename, const std::string& output_filename)
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
    if (name.rfind("sink_", 0) == 0 && name.find("bterm") == std::string::npos) {
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

  std::vector<std::string> sorted_nets(nets_to_replace.begin(), nets_to_replace.end());
  std::sort(sorted_nets.begin(), sorted_nets.end(),
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
  final_content += "// " + buf_net_prefix + "* and sink_* nets merged to '" + mesh_net_target + "'\n";
  final_content += "// CTS tree on original clock net '" + base_name + "' preserved\n";
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
