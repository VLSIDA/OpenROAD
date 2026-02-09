// SPDX-License-Identifier: BSD-3-Clause

#include "cms/ClockMesh.hh"
#include <algorithm>
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
#include "ord/OpenRoad.hh"
#include "sta/Liberty.hh"
#include "sta/Sdc.hh"
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
  sta::Sdc* sdc = sta_->sdc();

  if (!sdc) {
    logger_->warn(CMS, 4, "No SDC constraints found");
    return;
  }

  for (auto clk : *sdc->clocks()) {
    std::string clkName = clk->name();
    std::set<odb::dbNet*> clkNets;
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
void ClockMesh::findClockRoots(sta::Clock* clk, std::set<odb::dbNet*>& clockNets)
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
  sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
  sta::LibertyCell* libertyCell = network_->libertyCell(masterCell);
  if (!libertyCell) {
    return true;
  }
  if (inst->isBlock()) {
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
        std::string name = std::string(inst->getConstName()) + "/" +
                          std::string(mterm->getConstName());
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

  for (int y_pos = bbox.yMin(); y_pos <= bbox.yMax(); y_pos += pitch) {
    const int y_min = y_pos - half_width;
    const int y_max = y_pos + half_width;
    odb::Rect wire_rect(x_start, y_min, x_end, y_max);
    wires.emplace_back(layer, net, wire_rect, true);
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

  for (int x_pos = bbox.xMin(); x_pos <= bbox.xMax(); x_pos += pitch) {
    const int x_min = x_pos - half_width;
    const int x_max = x_pos + half_width;
    odb::Rect wire_rect(x_min, y_start, x_max, y_end);
    wires.emplace_back(layer, net, wire_rect, false);
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

  if (mesh_net_name_.empty()) {
    if (clockToSinks_.find(clock_name) != clockToSinks_.end()) {
      const auto& sinks = clockToSinks_[clock_name];
      if (!sinks.empty() && sinks[0].iterm) {
        odb::dbNet* orig_net = sinks[0].iterm->getNet();
        if (orig_net) {
          mesh_net_name_ = orig_net->getName();
        }
      }
    }
  }

  std::string mesh_net_name = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
  mesh_net_name += "_mesh";

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
void ClockMesh::createMeshGrid(const std::string& clock_name, odb::dbTechLayer* h_layer, odb::dbTechLayer* v_layer, int pitch, const std::vector<std::string>& buffer_list)
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

  odb::dbTech* tech = db_->getTech();
  int h_level = h_layer ? h_layer->getRoutingLevel() : 0;
  int v_level = v_layer ? v_layer->getRoutingLevel() : 0;
  int bterm_level = std::max(h_level, v_level) + 1;

  odb::dbTechLayer* bterm_layer = tech->findRoutingLayer(bterm_level);
  if (!bterm_layer) {
    logger_->error(CMS, 210, "No routing layer at level {} for BTERMs", bterm_level);
    return;
  }
  bterm_layer_ = bterm_layer;
  odb::dbTrackGrid* track_grid = block_->findTrackGrid(bterm_layer);
  int track_pitch_x = 0, track_pitch_y = 0;
  int track_offset_x = 0, track_offset_y = 0;
  if (track_grid) {
    std::vector<int> x_tracks, y_tracks;
    track_grid->getGridX(x_tracks);
    track_grid->getGridY(y_tracks);
    if (x_tracks.size() >= 2) {
      track_pitch_x = x_tracks[1] - x_tracks[0];
      track_offset_x = x_tracks[0];
    }
    if (y_tracks.size() >= 2) {
      track_pitch_y = y_tracks[1] - y_tracks[0];
      track_offset_y = y_tracks[0];
    }
  } else {
    logger_->warn(CMS, 211, "No track grid found for BTERM layer {}", bterm_layer->getName());
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

  int aligned_pitch = pitch;
  if (track_pitch_x > 0 && track_pitch_y > 0) {
    int track_pitch = std::max(track_pitch_x, track_pitch_y);
    aligned_pitch = ((pitch + track_pitch / 2) / track_pitch) * track_pitch;
    if (aligned_pitch < track_pitch) {
      aligned_pitch = track_pitch;
    }
  }
  int aligned_x_start = bbox.xMin();
  int aligned_y_start = bbox.yMin();
  if (track_pitch_x > 0) {
    int idx = (bbox.xMin() - track_offset_x + track_pitch_x / 2) / track_pitch_x;
    aligned_x_start = track_offset_x + idx * track_pitch_x;
  }
  if (track_pitch_y > 0) {
    int idx = (bbox.yMin() - track_offset_y + track_pitch_y / 2) / track_pitch_y;
    aligned_y_start = track_offset_y + idx * track_pitch_y;
  }
  odb::Rect aligned_bbox(aligned_x_start, aligned_y_start, bbox.xMax(), bbox.yMax());

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
        odb::dbTechLayer* buf_layer = selectBufferLayer(h_wire.layer, v_wire.layer);
        grid_intersections_.emplace_back(v_x, h_y, buf_layer);
      }
    }
  }

  mesh_wires_.clear();
  mesh_wires_.insert(mesh_wires_.end(), h_wires.begin(), h_wires.end());
  mesh_wires_.insert(mesh_wires_.end(), v_wires.begin(), v_wires.end());

  int buffers_placed = 0;
  if (!buffer_list.empty() && !grid_intersections_.empty()) {
    placeBuffersAtIntersections(buffer_list[0], mesh_net);
    connectBuffersToNets(mesh_net, clock_name);
    buffers_placed = grid_intersections_.size();

    std::string cts_net_name = mesh_net_name_.empty() ? clock_name : mesh_net_name_;
    buildCtsTreeToBuffers(cts_net_name);
  }

  mesh_generated_ = true;

  logger_->info(CMS, 120, "Created clock mesh: {} wires, {} vias, {} buffers",
                h_wires.size() + v_wires.size(), vias.size(), buffers_placed);
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

  std::map<std::pair<int, int>, int> gridpoint_bterm_count;
  int min_width = proxy_layer->getWidth();
  int half_width = min_width / 2;
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

    int grid_x = grid_point.x();
    int grid_y = grid_point.y();

    for (const GridIntersection& inter : grid_intersections_) {
      if (inter.x == grid_x && inter.y == grid_y && inter.has_buffer && inter.proxy_bterm) {
        int offset = min_width + half_width;
        if (sink_x != grid_x) {
          grid_x += (sink_x > grid_x) ? offset : -offset;
        } else if (sink_y != grid_y) {
          grid_y += (sink_y > grid_y) ? offset : -offset;
        } else {
          grid_x += offset;
        }
        break;
      }
    }

    auto grid_key = std::make_pair(grid_x, grid_y);
    auto it = gridpoint_bterm_count.find(grid_key);
    if (it != gridpoint_bterm_count.end()) {
      int offset_multiplier = it->second;
      int offset = (min_width + half_width) * offset_multiplier;
      if (sink_x != grid_x) {
        grid_x += (sink_x > grid_x) ? offset : -offset;
      } else if (sink_y != grid_y) {
        grid_y += (sink_y > grid_y) ? offset : -offset;
      } else {
        grid_x += (offset_multiplier % 2 == 0) ? offset : -offset;
      }
      it->second++;
    } else {
      gridpoint_bterm_count[grid_key] = 1;
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
      odb::dbBox::create(bpin, proxy_layer,
                         grid_x - half_width, grid_y - half_width,
                         grid_x + half_width, grid_y + half_width);
      bpin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    sink.iterm->disconnect();
    sink.iterm->connect(sink_net);
    bterms_created++;
  }

  logger_->info(CMS, 510, "Created {} sink BTerms for routing", bterms_created);
}

// Finds the nearest mesh grid wire to a given point
odb::Point ClockMesh::findNearestGridWire(const odb::Point& loc, const std::vector<MeshWire>& h_wires, const std::vector<MeshWire>& v_wires, odb::dbTechLayer** out_grid_layer)
{
  int best_x = loc.x();
  int best_y = loc.y();
  int min_dist_h = std::numeric_limits<int>::max();
  int min_dist_v = std::numeric_limits<int>::max();
  odb::dbTechLayer* nearest_h_layer = nullptr;
  odb::dbTechLayer* nearest_v_layer = nullptr;

  for (const auto& wire : h_wires) {
    int wire_y = (wire.rect.yMin() + wire.rect.yMax()) / 2;
    int dist = std::abs(wire_y - loc.y());
    if (dist < min_dist_h) {
      min_dist_h = dist;
      best_y = wire_y;
      nearest_h_layer = wire.layer;
    }
  }
  for (const auto& wire : v_wires) {
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

  for (GridIntersection& inter : grid_intersections_) {
    std::string buf_name = "mesh_buf_" + std::to_string(inter.x) +"_" + std::to_string(inter.y);
    odb::dbInst* buf_inst = odb::dbInst::create(block_, master, buf_name.c_str());
    if (buf_inst) {
      buf_inst->setLocation(inter.x, inter.y);
      buf_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      inter.buffer_inst = buf_inst;
      inter.has_buffer = true;
    }
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
int ClockMesh::createProxyBTermsWithSeparateNets(odb::dbNet* mesh_net, odb::dbTechLayer* proxy_layer)
{
  if (!mesh_net || !proxy_layer) {
    return 0;
  }
  int min_width = proxy_layer->getWidth();
  int half_width = min_width / 2;
  int created_count = 0;
  std::string base_name = mesh_net_name_.empty() ? "clk" : mesh_net_name_;

  for (GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.buffer_inst) {
      continue;
    }
    std::string net_name = base_name + "_buf_" + std::to_string(inter.x) + "_" + std::to_string(inter.y);
    odb::dbNet* buf_net = odb::dbNet::create(block_, net_name.c_str());
    if (!buf_net) {
      logger_->warn(CMS, 607, "Failed to create net '{}'", net_name);
      continue;
    }
    buf_net->setSigType(odb::dbSigType::CLOCK);

    odb::dbITerm* output_pin = getBufferOutputPin(inter.buffer_inst);
    if (output_pin) {
      output_pin->disconnect();
      output_pin->connect(buf_net);
    }

    std::string bterm_name = "proxy_" + std::to_string(inter.x) + "_" + std::to_string(inter.y);
    odb::dbBTerm* bterm = odb::dbBTerm::create(buf_net, bterm_name.c_str());
    if (!bterm) {
      continue;
    }
    bterm->setIoType(odb::dbIoType::INPUT);
    bterm->setSigType(odb::dbSigType::CLOCK);

    odb::dbBPin* bpin = odb::dbBPin::create(bterm);
    if (bpin) {
      odb::dbBox::create(bpin, proxy_layer, inter.x - half_width, inter.y - half_width, inter.x + half_width, inter.y + half_width);
      bpin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    inter.proxy_bterm = bterm;
    created_count++;
  }
  return created_count;
}

// Connects proxy BTerm locations to mesh with via stacks after routing
void ClockMesh::connectProxyBTermsToMesh(const std::string& clock_name)
{
  if (!proxy_layer_) {
    logger_->error(CMS, 620, "Proxy layer not set. Call setup_proxy_bterms first.");
    return;
  }

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(CMS, 621, "Could not find mesh net for clock '{}'", clock_name);
    return;
  }

  int proxy_level = proxy_layer_->getRoutingLevel();
  int h_level = mesh_h_layer_ ? mesh_h_layer_->getRoutingLevel() : 0;
  int v_level = mesh_v_layer_ ? mesh_v_layer_->getRoutingLevel() : 0;
  odb::dbTechLayer* top_mesh_layer = (h_level >= v_level) ? mesh_h_layer_ : mesh_v_layer_;
  int top_level = top_mesh_layer ? top_mesh_layer->getRoutingLevel() : 0;

  std::set<std::pair<int, int>> connected_locations;
  int buffer_bterms_connected = 0;

  for (const GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.proxy_bterm) {
      continue;
    }
    odb::Point target(inter.x, inter.y);
    if (top_mesh_layer && proxy_level > top_level) {
      createViaStackAtPoint(target, top_mesh_layer, proxy_layer_, mesh_net);
    }
    connected_locations.insert({inter.x, inter.y});
    buffer_bterms_connected++;
  }

  int sink_bterms_connected = 0;
  for (odb::dbNet* net : block_->getNets()) {
    std::string net_name = net->getName();
    if (net_name.rfind("sink_", 0) != 0) {
      continue;
    }

    odb::dbBTerm* bterm = net->get1stBTerm();
    if (!bterm) {
      continue;
    }

    for (odb::dbBPin* bpin : bterm->getBPins()) {
      for (odb::dbBox* box : bpin->getBoxes()) {
        int bterm_x = (box->xMin() + box->xMax()) / 2;
        int bterm_y = (box->yMin() + box->yMax()) / 2;

        if (connected_locations.count({bterm_x, bterm_y})) {
          continue;
        }

        odb::Point target(bterm_x, bterm_y);
        if (top_mesh_layer && proxy_level > top_level) {
          createViaStackAtPoint(target, top_mesh_layer, proxy_layer_, mesh_net);
        }
        connected_locations.insert({bterm_x, bterm_y});
        sink_bterms_connected++;
        break;
      }
      break;
    }
  }

  if (!connection_vias_.empty()) {
    writeViasToDb(connection_vias_);
    connection_vias_.clear();
  }

  logger_->info(CMS, 625, "Connected {} buffer and {} sink BTERMs to mesh",
                buffer_bterms_connected, sink_bterms_connected);
}

// Runs TritonCTS to build clock tree to buffer inputs
void ClockMesh::buildCtsTreeToBuffers(const std::string& tree_net_name)
{
  cts::TritonCTS* triton_cts = openroad_->getTritonCts();
  if (!triton_cts) {
    logger_->error(CMS, 400, "TritonCTS not available");
    return;
  }

  int result = triton_cts->setClockNets(tree_net_name.c_str());
  if (result != 0) {
    logger_->warn(CMS, 401, "Failed to set clock net '{}' for CTS", tree_net_name);
    return;
  }

  std::string clkbuf_list;
  for (odb::dbLib* lib : db_->getLibs()) {
    for (odb::dbMaster* master : lib->getMasters()) {
      std::string name = master->getName();
      if (name.find("clkbuf_") != std::string::npos) {
        if (!clkbuf_list.empty()) {
          clkbuf_list += " ";
        }
        clkbuf_list += name;
      }
    }
  }
  if (!clkbuf_list.empty()) {
    triton_cts->setBufferList(clkbuf_list.c_str());
  } else {
    logger_->warn(CMS, 402, "No clock buffers found in library");
  }

  triton_cts->runTritonCts();
}

// Writes Verilog with mesh nets merged back to clock net name
void ClockMesh::writeMeshVerilog(const std::string& clock_name, const std::string& input_filename, const std::string& output_filename)
{
  std::string mesh_net_name = mesh_net_name_;
  if (mesh_net_name.empty()) {
    logger_->warn(CMS, 753, "Mesh net name not stored, using SDC clock name");
    mesh_net_name = clock_name;
  }

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
  std::string buf_net_prefix = mesh_net_name + "_buf_";
  std::string mesh_net_suffix = mesh_net_name + "_mesh";

  for (odb::dbNet* net : block_->getNets()) {
    std::string name = net->getName();
    if (name.rfind(buf_net_prefix, 0) == 0) {
      nets_to_replace.insert(name);
    }
    if (name.rfind("sink_", 0) == 0 && name.find("bterm") == std::string::npos) {
      nets_to_replace.insert(name);
    }
    if (name == mesh_net_suffix) {
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
      content.replace(pos, net_name.length(), mesh_net_name);
      pos += mesh_net_name.length();
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
    std::string wire_pattern = " wire " + mesh_net_name + ";";
    if (!skip_line && line.find(wire_pattern) != std::string::npos) {
      skip_line = true;
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
  final_content += "// " + buf_net_prefix + "*, " + mesh_net_suffix + ", and sink_* nets merged to '" + mesh_net_name + "'\n";
  final_content += "// CTS tree on original clock net '" + mesh_net_name + "' preserved\n";
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
