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
#include "odb/dbWireCodec.h"
#include "ord/OpenRoad.hh"
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

  // Find the root clock net name from SDC and use it as base for everything
  if (mesh_net_name_.empty()) {
    sta::Sdc* sdc = sta_->sdc();
    if (sdc) {
      for (auto clk : *sdc->clocks()) {
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
  if (bterm_layer) {
    bterm_layer_ = bterm_layer;
  }

  // Align grid to the actual mesh layer tracks (generalized for any tech).
  // Horizontal wires on h_layer: y-positions must align with h_layer tracks.
  // Vertical wires on v_layer: x-positions must align with v_layer tracks.
  int h_track_pitch = 0, h_track_offset = 0;
  int v_track_pitch = 0, v_track_offset = 0;

  if (h_layer) {
    odb::dbTrackGrid* h_tracks = block_->findTrackGrid(h_layer);
    if (h_tracks) {
      std::vector<int> y_tracks;
      h_tracks->getGridY(y_tracks);
      if (y_tracks.size() >= 2) {
        h_track_pitch = y_tracks[1] - y_tracks[0];
        h_track_offset = y_tracks[0];
      }
    }
    if (h_track_pitch == 0) {
      logger_->warn(CMS, 211, "No y-tracks found for horizontal mesh layer {}", h_layer->getName());
    }
  }

  if (v_layer) {
    odb::dbTrackGrid* v_tracks = block_->findTrackGrid(v_layer);
    if (v_tracks) {
      std::vector<int> x_tracks;
      v_tracks->getGridX(x_tracks);
      if (x_tracks.size() >= 2) {
        v_track_pitch = x_tracks[1] - x_tracks[0];
        v_track_offset = x_tracks[0];
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

  // Align pitch to the coarser of the two layer track pitches
  int aligned_pitch = pitch;
  int max_track_pitch = std::max(h_track_pitch, v_track_pitch);
  if (max_track_pitch > 0) {
    aligned_pitch = ((pitch + max_track_pitch / 2) / max_track_pitch) * max_track_pitch;
    if (aligned_pitch < max_track_pitch) {
      aligned_pitch = max_track_pitch;
    }
  }

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
    placeBuffersAtIntersections(buffer_list[0], mesh_net);
    connectBuffersToNets(mesh_net, clock_name);
    buffers_placed = grid_intersections_.size();

    std::string cts_net_name = mesh_net_name_.empty() ? clock_name
                                                      : mesh_net_name_;
    buildCtsTreeToBuffers(cts_net_name, buffer_list);
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
      odb::dbBox::create(bpin, buf_bterm_layer, inter.x - half_width, inter.y - half_width, inter.x + half_width, inter.y + half_width);
      bpin->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    }

    // Store connection point so convertSWireToWire can break mesh wire here
    mesh_connection_points_.insert(
        std::make_tuple(inter.x, inter.y, buf_bterm_layer->getRoutingLevel()));

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

  // Use the user-specified buffer list first, fall back to auto-detect
  std::string clkbuf_list;
  if (!buffer_list.empty()) {
    for (const auto& buf : buffer_list) {
      if (!clkbuf_list.empty()) {
        clkbuf_list += " ";
      }
      clkbuf_list += buf;
    }
  } else {
    // Auto-detect: search for cells with "clkbuf" or "BUF" in name
    for (odb::dbLib* lib : db_->getLibs()) {
      for (odb::dbMaster* master : lib->getMasters()) {
        std::string name = master->getName();
        if (name.find("clkbuf_") != std::string::npos
            || name.find("CLKBUF") != std::string::npos) {
          if (!clkbuf_list.empty()) {
            clkbuf_list += " ";
          }
          clkbuf_list += name;
        }
      }
    }
  }

  if (!clkbuf_list.empty()) {
    triton_cts->setBufferList(clkbuf_list.c_str());
  } else {
    logger_->error(CMS, 402, "No clock buffers found. " "Specify buffers with -buffers option");
    return;
  }

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

  // Append grid geometry to the existing dbWire (which has merged routing)
  odb::dbWire* wire = mesh_net->getWire();
  if (!wire) {
    wire = odb::dbWire::create(mesh_net);
  }

  odb::dbWireEncoder encoder;
  encoder.append(wire);

  // Encode each wire segment, breaking at connection points so OpenRCX creates explicit nodes where routed wires meet mesh wires
  for (const auto& wr : wire_rects) {
    int cx_start, cy_start, cx_end, cy_end;
    if (wr.is_horizontal) {
      int cy = (wr.rect.yMin() + wr.rect.yMax()) / 2;
      cx_start = wr.rect.xMin();
      cx_end = wr.rect.xMax();
      cy_start = cy;
      cy_end = cy;
    } else {
      int cx = (wr.rect.xMin() + wr.rect.xMax()) / 2;
      cx_start = cx;
      cx_end = cx;
      cy_start = wr.rect.yMin();
      cy_end = wr.rect.yMax();
    }

    // Find connection points that fall on this wire segment
    int layer_level = wr.layer->getRoutingLevel();
    std::vector<int> break_coords;
    for (const auto& [px, py, plevel] : mesh_connection_points_) {
      if (plevel != layer_level) continue;
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

    encoder.newPath(wr.layer, odb::dbWireType::ROUTED);
    if (break_coords.empty()) {
      encoder.addPoint(cx_start, cy_start);
      encoder.addPoint(cx_end, cy_end);
    } else {
      // Sort and insert intermediate points at connection locations
      std::sort(break_coords.begin(), break_coords.end());
      encoder.addPoint(cx_start, cy_start);
      for (int coord : break_coords) {
        if (wr.is_horizontal) {
          encoder.addPoint(coord, cy_start);
        } else {
          encoder.addPoint(cx_start, coord);
        }
      }
      encoder.addPoint(cx_end, cy_end);
    }
  }

  // Encode each via
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

  for (const GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.buffer_inst) {
      continue;
    }
    odb::dbITerm* input_iterm = getBufferInputPin(inter.buffer_inst);
    if (!input_iterm || !input_iterm->getNet()) {
      continue;
    }
    std::string leaf_name = input_iterm->getNet()->getConstName();
    if (leaf_arrivals_ns_.count(leaf_name)) {
      continue;
    }
    sta::Pin* sta_pin = network_->dbToSta(input_iterm);
    if (sta_pin) {
      float arrival_sec = sta_->pinArrival(
          sta_pin, sta::RiseFall::rise(), sta::MinMax::max());
      leaf_arrivals_ns_[leaf_name] = arrival_sec * 1e9;
    } else {
      leaf_arrivals_ns_[leaf_name] = 0.0;
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
void ClockMesh::writeMeshSpice(const std::string& clock_name, const std::string& spice_file, float vdd_voltage, float rise_time_ns, float fall_time_ns, const std::vector<std::string>& spice_models)
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
  sta::Sdc* sdc = sta_->sdc();
  if (sdc) {
    for (auto clk : *sdc->clocks()) {
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
  out << ".option abstol=1e-10 reltol=0.003 vntol=1e-4\n";

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

  // Use cached CTS leaf arrival times (captured before merge)
  if (leaf_arrivals_ns_.empty()) {
    logger_->warn(CMS, 870, "No cached CTS leaf arrivals. " "Call capture_mesh_arrivals before merge_mesh_nets " "for accurate skew analysis. Using zero delays.");
    // Collect leaf net names with zero delay as fallback
    for (const GridIntersection& inter : grid_intersections_) {
      if (!inter.has_buffer || !inter.buffer_inst) {
        continue;
      }
      odb::dbITerm* input_iterm = getBufferInputPin(inter.buffer_inst);
      if (input_iterm && input_iterm->getNet()) {
        std::string leaf_name = input_iterm->getNet()->getConstName();
        if (!leaf_arrivals_ns_.count(leaf_name)) {
          leaf_arrivals_ns_[leaf_name] = 0.0;
        }
      }
    }
  }

  // Write per-leaf-net clock sources with STA arrival delay offsets
  out << "\n* CTS leaf clock sources (with STA arrival delays)\n";
  int vclk_count = 0;
  for (const auto& [leaf_name, arrival] : leaf_arrivals_ns_) {
    out << "Vclk_" << vclk_count << " " << leaf_name << " 0 PULSE(0 " << vdd
        << " " << arrival << "n " << rise_ns << "n " << fall_ns << "n "
        << pw_ns << "n " << period_ns << "n)"
        << " $ arrival=" << arrival << "ns\n";
    vclk_count++;
  }

  // Buffer instances
  out << "\n* Buffer instances\n";
  for (const GridIntersection& inter : grid_intersections_) {
    if (!inter.has_buffer || !inter.buffer_inst) {
      continue;
    }
    odb::dbInst* inst = inter.buffer_inst;
    std::string inst_name = inst->getCosntName();
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
      odb::dbNet* iterm_net = iterm->getNet();
      std::string net_name_str;
      if (iterm_net) {
        bool found = false;
        for (odb::dbCapNode* cn : mesh_net->getCapNodes()) {
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

  // Resistance segments
  out << "\n* Resistance segments\n";
  int r_count = 0;
  for (odb::dbRSeg* rseg : mesh_net->getRSegs()) {
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

  // Ground capacitances — try CapNode caps first (model-based extraction),
  out << "\n* Ground capacitances\n";
  int c_count = 0;
  for (odb::dbCapNode* cap_node : mesh_net->getCapNodes()) {
    double cap = cap_node->getCapacitance(0);
    if (cap > 0.0) {
      out << "C" << c_count << " " << node_name(cap_node)
          << " 0 " << cap << "f\n";
      c_count++;
    }
  }
  if (c_count == 0) {
    // Fallback: read cap from RSegs (LEF-RC mode stores cap here)
    for (odb::dbRSeg* rseg : mesh_net->getRSegs()) {
      double cap = rseg->getCapacitance(0);
      if (cap > 0.0) {
        odb::dbCapNode* tgt_node = rseg->getTargetCapNode();
        if (!tgt_node) {
          continue;
        }
        out << "C" << c_count << " " << node_name(tgt_node)
            << " 0 " << cap << "f\n";
        c_count++;
      }
    }
  }

  // Coupling capacitances
  out << "\n* Coupling capacitances\n";
  int cc_count = 0;
  std::set<uint32_t> visited_cc;
  for (odb::dbCapNode* cap_node : mesh_net->getCapNodes()) {
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
            << " " << node_name(tgt) << " " << cap << "f\n";
        cc_count++;
      }
    }
  }

  // Sink arrival time measurements
  out << "\n* Sink arrival time measurements (50% VDD, 1st rising edge)\n";
  out << "* Post-process: skew = max(t_sink_i) - min(t_sink_i)\n";
  float half_vdd = vdd / 2.0;
  int sink_count = 0;
  for (odb::dbITerm* iterm : mesh_net->getITerms()) {
    std::string inst_name = iterm->getInst()->getConstName();
    // Skip mesh buffer ITerms — they are not sinks
    if (inst_name.find("mesh_buf_") == 0) {
      continue;
    }
    // Find the CapNode for this sink ITerm to get its SPICE node name
    std::string sink_node;
    for (odb::dbCapNode* cn : mesh_net->getCapNodes()) {
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
  logger_->info(CMS, 868,
                "Added {} sink .measure statements for skew analysis",
                sink_count);

  // Simulation control — step = rise_time/10, duration = 3 periods
  // 3 periods is enough for .measure RISE=2 to trigger
  float tran_step = rise_ns / 10.0;
  float tran_stop = period_ns * 3.0;
  out << "\n* Simulation control\n";
  out << ".tran " << tran_step << "n " << tran_stop << "n\n";
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
