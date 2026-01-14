// SPDX-License-Identifier: BSD-3-Clause

#include "mesh/ClockMesh.hh"
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include "cts/TritonCTS.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "odb/dbShape.h"
#include "ord/OpenRoad.hh"
#include "sta/Corner.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/Liberty.hh"
#include "sta/Sdc.hh"
#include "sta/TimingArc.hh"
#include "utl/Logger.h"

namespace mesh {
using utl::MESH;
ClockMesh::ClockMesh() = default;

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

  if (logger_) {
    logger_->report("ClockMesh initialized successfully");
  }
}

void ClockMesh::run(const char* name)
{
  if (openroad_ == nullptr || logger_ == nullptr) {
    return;
  }
  const char* mesh_name = (name != nullptr && name[0] != '\0') ? name : "default";
  logger_->report("ClockMesh running for clock: {}", mesh_name);

  findClockSinks();

  mesh_generated_ = true;
}


void ClockMesh::findClockSinks()
{
  if (db_) {
    odb::dbChip* chip = db_->getChip();
    if (chip) {
      block_ = chip->getBlock();
    }
  }
  if (!block_) {
    logger_->error(MESH, 1, "No block found in database");
    return;
  }
  if (!sta_ || !network_) {
    logger_->error(MESH, 2, "STA not initialized");
    return;
  }

  clockToSinks_.clear();
  visitedClockNets_.clear();
  staClockNets_ = sta_->findClkNets();
  sta::Sdc* sdc = sta_->sdc();

  if (!sdc) {
    logger_->warn(MESH, 4, "No SDC constraints found");
    return;
  }

  for (auto clk : *sdc->clocks()) {
    std::string clkName = clk->name();
    logger_->report("Found clock in SDC: {}", clkName);
    std::set<odb::dbNet*> clkNets;
    findClockRoots(clk, clkNets);
    for (odb::dbNet* net : clkNets) {
      if (net && visitedClockNets_.find(net) == visitedClockNets_.end()) {
        visitedClockNets_.insert(net);
        std::vector<ClockSink> sinks;
        if (separateSinks(net, sinks)) {
          clockToSinks_[clkName].insert(clockToSinks_[clkName].end(), sinks.begin(), sinks.end());
          logger_->report("Stored {} sinks for clock '{}'", sinks.size(), clkName);
        }
      }
    }
  }

  logger_->report("Total clocks with sinks: {}", clockToSinks_.size());
  for (const auto& [clk_name, sinks] : clockToSinks_) {
    logger_->report("  Clock '{}': {} sinks", clk_name, sinks.size());
  }
}

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

float ClockMesh::getInputPinCap(odb::dbITerm* iterm)
{
  odb::dbInst* inst = iterm->getInst();
  sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
  sta::LibertyCell* libertyCell = network_->libertyCell(masterCell);

  if (!libertyCell) {
    return 0.0;
  }

  sta::LibertyPort* inputPort =
      libertyCell->findLibertyPort(iterm->getMTerm()->getConstName());

  if (inputPort) {
    return inputPort->capacitance();
  }

  return 0.0;
}

bool ClockMesh::hasInsertionDelay(odb::dbInst* inst, odb::dbMTerm* mterm)
{
  sta::LibertyCell* libCell = network_->libertyCell(network_->dbToSta(inst));
  if (libCell) {
    sta::LibertyPort* libPort = libCell->findLibertyPort(mterm->getConstName());
    if (libPort) {
      const float rise = libPort->clkTreeDelay(
          0.0, sta::RiseFall::rise(), sta::MinMax::max());
      const float fall = libPort->clkTreeDelay(
          0.0, sta::RiseFall::fall(), sta::MinMax::max());

      if (rise != 0 || fall != 0) {
        return true;
      }
    }
  }
  return false;
}

double ClockMesh::computeInsertionDelay(const std::string& name,
                                        odb::dbInst* inst,
                                        odb::dbMTerm* mterm)
{
  if (!hasInsertionDelay(inst, mterm)) {
    return 0.0;
  }

  sta::LibertyCell* libCell = network_->libertyCell(network_->dbToSta(inst));
  if (!libCell) {
    return 0.0;
  }

  sta::LibertyPort* libPort = libCell->findLibertyPort(mterm->getConstName());
  if (!libPort) {
    return 0.0;
  }

  const float rise = libPort->clkTreeDelay(
      0.0, sta::RiseFall::rise(), sta::MinMax::max());
  const float fall = libPort->clkTreeDelay(
      0.0, sta::RiseFall::fall(), sta::MinMax::max());

  return (rise + fall) / 2.0;
}

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
        float cap = getInputPinCap(iterm);
        double insDelay = computeInsertionDelay(name, inst, mterm);
        bool isMacro = inst->isBlock();
        sinks.emplace_back(name, x, y, cap, insDelay, iterm, isMacro);
      }
    }
  }

  return !sinks.empty();
}

void ClockMesh::testPrintSinks()
{
  if (!logger_) {
    return;
  }

  logger_->report("TEST: Found {} clock(s)", clockToSinks_.size());

  for (const auto& [clockName, sinks] : clockToSinks_) {
    logger_->report("\nTEST: Clock '{}' has {} sinks:", clockName, sinks.size());
    int count = 0;
    for (const auto& sink : sinks) {
      logger_->report("  [{}] {} at ({}, {}) cap={:.3e} macro={}",
                     ++count, sink.name, sink.x, sink.y, sink.inputCap, sink.isMacro);
    }
  }
}

// Mesh Grid Creation Methods
odb::Rect ClockMesh::calculateBoundingBox(const std::vector<ClockSink>& sinks)
{
  if (sinks.empty()) {
    logger_->warn(MESH, 100, "No sinks provided for bounding box calculation");
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
    odb::Rect die_area = block_->getDieArea();
    odb::Rect core_area = block_->getCoreArea();

    logger_->report("Die area: ({}, {}) to ({}, {})",
                   die_area.xMin(), die_area.yMin(),
                   die_area.xMax(), die_area.yMax());
    logger_->report("Core area: ({}, {}) to ({}, {})",
                   core_area.xMin(), core_area.yMin(),
                   core_area.xMax(), core_area.yMax());

    bbox.set_xlo(std::min(bbox.xMin(), core_area.xMin()));
    bbox.set_ylo(std::min(bbox.yMin(), core_area.yMin()));
    bbox.set_xhi(std::max(bbox.xMax(), core_area.xMax()));
    bbox.set_yhi(std::max(bbox.yMax(), core_area.yMax()));
  }

  logger_->report("Bounding box calculated: ({}, {}) to ({}, {})",
                 bbox.xMin(), bbox.yMin(), bbox.xMax(), bbox.yMax());

  return bbox;
}

void ClockMesh::createHorizontalWires(odb::dbNet* net, odb::dbTechLayer* layer, const odb::Rect& bbox, int wire_width, int pitch, std::vector<MeshWire>& wires)
{
  if (!net || !layer) {
    logger_->error(MESH, 101, "Invalid net or layer for horizontal wires");
    return;
  }
  const int half_width = wire_width / 2;
  const int x_start = bbox.xMin();
  const int x_end = bbox.xMax();
  int wire_count = 0;
  logger_->report("Creating horizontal wires on layer {} from Y={} to Y={}, pitch={}", layer->getName(), bbox.yMin(), bbox.yMax(), pitch);
  for (int y_pos = bbox.yMin(); y_pos <= bbox.yMax(); y_pos += pitch) {
    const int y_min = y_pos - half_width;
    const int y_max = y_pos + half_width;
    odb::Rect wire_rect(x_start, y_min, x_end, y_max);
    wires.emplace_back(layer, net, wire_rect, true);
    wire_count++;
  }
  logger_->report("Created {} horizontal wires on layer {}", wire_count, layer->getName());
}

void ClockMesh::createVerticalWires(odb::dbNet* net, odb::dbTechLayer* layer, const odb::Rect& bbox, int wire_width, int pitch, std::vector<MeshWire>& wires)
{
  if (!net || !layer) {
    logger_->error(MESH, 102, "Invalid net or layer for vertical wires");
    return;
  }
  const int half_width = wire_width / 2;
  const int y_start = bbox.yMin();
  const int y_end = bbox.yMax();
  int wire_count = 0;
  logger_->report("Creating vertical wires on layer {} from X={} to X={}, pitch={}", layer->getName(), bbox.xMin(), bbox.xMax(), pitch);
  for (int x_pos = bbox.xMin(); x_pos <= bbox.xMax(); x_pos += pitch) {
    const int x_min = x_pos - half_width;
    const int x_max = x_pos + half_width;
    odb::Rect wire_rect(x_min, y_start, x_max, y_end);
    wires.emplace_back(layer, net, wire_rect, false);
    wire_count++;
  }

  logger_->report("Created {} vertical wires on layer {}", wire_count, layer->getName());
}

void ClockMesh::createViasAtIntersections(const std::vector<MeshWire>& h_wires, const std::vector<MeshWire>& v_wires, std::vector<MeshVia>& vias)
{
  if (h_wires.empty() || v_wires.empty()) {
    logger_->warn(MESH, 103, "No wires provided for via creation");
    return;
  }
  int via_count = 0;
  logger_->report("Creating vias at intersections: {} h_wires x {} v_wires", h_wires.size(), v_wires.size());

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
        via_count++;
      }
    }
  }
  logger_->report("Created {} vias at wire intersections", via_count);
}

odb::dbNet* ClockMesh::getOrCreateClockNet(const std::string& clock_name)
{
  if (!block_) {
    logger_->error(MESH, 104, "No block available for net creation");
    return nullptr;
  }

  if (clockToSinks_.find(clock_name) != clockToSinks_.end()) {
    const auto& sinks = clockToSinks_[clock_name];
    if (!sinks.empty() && sinks[0].iterm) {
      odb::dbNet* clock_net = sinks[0].iterm->getNet();
      if (clock_net) {
        logger_->report("Using existing clock net: {}", clock_net->getName());
        if (!clock_net->isSpecial()) {
          clock_net->setSpecial();
        }
        if (clock_net->getSigType() != odb::dbSigType::CLOCK) {
          clock_net->setSigType(odb::dbSigType::CLOCK);
        }
        return clock_net;
      }
    }
  }

  logger_->warn(MESH, 117, "Could not get clock net from sinks for '{}'", clock_name);
  for (const auto& [clk_name, sinks] : clockToSinks_) {
    if (!sinks.empty() && sinks[0].iterm) {
      odb::dbNet* clock_net = sinks[0].iterm->getNet();
      if (clock_net) {
        logger_->report("Found clock net '{}' from clock '{}' sinks",
                       clock_net->getName(), clk_name);
        if (!clock_net->isSpecial()) {
          clock_net->setSpecial();
        }
        if (clock_net->getSigType() != odb::dbSigType::CLOCK) {
          clock_net->setSigType(odb::dbSigType::CLOCK);
        }
        return clock_net;
      }
    }
  }

  logger_->error(MESH, 105, "Could not find clock net for '{}'", clock_name);
  return nullptr;
}

void ClockMesh::writeWiresToDb(const std::vector<MeshWire>& wires)
{
  if (!block_) {
    logger_->error(MESH, 106, "No block available for writing wires");
    return;
  }
  int written_count = 0;
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
      logger_->warn(MESH, 107, "Failed to get/create SWire for net {}", wire.net->getName());
      continue;
    }
    odb::dbSBox* sbox = odb::dbSBox::create(swire, wire.layer, wire.rect.xMin(), wire.rect.yMin(), wire.rect.xMax(), wire.rect.yMax(), odb::dbWireShapeType::STRIPE);
    if (sbox) {
      written_count++;
    }
  }
  logger_->report("Wrote {} wires to database", written_count);
}

void ClockMesh::writeViasToDb(const std::vector<MeshVia>& vias)
{
  if (!block_) {
    logger_->error(MESH, 109, "No block available for writing vias");
    return;
  }
  odb::dbTech* tech = db_->getTech();
  if (!tech) {
    logger_->error(MESH, 110, "No technology available");
    return;
  }
  int written_count = 0;

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

      odb::dbSBox* via_box = odb::dbSBox::create(swire, tech_via, via_x, via_y, odb::dbWireShapeType::NONE);
      if (via_box) {
        written_count++;
      }
    }
  }
  logger_->report("Wrote {} vias to database", written_count);
}


// Main Mesh Grid Creation
void ClockMesh::createMeshGrid(const std::string& clock_name, odb::dbTechLayer* h_layer, odb::dbTechLayer* v_layer, int wire_width, int pitch, const std::vector<std::string>& buffer_list, odb::dbTechLayer* tree_layer)
{
  if (!logger_ || !block_) {
    logger_->error(MESH, 113, "ClockMesh not properly initialized");
    return;
  }

  logger_->report("======================================");
  logger_->report("Creating clock mesh grid for clock: {}", clock_name);
  logger_->report("Horizontal layer: {}, Vertical layer: {}", h_layer ? h_layer->getName() : "NULL", v_layer ? v_layer->getName() : "NULL");
  logger_->report("Wire width: {}, Pitch: {}", wire_width, pitch);

  // if (!tree_layer) {
  //   odb::dbTech* tech = db_->getTech();
  //   int h_level = h_layer->getRoutingLevel();
  //   tree_layer = tech->findRoutingLayer(h_level + 1);
  //   if (!tree_layer) {
  //     tree_layer = h_layer;
  //   }
  // }
  // logger_->report("CTS tree layer: {}", tree_layer ? tree_layer->getName() : "NULL");

  // Store layers
  mesh_h_layer_ = h_layer;
  mesh_v_layer_ = v_layer;
  // tree_layer_ = tree_layer;

  // Get clock sinks
  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    logger_->error(MESH, 114, "No sinks found for clock: {}", clock_name);
    return;
  }

  const std::vector<ClockSink>& sinks = clockToSinks_[clock_name];
  logger_->report("Found {} sinks for clock {}", sinks.size(), clock_name);

  // Calculate bounding box
  odb::Rect bbox = calculateBoundingBox(sinks);
  if (bbox.area() == 0) {
    logger_->error(MESH, 115, "Invalid bounding box calculated");
    return;
  }
  mesh_bbox_ = bbox;
  mesh_wire_width_ = wire_width;
  mesh_pitch_ = pitch;

  // Get existing clock net
  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(MESH, 116, "Failed to find clock net for '{}'", clock_name);
    return;
  }

  // Create horizontal wires
  std::vector<MeshWire> h_wires;
  if (h_layer) {
    createHorizontalWires(mesh_net, h_layer, bbox, wire_width, pitch, h_wires);
  }

  // Create vertical wires
  std::vector<MeshWire> v_wires;
  if (v_layer) {
    createVerticalWires(mesh_net, v_layer, bbox, wire_width, pitch, v_wires);
  }

  // Create vias at intersections
  std::vector<MeshVia> vias;
  if (!h_wires.empty() && !v_wires.empty()) {
    createViasAtIntersections(h_wires, v_wires, vias);
  }

  // Write to database
  if (!h_wires.empty()) {
    writeWiresToDb(h_wires);
  }
  if (!v_wires.empty()) {
    writeWiresToDb(v_wires);
  }
  if (!vias.empty()) {
    writeViasToDb(vias);
  }

  // Compute grid intersections for buffer placement
  if (!h_wires.empty() && !v_wires.empty()) {
    grid_intersections_.clear();
    logger_->report("Computing grid intersections...");

    for (const MeshWire& h_wire : h_wires) {
      int h_y = (h_wire.rect.yMin() + h_wire.rect.yMax()) / 2;
      for (const MeshWire& v_wire : v_wires) {
        int v_x = (v_wire.rect.xMin() + v_wire.rect.xMax()) / 2;
        odb::dbTechLayer* buf_layer = selectBufferLayer(h_wire.layer, v_wire.layer);
        grid_intersections_.emplace_back(v_x, h_y, buf_layer);
      }
    }
    logger_->report("Computed {} grid intersections", grid_intersections_.size());
  }

  logger_->report("======================================");
  logger_->report("Clock mesh grid creation completed!");
  logger_->report("Total wires: {} (H: {}, V: {})", h_wires.size() + v_wires.size(), h_wires.size(), v_wires.size());
  logger_->report("Total vias: {}", vias.size());

  // Store mesh wires for later sink connection
  mesh_wires_.clear();
  mesh_wires_.insert(mesh_wires_.end(), h_wires.begin(), h_wires.end());
  mesh_wires_.insert(mesh_wires_.end(), v_wires.begin(), v_wires.end());

  // Place buffers and connect to nets
  if (!buffer_list.empty() && !grid_intersections_.empty()) {
    logger_->report("======================================");
    logger_->report("Placing buffers and connecting to mesh/tree nets...");

    placeBuffersAtIntersections(buffer_list[0], mesh_net);
    connectBuffersToNets(mesh_net, clock_name);

    // Call TritonCTS to build tree to mesh buffer inputs (CTS chooses its own buffers)
    buildCtsTreeToBuffers(clock_name);
  }

  logger_->report("======================================");
  logger_->report("Mesh and CTS tree created. Run detailed_placement to legalize buffers,");
  logger_->report("then call connect_sinks_to_mesh to attach sinks with stubs.");

  mesh_generated_ = true;
}



void ClockMesh::createBufferStubWire(const GridIntersection& inter,odb::dbNet* mesh_net, odb::dbSWire* swire, odb::dbTechLayer* li1_layer)
{
  if (!inter.buffer_inst || !inter.has_buffer) {
    return;
  }
  odb::dbITerm* output_pin = getBufferOutputPin(inter.buffer_inst);
  if (!output_pin) {
    return;
  }

  int out_x, out_y;
  computeITermPosition(output_pin, out_x, out_y);
  int target_x = inter.x;
  int target_y = inter.y;
  odb::dbTech* tech = db_->getTech();
  int h_level = mesh_h_layer_ ? mesh_h_layer_->getRoutingLevel() : 0;
  int v_level = mesh_v_layer_ ? mesh_v_layer_->getRoutingLevel() : 0;
  int stub_level = std::max(h_level, v_level) + 1;
  odb::dbTechLayer* stub_layer = tech->findRoutingLayer(stub_level);

  if (!stub_layer) {
    stub_layer = (h_level > v_level) ? mesh_h_layer_ : mesh_v_layer_;
    stub_level = std::max(h_level, v_level);
  }

  int half_width = mesh_wire_width_ / 2;
  odb::Point out_point(out_x, out_y);
  odb::Point target_point(target_x, target_y);
  createViaStackAtPoint(out_point, li1_layer, stub_layer, mesh_net);
  if (out_x != target_x) {
    odb::dbSBox::create(swire, stub_layer, std::min(out_x, target_x), out_y - half_width, std::max(out_x, target_x), out_y + half_width, odb::dbWireShapeType::STRIPE);
  }
  if (out_y != target_y) {
    odb::dbSBox::create(swire, stub_layer, target_x - half_width, std::min(out_y, target_y), target_x + half_width, std::max(out_y, target_y), odb::dbWireShapeType::STRIPE);
  }

  // Connect down to both horizontal and vertical grid layers
  if (mesh_h_layer_) {
    createViaStackAtPoint(target_point, mesh_h_layer_, stub_layer, mesh_net);
  }
  if (mesh_v_layer_ && mesh_v_layer_ != mesh_h_layer_) {
    createViaStackAtPoint(target_point, mesh_v_layer_, stub_layer, mesh_net);
  }
}

// Connect all buffers to their associated intersections 
void ClockMesh::connectBuffersToIntersections(const std::string& clock_name)
{
  logger_->report("======================================");
  logger_->report("Connecting buffers to their grid intersections...");

  if (!mesh_generated_) {
    logger_->error(MESH, 504, "Mesh not generated. Call create_clock_mesh first.");
    return;
  }

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(MESH, 505, "Could not find mesh net for clock '{}'", clock_name);
    return;
  }

  odb::dbSWire* swire = nullptr;
  auto swires = mesh_net->getSWires();
  if (!swires.empty()) {
    swire = *swires.begin();
  } else {
    swire = odb::dbSWire::create(mesh_net, odb::dbWireType::ROUTED);
  }
  odb::dbTech* tech = db_->getTech();
  odb::dbTechLayer* li1_layer = tech->findLayer("li1");
  if (!li1_layer) {
    li1_layer = tech->findRoutingLayer(1);
  }

  int connected = 0;
  for (const GridIntersection& inter : grid_intersections_) {
    if (inter.buffer_inst && inter.has_buffer) {
      createBufferStubWire(inter, mesh_net, swire, li1_layer);
      connected++;
    }
  }
  if (!connection_vias_.empty()) {
    logger_->report("Writing {} buffer connection vias...", connection_vias_.size());
    writeViasToDb(connection_vias_);
    connection_vias_.clear();
  }

  logger_->report("Connected {} buffers to their intersections", connected);
}


void ClockMesh::connectSinks(const std::string& clock_name)
{
  logger_->report("======================================");
  logger_->report("Connecting sinks and buffers to mesh (post-placement)...");

  if (!mesh_generated_) {
    logger_->error(MESH, 500, "Mesh not generated. Call create_clock_mesh first.");
    return;
  }

  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(MESH, 501, "Could not find mesh net for clock '{}'", clock_name);
    return;
  }
  odb::dbSWire* swire = nullptr;
  auto swires = mesh_net->getSWires();
  if (!swires.empty()) {
    swire = *swires.begin();
  } else {
    swire = odb::dbSWire::create(mesh_net, odb::dbWireType::ROUTED);
  }

  int half_width = mesh_wire_width_ / 2;
  odb::dbTech* tech = db_->getTech();
  odb::dbTechLayer* li1_layer = tech->findLayer("li1");
  if (!li1_layer) {
    li1_layer = tech->findRoutingLayer(1);
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
  logger_->report("Connecting clock sinks to mesh...");

  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    logger_->warn(MESH, 503, "No sinks found for clock '{}'", clock_name);
    return;
  }

  const std::vector<ClockSink>& sinks = clockToSinks_[clock_name];
  int sink_connected = 0;

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
      continue;
    }
    sink.iterm->disconnect();
    sink.iterm->connect(mesh_net);
    createViaStackAtPoint(sink_point, li1_layer, grid_layer, mesh_net);
    if (grid_point.y() != sink_y) {
      odb::dbSBox::create(swire, grid_layer, sink_x - half_width, std::min(sink_y, grid_point.y()), sink_x + half_width, std::max(sink_y, grid_point.y()), odb::dbWireShapeType::STRIPE);
    } else if (grid_point.x() != sink_x) {
      //create horizontal stub
      odb::dbSBox::create(swire, grid_layer, std::min(sink_x, grid_point.x()), sink_y - half_width, std::max(sink_x, grid_point.x()), sink_y + half_width, odb::dbWireShapeType::STRIPE);
    }

    sink_connected++;
  }
  logger_->report("Connected {} sinks to mesh", sink_connected);
  if (!connection_vias_.empty()) {
    logger_->report("Writing {} connection vias...", connection_vias_.size());
    writeViasToDb(connection_vias_);
    connection_vias_.clear();
  }
  logger_->report("======================================");
  logger_->report("Post-placement connections completed.");
}

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

odb::dbTechLayer* ClockMesh::getSinkPinLayer(odb::dbITerm* iterm)
{
  if (!iterm) {
    return nullptr;
  }
  odb::dbITermShapeItr itr;
  odb::dbShape shape;
  for (itr.begin(iterm); itr.next(shape);) {
    if (!shape.isVia() && shape.getTechLayer()) {
      return shape.getTechLayer();
    }
  }
  odb::dbMTerm* mterm = iterm->getMTerm();
  if (mterm) {
    for (odb::dbMPin* mpin : mterm->getMPins()) {
      for (odb::dbBox* box : mpin->getGeometry()) {
        if (box->getTechLayer()) {
          return box->getTechLayer();
        }
      }
    }
  }

  return nullptr;
}

void ClockMesh::createSinkStubWire(odb::dbITerm* sink_iterm, const odb::Point& sink_loc, const odb::Point& grid_point, odb::dbTechLayer* sink_layer, odb::dbTechLayer* grid_layer, odb::dbNet* mesh_net)
{
  if (!sink_iterm || !mesh_net || !sink_layer || !grid_layer) {
    return;
  }

  odb::dbSWire* swire = nullptr;
  auto swires = mesh_net->getSWires();
  if (!swires.empty()) {
    swire = *swires.begin();
  } else {
    swire = odb::dbSWire::create(mesh_net, odb::dbWireType::ROUTED);
  }
  if (!swire) {
    return;
  }

  int half_width = mesh_wire_width_ / 2;
  int x1 = sink_loc.x();
  int y1 = sink_loc.y();
  int x2 = grid_point.x();
  int y2 = grid_point.y();

  if (x1 != x2) {
    odb::dbSBox::create(swire, sink_layer, std::min(x1, x2), y1 - half_width, std::max(x1, x2), y1 + half_width, odb::dbWireShapeType::STRIPE);
  } else if (y1 != y2) {
    odb::dbSBox::create(swire, sink_layer, x1 - half_width, std::min(y1, y2), x1 + half_width, std::max(y1, y2), odb::dbWireShapeType::STRIPE);
  }

  if (sink_layer->getRoutingLevel() != grid_layer->getRoutingLevel()) {
    createViaStackAtPoint(grid_point, sink_layer, grid_layer, mesh_net);
  }
}

void ClockMesh::connectSinksToMesh(const std::string& clock_name, odb::dbNet* mesh_net, const std::vector<MeshWire>& h_wires, const std::vector<MeshWire>& v_wires)
{
  if (!mesh_net || h_wires.empty() || v_wires.empty()) {
    return;
  }

  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    return;
  }
  const std::vector<ClockSink>& sinks = clockToSinks_[clock_name];
  logger_->report("======================================");
  logger_->report("Connecting {} sinks to mesh...", sinks.size());
  int connected_count = 0;
  for (const auto& sink : sinks) {
    if (!sink.iterm) {
      continue;
    }
    odb::Point sink_loc(sink.x, sink.y);
    odb::dbTechLayer* sink_pin_layer = getSinkPinLayer(sink.iterm);
    if (!sink_pin_layer) {
      continue;
    }
    odb::dbTechLayer* grid_layer = nullptr;
    odb::Point grid_point = findNearestGridWire(sink_loc, h_wires, v_wires, &grid_layer);
    if (!grid_layer) {
      continue;
    }
    sink.iterm->disconnect();
    sink.iterm->connect(mesh_net);
    createViaStackAtPoint(sink_loc, sink_pin_layer, grid_layer, mesh_net);
    createSinkStubWire(sink.iterm, sink_loc, grid_point, grid_layer, grid_layer, mesh_net);
    connected_count++;
  }
  logger_->report("Connected {} sinks to mesh", connected_count);
}

odb::dbTechLayer* ClockMesh::selectBufferLayer(odb::dbTechLayer* h_layer, odb::dbTechLayer* v_layer)
{
  if (!h_layer) return v_layer;
  if (!v_layer) return h_layer;

  int h_level = h_layer->getRoutingLevel();
  int v_level = v_layer->getRoutingLevel();

  return (h_level < v_level) ? h_layer : v_layer;
}

// GridIntersection* ClockMesh::findNearestGridIntersection(int x, int y)
// {
//   if (grid_intersections_.empty()) {
//     return nullptr;
//   }

//   GridIntersection* nearest = &grid_intersections_[0];
//   int min_distance = std::abs(nearest->x - x) + std::abs(nearest->y - y);

//   for (GridIntersection& inter : grid_intersections_) {
//     int distance = std::abs(inter.x - x) + std::abs(inter.y - y);
//     if (distance < min_distance) {
//       min_distance = distance;
//       nearest = &inter;
//     }
//   }
//   return nearest;
// }

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

void ClockMesh::placeBuffersAtIntersections(const std::string& buffer_master, odb::dbNet* mesh_net)
{
  logger_->report("======================================");
  logger_->report("Placing buffers at grid intersections...");

  if (grid_intersections_.empty()) {
    logger_->warn(MESH, 307, "No grid intersections available");
    return;
  }
  odb::dbMaster* master = db_->findMaster(buffer_master.c_str());
  if (!master) {
    logger_->error(MESH, 308, "Buffer master '{}' not found", buffer_master);
    return;
  }
  int placed_count = 0;
  for (GridIntersection& inter : grid_intersections_) {
    std::string buf_name = "mesh_buf_" + std::to_string(inter.x) +"_" + std::to_string(inter.y);
    odb::dbInst* buf_inst = odb::dbInst::create(block_, master, buf_name.c_str());
    if (buf_inst) {
      buf_inst->setLocation(inter.x, inter.y);
      buf_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
      inter.buffer_inst = buf_inst;
      inter.has_buffer = true;
      placed_count++;
    }
  }
  logger_->report("======================================");
  logger_->report("Placed {} buffers at grid intersections", placed_count);
}

void ClockMesh::connectBuffersToNets(odb::dbNet* mesh_net, const std::string& clock_name)
{
  logger_->report("======================================");
  logger_->report("Connecting buffers to mesh and tree nets...");
  std::string tree_net_name = getTreeNetName(clock_name);
  odb::dbNet* tree_net = block_->findNet(tree_net_name.c_str());
  if (!tree_net) {
    tree_net = odb::dbNet::create(block_, tree_net_name.c_str());
    if (tree_net) {
      tree_net->setSigType(odb::dbSigType::CLOCK);
      logger_->report("Created tree net '{}'", tree_net_name);
    }
  }
  odb::dbBTerm* clk_root = mesh_net->get1stBTerm();
  if (clk_root && tree_net) {
    clk_root->disconnect();
    clk_root->connect(tree_net);
    logger_->report("Connected clock root '{}' to tree net", clk_root->getConstName());
  }
  int connected_count = 0;
  for (const GridIntersection& intersection : grid_intersections_) {
    if (!intersection.buffer_inst || !intersection.has_buffer) {
      continue;
    }
    // Connect output to mesh net
    odb::dbITerm* output_pin = getBufferOutputPin(intersection.buffer_inst);
    if (output_pin) {
      output_pin->connect(mesh_net);
    }
    // Connect input to tree net (for CTS to drive)
    odb::dbITerm* input_pin = getBufferInputPin(intersection.buffer_inst);
    if (input_pin && tree_net) {
      input_pin->connect(tree_net);
    }
    connected_count++;
  }
  logger_->report("Connected {} buffers (outputs->mesh, inputs->tree)", connected_count);
}


void ClockMesh::buildCtsTreeToBuffers(const std::string& clock_name)
{
  cts::TritonCTS* triton_cts = openroad_->getTritonCts();
  if (!triton_cts) {
    logger_->error(MESH, 400, "TritonCTS not available");
    return;
  }
  std::string tree_net_name = getTreeNetName(clock_name);
  int result = triton_cts->setClockNets(tree_net_name.c_str());
  if (result != 0) {
    logger_->warn(MESH, 401, "Failed to set clock net '{}' for CTS", tree_net_name);
    return;
  }
  logger_->report("Set CTS clock net: {}", tree_net_name);

  // Find all clock buffers in the library and provide to CTS
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
    logger_->report("Set CTS buffer list: {}", clkbuf_list);
  } else {
    logger_->warn(MESH, 402, "No clock buffers found in library");
  }
  logger_->report("Running TritonCTS...");
  triton_cts->runTritonCts();
  logger_->report("CTS tree construction completed");
  logger_->report("======================================");
}

}  // namespace mesh
