// SPDX-License-Identifier: BSD-3-Clause

#include "mesh/ClockMesh.hh"

#include <cmath>
#include <limits>
#include <map>

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
  testPrintSinks();

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

void ClockMesh::findClockRoots(sta::Clock* clk,
                               std::set<odb::dbNet*>& clockNets)
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


bool ClockMesh::separateSinks(odb::dbNet* net,
                               std::vector<ClockSink>& sinks)
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
        // Log sink data for verification (parseable format)
        logger_->report("MESH_SINK_DATA: {} {} {} {}", name, x, y, net->getConstName());
      }
    }
  }

  return !sinks.empty();
}

// ========== TEST FUNCTION ONLY (comment out later) ==========

void ClockMesh::testPrintSinks()
{
  if (!logger_) {
    return;
  }

  logger_->report("TEST: Found {} clock(s)", clockToSinks_.size());

  for (const auto& [clockName, sinks] : clockToSinks_) {
    logger_->report("\nTEST: Clock '{}' has {} sinks:", clockName, sinks.size());

    // Print all sinks with details
    int count = 0;
    for (const auto& sink : sinks) {
      logger_->report("  [{}] {} at ({}, {}) cap={:.3e} macro={}",
                     ++count, sink.name, sink.x, sink.y, sink.inputCap, sink.isMacro);
    }
  }
}

// ========== MESH GRID CREATION FUNCTIONS (following PDN pattern) ==========

// Step 1: Calculate bounding box from clock sinks (similar to PDN domain calculation)
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

  // Add margin around the bounding box (10% on each side)
  // int margin_x = (max_x - min_x) / 10;
  // int margin_y = (max_y - min_y) / 10;

  odb::Rect bbox(min_x, min_y, max_x, max_y);

  // Clamp to core area (inner bounding box) instead of die area
  if (block_) {
    odb::Rect die_area = block_->getDieArea();
    odb::Rect core_area = block_->getCoreArea();

    logger_->report("Die area: ({}, {}) to ({}, {})",
                   die_area.xMin(), die_area.yMin(),
                   die_area.xMax(), die_area.yMax());
    logger_->report("Core area: ({}, {}) to ({}, {})",
                   core_area.xMin(), core_area.yMin(),
                   core_area.xMax(), core_area.yMax());

    // Use core area as the boundary (this is the actual placement area)
    bbox.set_xlo(std::min(bbox.xMin(), core_area.xMin()));
    bbox.set_ylo(std::min(bbox.yMin(), core_area.yMin()));
    bbox.set_xhi(std::max(bbox.xMax(), core_area.xMax()));
    bbox.set_yhi(std::max(bbox.yMax(), core_area.yMax()));


    // bbox.set_xlo(bbox.xMin());
    // bbox.set_ylo(bbox.yMin());
    // bbox.set_xhi(bbox.xMax());
    // bbox.set_yhi(bbox.yMax());
  }

  logger_->report("Bounding box calculated: ({}, {}) to ({}, {})",
                 bbox.xMin(), bbox.yMin(), bbox.xMax(), bbox.yMax());

  return bbox;
}

// Step 2: Create horizontal wires (following PDN Straps::makeStraps pattern)
void ClockMesh::createHorizontalWires(odb::dbNet* net,
                                      odb::dbTechLayer* layer,
                                      const odb::Rect& bbox,
                                      int wire_width,
                                      int pitch,
                                      std::vector<MeshWire>& wires)
{
  if (!net || !layer) {
    logger_->error(MESH, 101, "Invalid net or layer for horizontal wires");
    return;
  }

  const int half_width = wire_width / 2;
  const int x_start = bbox.xMin();
  const int x_end = bbox.xMax();

  int wire_count = 0;

  logger_->report("Creating horizontal wires on layer {} from Y={} to Y={}, pitch={}",
                 layer->getName(), bbox.yMin(), bbox.yMax(), pitch);

  // Loop through Y positions at pitch intervals (similar to PDN makeStraps)
  for (int y_pos = bbox.yMin(); y_pos <= bbox.yMax(); y_pos += pitch) {
    const int y_min = y_pos - half_width;
    const int y_max = y_pos + half_width;

    // Create horizontal wire rectangle
    odb::Rect wire_rect(x_start, y_min, x_end, y_max);

    // Add wire to collection
    wires.emplace_back(layer, net, wire_rect, true);  // true = horizontal
    wire_count++;
  }

  logger_->report("Created {} horizontal wires on layer {}", wire_count, layer->getName());
}

// Step 3: Create vertical wires (following PDN Straps::makeStraps pattern)
void ClockMesh::createVerticalWires(odb::dbNet* net,
                                    odb::dbTechLayer* layer,
                                    const odb::Rect& bbox,
                                    int wire_width,
                                    int pitch,
                                    std::vector<MeshWire>& wires)
{
  if (!net || !layer) {
    logger_->error(MESH, 102, "Invalid net or layer for vertical wires");
    return;
  }

  const int half_width = wire_width / 2;
  const int y_start = bbox.yMin();
  const int y_end = bbox.yMax();

  int wire_count = 0;

  logger_->report("Creating vertical wires on layer {} from X={} to X={}, pitch={}",
                 layer->getName(), bbox.xMin(), bbox.xMax(), pitch);

  // Loop through X positions at pitch intervals (similar to PDN makeStraps)
  for (int x_pos = bbox.xMin(); x_pos <= bbox.xMax(); x_pos += pitch) {
    const int x_min = x_pos - half_width;
    const int x_max = x_pos + half_width;

    // Create vertical wire rectangle
    odb::Rect wire_rect(x_min, y_start, x_max, y_end);

    // Add wire to collection
    wires.emplace_back(layer, net, wire_rect, false);  // false = vertical
    wire_count++;
  }

  logger_->report("Created {} vertical wires on layer {}", wire_count, layer->getName());
}

// Step 4: Create vias at intersections (following PDN Grid::getIntersections pattern)
void ClockMesh::createViasAtIntersections(const std::vector<MeshWire>& h_wires,
                                          const std::vector<MeshWire>& v_wires,
                                          std::vector<MeshVia>& vias)
{
  if (h_wires.empty() || v_wires.empty()) {
    logger_->warn(MESH, 103, "No wires provided for via creation");
    return;
  }

  int via_count = 0;

  logger_->report("Creating vias at intersections: {} h_wires x {} v_wires",
                 h_wires.size(), v_wires.size());

  // Find all intersections between horizontal and vertical wires
  // (similar to PDN Grid::getIntersections using R-tree spatial queries)
  for (const auto& h_wire : h_wires) {
    for (const auto& v_wire : v_wires) {
      // Check if rectangles intersect
      odb::Rect intersection = h_wire.rect;
      if (intersection.intersects(v_wire.rect)) {
        // Calculate intersection area
        intersection.set_xlo(std::max(h_wire.rect.xMin(), v_wire.rect.xMin()));
        intersection.set_ylo(std::max(h_wire.rect.yMin(), v_wire.rect.yMin()));
        intersection.set_xhi(std::min(h_wire.rect.xMax(), v_wire.rect.xMax()));
        intersection.set_yhi(std::min(h_wire.rect.yMax(), v_wire.rect.yMax()));

        // Determine upper and lower layers
        odb::dbTechLayer* lower_layer = nullptr;
        odb::dbTechLayer* upper_layer = nullptr;

        if (h_wire.layer->getRoutingLevel() < v_wire.layer->getRoutingLevel()) {
          lower_layer = h_wire.layer;
          upper_layer = v_wire.layer;
        } else {
          lower_layer = v_wire.layer;
          upper_layer = h_wire.layer;
        }

        // Create via at intersection
        vias.emplace_back(lower_layer, upper_layer, h_wire.net, intersection);
        via_count++;
      }
    }
  }

  logger_->report("Created {} vias at wire intersections", via_count);
}

// Step 5: Get or create clock net
odb::dbNet* ClockMesh::getOrCreateClockNet(const std::string& clock_name)
{
  if (!block_) {
    logger_->error(MESH, 104, "No block available for net creation");
    return nullptr;
  }

  // First, try to find the original clock net by searching all nets
  // Look for nets connected to the clock sinks we found
  if (clockToSinks_.find(clock_name) != clockToSinks_.end()) {
    const auto& sinks = clockToSinks_[clock_name];
    if (!sinks.empty() && sinks[0].iterm) {
      odb::dbNet* clock_net = sinks[0].iterm->getNet();
      if (clock_net) {
        logger_->report("Using existing clock net: {}", clock_net->getName());
        // Make sure it's marked as special and clock type
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

  // If we couldn't find the original clock net, search by common clock net names
  std::vector<std::string> possible_names = {
    clock_name,
    "clk",
    "core_clock",
    clock_name + "_mesh"
  };

  for (const auto& name : possible_names) {
    odb::dbNet* net = block_->findNet(name.c_str());
    if (net) {
      logger_->report("Found clock net by name: {}", net->getName());
      if (!net->isSpecial()) {
        net->setSpecial();
      }
      if (net->getSigType() != odb::dbSigType::CLOCK) {
        net->setSigType(odb::dbSigType::CLOCK);
      }
      return net;
    }
  }

  // Last resort: create new net for clock mesh
  std::string mesh_net_name = clock_name + "_mesh";
  odb::dbNet* net = odb::dbNet::create(block_, mesh_net_name.c_str());
  if (!net) {
    logger_->error(MESH, 105, "Failed to create clock mesh net: {}", mesh_net_name);
    return nullptr;
  }

  // Set net type to CLOCK
  net->setSpecial();
  net->setSigType(odb::dbSigType::CLOCK);

  logger_->report("Created new clock mesh net: {}", mesh_net_name);
  logger_->warn(MESH, 106, "Could not find original clock net - created separate mesh net. Mesh will not be connected to clock source.");
  return net;
}

// Step 6: Write wires to database
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

    // Get or create special wire (SWire) for the net
    odb::dbSWire* swire = nullptr;
    auto swires = wire.net->getSWires();
    if (!swires.empty()) {
      // Use existing SWire
      swire = *swires.begin();
    } else {
      // Create new SWire
      swire = odb::dbSWire::create(wire.net, odb::dbWireType::ROUTED);
    }

    if (!swire) {
      logger_->warn(MESH, 107, "Failed to get/create SWire for net {}", wire.net->getName());
      continue;
    }

    // Create SBox (special box) for the wire shape
    odb::dbSBox* sbox = odb::dbSBox::create(
        swire, wire.layer, wire.rect.xMin(), wire.rect.yMin(),
        wire.rect.xMax(), wire.rect.yMax(), odb::dbWireShapeType::STRIPE);

    if (sbox) {
      written_count++;
    } else {
      logger_->warn(MESH, 108, "Failed to create SBox for wire on layer {}",
                   wire.layer->getName());
    }
  }

  logger_->report("Wrote {} wires to database", written_count);
}

// Step 7: Write vias to database
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

    // Get or create SWire for the net
    odb::dbSWire* swire = nullptr;
    auto swires = via.net->getSWires();
    if (!swires.empty()) {
      // Use existing SWire
      swire = *swires.begin();
    } else {
      // Create new SWire
      swire = odb::dbSWire::create(via.net, odb::dbWireType::ROUTED);
    }

    if (!swire) {
      continue;
    }

    // Find appropriate via rule between the layers
    odb::dbTechVia* tech_via = nullptr;

    // Search for a default via between these layers
    for (odb::dbTechVia* tv : tech->getVias()) {
      if (tv->getBottomLayer() == via.lower_layer &&
          tv->getTopLayer() == via.upper_layer) {
        tech_via = tv;
        break;
      }
    }

    if (!tech_via) {
      logger_->warn(MESH, 111, "No tech via found between layers {} and {}",
                   via.lower_layer->getName(), via.upper_layer->getName());
      continue;
    }

    // Calculate via center
    int via_x = (via.area.xMin() + via.area.xMax()) / 2;
    int via_y = (via.area.yMin() + via.area.yMax()) / 2;

    // Create via using dbSBox (using NONE type for vias)
    odb::dbSBox* via_box = odb::dbSBox::create(
        swire, tech_via, via_x, via_y, odb::dbWireShapeType::NONE);

    if (via_box) {
      written_count++;
    } else {
      logger_->warn(MESH, 112, "Failed to create via at ({}, {})", via_x, via_y);
    }
  }

  logger_->report("Wrote {} vias to database", written_count);
}

// Main orchestration function (following PDN buildGrids pattern)
void ClockMesh::createMeshGrid(const std::string& clock_name,
                               odb::dbTechLayer* h_layer,
                               odb::dbTechLayer* v_layer,
                               int wire_width,
                               int pitch,
                               const std::vector<std::string>& buffer_list)
{
  if (!logger_ || !block_) {
    logger_->error(MESH, 113, "ClockMesh not properly initialized");
    return;
  }

  logger_->report("======================================");
  logger_->report("Creating clock mesh grid for clock: {}", clock_name);
  logger_->report("Horizontal layer: {}, Vertical layer: {}",
                 h_layer ? h_layer->getName() : "NULL",
                 v_layer ? v_layer->getName() : "NULL");
  logger_->report("Wire width: {}, Pitch: {}", wire_width, pitch);
  logger_->report("======================================");

  // Step 1: Get clock sinks
  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    logger_->error(MESH, 114, "No sinks found for clock: {}", clock_name);
    logger_->report("Available clocks:");
    for (const auto& [clk, sinks] : clockToSinks_) {
      logger_->report("  - {} ({} sinks)", clk, sinks.size());
    }
    return;
  }

  const std::vector<ClockSink>& sinks = clockToSinks_[clock_name];
  logger_->report("Found {} sinks for clock {}", sinks.size(), clock_name);

  // Step 2: Calculate bounding box from sinks
  odb::Rect bbox = calculateBoundingBox(sinks);
  if (bbox.area() == 0) {
    logger_->error(MESH, 115, "Invalid bounding box calculated");
    return;
  }

  // Store bbox for position-aware load calculation
  mesh_bbox_ = bbox;

  // Step 3: Get or create clock mesh net
  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(MESH, 116, "Failed to create clock mesh net");
    return;
  }

  // Step 4: Create horizontal wires (following PDN Straps pattern)
  std::vector<MeshWire> h_wires;
  if (h_layer) {
    createHorizontalWires(mesh_net, h_layer, bbox, wire_width, pitch, h_wires);
  }

  // Step 5: Create vertical wires (following PDN Straps pattern)
  std::vector<MeshWire> v_wires;
  if (v_layer) {
    createVerticalWires(mesh_net, v_layer, bbox, wire_width, pitch, v_wires);
  }

  // Step 6: Create vias at intersections (following PDN Grid::makeVias pattern)
  std::vector<MeshVia> vias;
  if (!h_wires.empty() && !v_wires.empty()) {
    createViasAtIntersections(h_wires, v_wires, vias);
  }

  // Step 7: Write wires to database
  if (!h_wires.empty()) {
    writeWiresToDb(h_wires);
  }
  if (!v_wires.empty()) {
    writeWiresToDb(v_wires);
  }

  // Step 8: Write vias to database
  if (!vias.empty()) {
    writeViasToDb(vias);
  }

  // Store for later reference
  mesh_wires_.insert(mesh_wires_.end(), h_wires.begin(), h_wires.end());
  mesh_wires_.insert(mesh_wires_.end(), v_wires.begin(), v_wires.end());
  mesh_vias_.insert(mesh_vias_.end(), vias.begin(), vias.end());

  logger_->report("======================================");
  logger_->report("Clock mesh grid creation completed!");
  logger_->report("Total wires: {} (H: {}, V: {})",
                 h_wires.size() + v_wires.size(), h_wires.size(), v_wires.size());
  logger_->report("Total vias: {}", vias.size());
  logger_->report("======================================");

  // Buffer insertion removed - keeping only grid creation
  if (!buffer_list.empty()) {
    logger_->warn(utl::MESH, 300, "Buffer insertion not yet implemented");
  }

  mesh_generated_ = true;
}

}  // namespace mesh
