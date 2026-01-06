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
 // testPrintSinks();

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

  // Log summary of found clocks
  logger_->report("Total clocks with sinks: {}", clockToSinks_.size());
  for (const auto& [clk_name, sinks] : clockToSinks_) {
    logger_->report("  Clock '{}': {} sinks", clk_name, sinks.size());
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

// ========== TEST FUNCTION ==========

// void ClockMesh::testPrintSinks()
// {
//   if (!logger_) {
//     return;
//   }

//   logger_->report("TEST: Found {} clock(s)", clockToSinks_.size());

//   for (const auto& [clockName, sinks] : clockToSinks_) {
//     logger_->report("\nTEST: Clock '{}' has {} sinks:", clockName, sinks.size());

//     // Print all sinks with details
//     int count = 0;
//     for (const auto& sink : sinks) {
//       logger_->report("  [{}] {} at ({}, {}) cap={:.3e} macro={}",
//                      ++count, sink.name, sink.x, sink.y, sink.inputCap, sink.isMacro);
//     }
//   }
// }

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

  for (const auto& h_wire : h_wires) {
    for (const auto& v_wire : v_wires) {
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

        // Skip via creation if both wires are on the same layer
        if (h_wire.layer == v_wire.layer) {
          continue;
        }

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
  logger_->warn(MESH, 117, "Could not get clock net from sinks for '{}'. Checking other available clocks...", clock_name);
  for (const auto& [clk_name, sinks] : clockToSinks_) {
    if (!sinks.empty() && sinks[0].iterm) {
      odb::dbNet* clock_net = sinks[0].iterm->getNet();
      if (clock_net) {
        logger_->report("Found clock net '{}' from clock '{}' sinks", lock_net->getName(), clk_name);
        logger_->warn(MESH, 118,"Using clock '{}' instead of '{}'. " "You may want to use -clock {} in your command.", clk_name, clock_name, clk_name);
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
  logger_->error(MESH, 105,"Could not find clock net for '{}'. " "None of the clocks in the design have valid nets connected to their sinks.", clock_name);
  if (clockToSinks_.empty()) {
    logger_->report("No clocks with sinks found. Did you run 'run_mesh' first?");
  } else {
    logger_->report("Clocks with sinks found (but no valid nets):");
    for (const auto& [clk, sinks] : clockToSinks_) {
      logger_->report("  - {} ({} sinks)", clk, sinks.size());
    }
    logger_->report("Check that synthesis created clock nets and connected sinks properly.");
  }
  return nullptr;
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

    int lower_level = via.lower_layer->getRoutingLevel();
    int upper_level = via.upper_layer->getRoutingLevel();
    int via_x = (via.area.xMin() + via.area.xMax()) / 2;
    int via_y = (via.area.yMin() + via.area.yMax()) / 2;

    // Check if layers are non-consecutive
    if (upper_level - lower_level > 1) {
      // Create via stack through intermediate layers
     // logger_->report("Creating via stack from {} (level {}) to {} (level {}) at ({}, {})",
       //              via.lower_layer->getName(), lower_level,
        //             via.upper_layer->getName(), upper_level,
        //             via_x, via_y);

      for (int level = lower_level; level < upper_level; level++) {
        odb::dbTechLayer* lower = tech->findRoutingLayer(level);
        odb::dbTechLayer* upper = tech->findRoutingLayer(level + 1);

        if (!lower || !upper) {
          logger_->warn(MESH, 111, "Could not find routing layers for level {} and {}",
                       level, level + 1);
          continue;
        }

        // Find tech via between consecutive layers
        odb::dbTechVia* tech_via = nullptr;
        for (odb::dbTechVia* tv : tech->getVias()) {
          if (tv->getBottomLayer() == lower && tv->getTopLayer() == upper) {
            tech_via = tv;
            break;
          }
        }

        if (!tech_via) {
          logger_->warn(MESH, 111, "No tech via found between layers {} and {}",
                       lower->getName(), upper->getName());
          continue;
        }

        // Get or create SWire for the net
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

        // Create via between consecutive layers
        odb::dbSBox* via_box = odb::dbSBox::create(
            swire, tech_via, via_x, via_y, odb::dbWireShapeType::NONE);

        if (via_box) {
          written_count++;
          logger_->report("  Created via between {} and {} at ({}, {})",
                         lower->getName(), upper->getName(), via_x, via_y);
        } else {
          logger_->warn(MESH, 112, "Failed to create via at ({}, {})", via_x, via_y);
        }
      }
    } else {
      // Consecutive layers - create single via
      odb::dbTechVia* tech_via = nullptr;

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

      // Get or create SWire for the net
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

      // Create via using dbSBox
      odb::dbSBox* via_box = odb::dbSBox::create(
          swire, tech_via, via_x, via_y, odb::dbWireShapeType::NONE);

      if (via_box) {
        written_count++;
      } else {
        logger_->warn(MESH, 112, "Failed to create via at ({}, {})", via_x, via_y);
      }
    }
  }

  logger_->report("Wrote {} vias to database", written_count);
}

// Main function 
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

  // Get clock sinks
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

  //Bounding box
  odb::Rect bbox = calculateBoundingBox(sinks);
  if (bbox.area() == 0) {
    logger_->error(MESH, 115, "Invalid bounding box calculated");
    return;
  }
  mesh_bbox_ = bbox;

  // Store wire width for connection routing
  mesh_wire_width_ = wire_width;
  logger_->report("Stored wire width: {} DBU (half-width: {} DBU)",
                 mesh_wire_width_, mesh_wire_width_ / 2);

  // Step 6: Get existing clock net from synthesis
  odb::dbNet* mesh_net = getOrCreateClockNet(clock_name);
  if (!mesh_net) {
    logger_->error(MESH, 116, "Failed to find clock net for '{}'. Cannot create mesh without existing clock net.", clock_name);
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

  // Step 9: Connect clock root to mesh grid
  if (!h_wires.empty() && !v_wires.empty()) {
    connectClockRootToMesh(clock_name, mesh_net, h_wires, v_wires);
  }

  // Step 10: Connect all clock sinks to mesh grid
  if (!h_wires.empty() && !v_wires.empty()) {
    connectSinksToMesh(clock_name, mesh_net, h_wires, v_wires);
  }

  // Step 11: Write connection vias to database (stored during root/sink connection)
  if (!connection_vias_.empty()) {
    logger_->report("======================================");
    logger_->report("Writing {} connection vias to database...", connection_vias_.size());
    writeViasToDb(connection_vias_);
    logger_->report("======================================");
  }

  // Buffer insertion removed - keeping only grid creation
  if (!buffer_list.empty()) {
    logger_->warn(utl::MESH, 300, "Buffer insertion not yet implemented");
  }

  mesh_generated_ = true;
}


// Get clock root BTerm location
odb::Point ClockMesh::getClockRootLocation(odb::dbBTerm* bterm)
{
  if (!bterm) {
    logger_->error(MESH, 200, "NULL BTerm provided");
    return odb::Point(0, 0);
  }

  int x, y;
  bterm->getFirstPinLocation(x, y);
  logger_->report("Clock root '{}' location: ({}, {})",
                 bterm->getConstName(), x, y);
  return odb::Point(x, y);
}

// Get root pin layer
odb::dbTechLayer* ClockMesh::getRootPinLayer(odb::dbBTerm* bterm)
{
  if (!bterm) {
    return nullptr;
  }

  for (odb::dbBPin* bpin : bterm->getBPins()) {
    for (odb::dbBox* box : bpin->getBoxes()) {
      if (box->getTechLayer()) {
        odb::dbTechLayer* layer = box->getTechLayer();
        logger_->report("Clock root pin on layer: {} (routing level {})",
                       layer->getName(), layer->getRoutingLevel());
        return layer;
      }
    }
  }

  logger_->error(MESH, 207, "Could not determine clock root layer");
  return nullptr;
}

// Find nearest grid wire to a location and return grid layer
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

  // Choose closest grid wire 
  odb::Point grid_point;
  if (out_grid_layer) {
    if (nearest_h_layer && nearest_v_layer) {
      if (min_dist_h < min_dist_v) {
        *out_grid_layer = nearest_h_layer;
        for (const auto& wire : h_wires) {
          int wire_y = (wire.rect.yMin() + wire.rect.yMax()) / 2;
          if (wire_y == best_y) {
            int clamped_x = std::max(wire.rect.xMin(), std::min(wire.rect.xMax(), loc.x()));
            grid_point = odb::Point(clamped_x, best_y);
            logger_->report("Nearest grid: horizontal wire at y={}, x extent [{}, {}], clamped x={} (using {})",
                           best_y, wire.rect.xMin(), wire.rect.xMax(), clamped_x, nearest_h_layer->getName());
            break;
          }
        }
      } else {
        *out_grid_layer = nearest_v_layer;
        for (const auto& wire : v_wires) {
          int wire_x = (wire.rect.xMin() + wire.rect.xMax()) / 2;
          if (wire_x == best_x) {
            int clamped_y = std::max(wire.rect.yMin(), std::min(wire.rect.yMax(), loc.y()));
            grid_point = odb::Point(best_x, clamped_y);
            logger_->report("Nearest grid: vertical wire at x={}, y extent [{}, {}], clamped y={} (using {})",
                           best_x, wire.rect.yMin(), wire.rect.yMax(), clamped_y, nearest_v_layer->getName());
            break;
          }
        }
      }
    } else {
      *out_grid_layer = nearest_h_layer ? nearest_h_layer : nearest_v_layer;
      grid_point = odb::Point(best_x, best_y);
    }
  } else {
    grid_point = odb::Point(best_x, best_y);
  }

  return grid_point;
}

// Create via stack between layers (stores vias for later writing)
void ClockMesh::createViaStackAtPoint(const odb::Point& location,
                                     odb::dbTechLayer* from_layer,
                                     odb::dbTechLayer* to_layer,
                                     odb::dbNet* net)
{
  if (!from_layer || !to_layer || !net) {
    logger_->error(MESH, 208, "Invalid parameters for via stack");
    return;
  }

  odb::dbTech* tech = db_->getTech();
  if (!tech) {
    logger_->error(MESH, 209, "No technology available");
    return;
  }

  int from_level = from_layer->getRoutingLevel();
  int to_level = to_layer->getRoutingLevel();

  if (from_level == to_level) {
    logger_->warn(MESH, 210, "No via needed - same layer");
    return;
  }

  if (from_level > to_level) {
    std::swap(from_layer, to_layer);
    std::swap(from_level, to_level);
  }

  logger_->report("Storing via stack from {} (level {}) to {} (level {}) at ({}, {})",
                 from_layer->getName(), from_level,
                 to_layer->getName(), to_level,
                 location.x(), location.y());

  int vias_stored = 0;
  for (int level = from_level; level < to_level; level++) {
    odb::dbTechLayer* lower = tech->findRoutingLayer(level);
    odb::dbTechLayer* upper = tech->findRoutingLayer(level + 1);

    if (!lower || !upper) {
      logger_->warn(MESH, 212, "Could not find routing layers for level {} and {}",
                   level, level + 1);
      continue;
    }

    odb::Rect via_area(location.x(), location.y(), location.x(), location.y());
    connection_vias_.emplace_back(lower, upper, net, via_area);
    vias_stored++;

    logger_->report("  Stored via between {} and {} at ({}, {})",
                   lower->getName(), upper->getName(),
                   location.x(), location.y());
  }

  logger_->report("Via stack storage completed: {} vias stored for later writing",
                 vias_stored);
}

// Create straight wire from root to nearest grid
void ClockMesh::createRootToGridConnection(odb::dbBTerm* root,
                                          const odb::Point& root_loc,
                                          const odb::Point& grid_point,
                                          odb::dbTechLayer* root_layer,
                                          odb::dbTechLayer* grid_layer,
                                          odb::dbNet* mesh_net)
{
  if (!root || !mesh_net || !root_layer || !grid_layer) {
    logger_->error(MESH, 201, "Invalid parameters for root connection");
    return;
  }

  logger_->report("Connecting clock root '{}' at ({}, {}) on {} to grid at ({}, {}) on {}",
                 root->getConstName(), root_loc.x(), root_loc.y(), root_layer->getName(),
                 grid_point.x(), grid_point.y(), grid_layer->getName());

  odb::dbSWire* swire = nullptr;
  auto swires = mesh_net->getSWires();
  if (!swires.empty()) {
    swire = *swires.begin();
  } else {
    swire = odb::dbSWire::create(mesh_net, odb::dbWireType::ROUTED);
  }

  if (!swire) {
    logger_->error(MESH, 203, "Failed to create SWire for root connection");
    return;
  }

  int half_width = mesh_wire_width_ / 2;
  int x1 = root_loc.x();
  int y1 = root_loc.y();
  int x2 = grid_point.x();
  int y2 = grid_point.y();

  // Create single straight wire (either horizontal or vertical)
  if (x1 == x2 && y1 == y2) {
    logger_->warn(MESH, 227,
                 "Clock root at ({},{}) is exactly on grid point - no wire needed",
                 x1, y1);
  } else if (x1 != x2) {
    // Horizontal wire to vertical grid
    odb::dbSBox::create(swire, root_layer,
                       std::min(x1, x2), y1 - half_width,
                       std::max(x1, x2), y1 + half_width,
                       odb::dbWireShapeType::STRIPE);
    logger_->report("Created horizontal wire: ({},{}) to ({},{}) on {}",
                   x1, y1, x2, y1, root_layer->getName());
  } else if (y1 != y2) {
    // Vertical wire to horizontal grid
    odb::dbSBox::create(swire, root_layer,
                       x1 - half_width, std::min(y1, y2),
                       x1 + half_width, std::max(y1, y2),
                       odb::dbWireShapeType::STRIPE);
    logger_->report("Created vertical wire: ({},{}) to ({},{}) on {}",
                   x1, y1, x1, y2, root_layer->getName());
  }

  logger_->report("Root connection wire created on layer {}",
                 root_layer->getName());

  if (root_layer->getRoutingLevel() != grid_layer->getRoutingLevel()) {
    createViaStackAtPoint(grid_point, root_layer, grid_layer, mesh_net);
  }
}

// Connect clock root BTerm to mesh grid
void ClockMesh::connectClockRootToMesh(const std::string& clock_name,
                                      odb::dbNet* mesh_net,
                                      const std::vector<MeshWire>& h_wires,
                                      const std::vector<MeshWire>& v_wires)
{
  if (!mesh_net) {
    logger_->error(MESH, 204, "NULL mesh net provided");
    return;
  }

  if (h_wires.empty() || v_wires.empty()) {
    logger_->error(MESH, 205, "No mesh wires provided for connection");
    return;
  }

  logger_->report("======================================");
  logger_->report("Connecting clock root to mesh for: {}", clock_name);
  logger_->report("======================================");

  odb::dbBTerm* clk_root = mesh_net->get1stBTerm();
  if (!clk_root) {
    logger_->warn(MESH, 206,
                 "No clock root BTerm found on net '{}'. Mesh will not be connected to clock source.",
                 mesh_net->getName());
    return;
  }

  logger_->report("Found clock root BTerm: {}", clk_root->getConstName());

  odb::Point root_loc = getClockRootLocation(clk_root);
  odb::dbTechLayer* root_layer = getRootPinLayer(clk_root);

  if (!root_layer) {
    logger_->error(MESH, 215, "Could not determine root pin layer");
    return;
  }

  logger_->report("Root pin layer: {} (level {})",
                 root_layer->getName(), root_layer->getRoutingLevel());

  // Simple approach: always connect to nearest vertical grid wire
  if (v_wires.empty()) {
    logger_->error(MESH, 216, "No vertical grid wires available");
    return;
  }

  int min_dist = std::numeric_limits<int>::max();
  int nearest_x = 0;
  int wire_y_min = 0;
  int wire_y_max = 0;
  odb::dbTechLayer* v_layer = nullptr;

  for (const auto& wire : v_wires) {
    int wire_x = (wire.rect.xMin() + wire.rect.xMax()) / 2;
    int dist = std::abs(wire_x - root_loc.x());
    if (dist < min_dist) {
      min_dist = dist;
      nearest_x = wire_x;
      wire_y_min = wire.rect.yMin();
      wire_y_max = wire.rect.yMax();
      v_layer = wire.layer;
    }
  }

  // Clamp to wire extent
  int clamped_y = std::max(wire_y_min, std::min(wire_y_max, root_loc.y()));
  odb::Point grid_point(nearest_x, clamped_y);

  logger_->report("Root location: ({},{}), Grid point: ({},{})",
                 root_loc.x(), root_loc.y(), grid_point.x(), grid_point.y());
  logger_->report("Connecting to vertical wire at x={}, distance={}",
                 nearest_x, min_dist);

  // Via stack from root layer to vertical layer at root location
  createViaStackAtPoint(root_loc, root_layer, v_layer, mesh_net);

  // Route on vertical layer from root to grid
  createRootToGridConnection(clk_root, root_loc, grid_point,
                            v_layer, v_layer, mesh_net);

  logger_->report("======================================");
  logger_->report("Clock root connection completed!");
  logger_->report("Root layer: {} (level {}), Vertical grid layer: {} (level {})",
                 root_layer->getName(), root_layer->getRoutingLevel(),
                 v_layer->getName(), v_layer->getRoutingLevel());
  logger_->report("======================================");
}

// Get sink pin layer
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

// Create straight stub wire to nearest grid
void ClockMesh::createSinkStubWire(odb::dbITerm* sink_iterm,
                                  const odb::Point& sink_loc,
                                  const odb::Point& grid_point,
                                  odb::dbTechLayer* sink_layer,
                                  odb::dbTechLayer* grid_layer,
                                  odb::dbNet* mesh_net)
{
  if (!sink_iterm || !mesh_net || !sink_layer || !grid_layer) {
    logger_->warn(MESH, 217, "Invalid parameters for sink stub wire");
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
    logger_->warn(MESH, 218, "Failed to get/create SWire for sink stub");
    return;
  }

  int half_width = mesh_wire_width_ / 2;
  int x1 = sink_loc.x();
  int y1 = sink_loc.y();
  int x2 = grid_point.x();
  int y2 = grid_point.y();

  // Create single straight wire (either horizontal or vertical)
  if (x1 != x2) {
    // Horizontal wire to vertical grid
    odb::dbSBox::create(swire, sink_layer,
                       std::min(x1, x2), y1 - half_width,
                       std::max(x1, x2), y1 + half_width,
                       odb::dbWireShapeType::STRIPE);
  } else if (y1 != y2) {
    // Vertical wire to horizontal grid
    odb::dbSBox::create(swire, sink_layer,
                       x1 - half_width, std::min(y1, y2),
                       x1 + half_width, std::max(y1, y2),
                       odb::dbWireShapeType::STRIPE);
  }

  if (sink_layer->getRoutingLevel() != grid_layer->getRoutingLevel()) {
    createViaStackAtPoint(grid_point, sink_layer, grid_layer, mesh_net);
  }
}

// Connect all clock sinks to mesh grid
void ClockMesh::connectSinksToMesh(const std::string& clock_name,
                                  odb::dbNet* mesh_net,
                                  const std::vector<MeshWire>& h_wires,
                                  const std::vector<MeshWire>& v_wires)
{
  if (!mesh_net) {
    logger_->error(MESH, 219, "NULL mesh net provided for sink connection");
    return;
  }

  if (h_wires.empty() || v_wires.empty()) {
    logger_->error(MESH, 220, "No mesh wires provided for sink connection");
    return;
  }

  if (clockToSinks_.find(clock_name) == clockToSinks_.end()) {
    logger_->error(MESH, 221, "No sinks found for clock: {}", clock_name);
    return;
  }

  const std::vector<ClockSink>& sinks = clockToSinks_[clock_name];

  logger_->report("======================================");
  logger_->report("Connecting {} sinks to mesh for: {}", sinks.size(), clock_name);
  logger_->report("======================================");

  int connected_count = 0;
  int skipped_count = 0;

  for (const auto& sink : sinks) {
    if (!sink.iterm) {
      logger_->warn(MESH, 222, "Sink {} has no ITerms, skipping", sink.name);
      skipped_count++;
      continue;
    }

    odb::Point sink_loc(sink.x, sink.y);

    odb::dbTechLayer* sink_pin_layer = getSinkPinLayer(sink.iterm);
    if (!sink_pin_layer) {
      logger_->warn(MESH, 223, "Could not determine pin layer for sink {}, skipping",
                   sink.name);
      skipped_count++;
      continue;
    }

    odb::dbTechLayer* grid_layer = nullptr;
    odb::Point grid_point = findNearestGridWire(sink_loc, h_wires, v_wires, &grid_layer);

    if (!grid_layer) {
      logger_->warn(MESH, 224, "Could not determine grid layer for sink {}, skipping",
                   sink.name);
      skipped_count++;
      continue;
    }

    sink.iterm->disconnect();
    sink.iterm->connect(mesh_net);

    // Via stack from sink pin to grid layer at sink location
    createViaStackAtPoint(sink_loc, sink_pin_layer, grid_layer, mesh_net);

    // Route on grid layer to grid point
    createSinkStubWire(sink.iterm, sink_loc, grid_point,
                      grid_layer, grid_layer, mesh_net);

    connected_count++;

    if (connected_count % 10 == 0) {
      logger_->report("  Connected {}/{} sinks...", connected_count, sinks.size());
    }
  }

  logger_->report("======================================");
  logger_->report("Sink connection completed!");
  logger_->report("Total sinks: {}, Connected: {}, Skipped: {}",
                 sinks.size(), connected_count, skipped_count);
  logger_->report("======================================");
}

}  // namespace mesh
