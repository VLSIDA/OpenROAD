// SPDX-License-Identifier: BSD-3-Clause

#include "mesh/ClockMesh.hh"

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "odb/dbShape.h"
#include "ord/OpenRoad.hh"
#include "sta/Liberty.hh"
#include "sta/Sdc.hh"
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
          clockToSinks_[clkName].insert(
              clockToSinks_[clkName].end(), sinks.begin(), sinks.end());
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

}  // namespace mesh
