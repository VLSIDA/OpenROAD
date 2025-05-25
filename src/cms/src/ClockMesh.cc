// Copyright (c) 2021, The Regents of the University of California
// All rights reserved.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "cms/ClockMesh.hh"

#include "straps.h"
#include "sta/StaMain.hh"

extern "C" {
extern int Cms_Init(Tcl_Interp *interp);
}

namespace cms {

using utl::CMS;
using std::string;
using sta::LibertyLibrary;
using sta::LibertyLibrarySeq;
using sta::LibertyPort;
using odb::dbPlacementStatus;

// Tcl files encoded into strings.
extern const char *cms_tcl_inits[];

ClockMesh::ClockMesh()
{
  this->value_ = 0;
  this->buffer_ptr_ = 0;
  this->buffers_ = nullptr;
  this->dbu_ = db_->getTech()->getDbUnitsPerMicron();
}

ClockMesh::~ClockMesh()
{
  if (this->buffers_ != nullptr) {
    for (int i = 0; i < value_; i++) {
      network_->deleteInstance(buffers_[i]);
    }
    delete[] this->buffers_;
    this->buffers_ = nullptr;
    delete point_;
    this->point_ = nullptr;
  }
  
}

void
ClockMesh::init(Tcl_Interp* tcl_interp,
	   odb::dbDatabase* db,
     sta::dbNetwork* network,
     rsz::Resizer* resizer,
     utl::Logger* logger)
{
  db_ = db;
  logger_ = logger;
  network_ = network;
  resizer_ = resizer;
  // Define swig TCL commands.
  Cms_Init(tcl_interp);
  // Eval encoded cms TCL sources.
  sta::evalTclInit(tcl_interp, cms::cms_tcl_inits);
}

int
ClockMesh::dump_value()
{
  return this->value_;
  logger_->info(CMS, 189, "Dumping ClockMesh Value of {}",value_);
}

int
ClockMesh::set_value(int value)
{
  this->value_ = std::abs(value);
  logger_->info(CMS, 151, "Set ClockMesh Value to {}", value);
  return this->value_;
}

int
ClockMesh::createBufferArray(int amount)
{
  if (amount == 0) {
    logger_->error(CMS, 409, "Need to set CMS Buffer Amount to non zero");
    return 1;
  } else {
    this->buffers_ = new sta::Instance*[amount];
    this->point_ = new Point[amount];
    logger_->info(CMS, 125, "CMS Buffer and Point arrays initialized!");
    return 0;
  }
}

void
ClockMesh::addBuffer()
{
  resizer_->initBlock();
  this->point_[buffer_ptr_].setX(buffer_ptr_);
  this->point_[buffer_ptr_].setY(buffer_ptr_);
  const string buffer_name = makeUniqueInstName("clock_mesh_buffer",true);
  buffers_[buffer_ptr_] = network_->makeInstance(buffer_cells_[0],
                          buffer_name.c_str(),
                          nullptr);
  dbInst* db_inst = resizer_->getDbNetwork()->staToDb(buffers_[buffer_ptr_]);
  //set the location
  setLocation(db_inst, point_[buffer_ptr_]);
  //call legalizer later
  //incremenet area of the design
  resizer_->designAreaIncr(area(db_inst->getMaster()));

  logger_->info(CMS, 95, "CMS added buffer: {} at point X: {} Y: {}",buffer_name, point_[buffer_ptr_].getX(),point_[buffer_ptr_].getY());
  buffer_ptr_++;
}

void
ClockMesh::findBuffers()
{
  if (buffer_cells_.empty()) {
    sta::LibertyLibraryIterator* lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      LibertyLibrary* lib = lib_iter->next();
      for (LibertyCell* buffer : *lib->buffers()) {
        if (!resizer_->dontUse(buffer) && isLinkCell(buffer)) {
          buffer_cells_.emplace_back(buffer);
        }
      }
    }
    delete lib_iter;
    if (buffer_cells_.empty()) {
      logger_->error(CMS, 124, "no buffers found.");
    } else {
      sort(buffer_cells_,
           [this](const LibertyCell* buffer1, const LibertyCell* buffer2) {
             return bufferDriveResistance(buffer1)
                    > bufferDriveResistance(buffer2);
           });
    }
  }
}

std::string
ClockMesh::makeUniqueInstName(const char* base_name, bool underscore)
{
  string inst_name;
  do {
    // sta::stringPrint can lead to string overflow and fatal
    if (underscore) {
      inst_name = fmt::format("{}_{}", base_name, unique_inst_index_++);
    } else {
      inst_name = fmt::format("{}{}", base_name, unique_inst_index_++);
    }
  } while (network_->findInstance(inst_name.c_str()));
  return inst_name;
}

float
ClockMesh::bufferDriveResistance(const LibertyCell* buffer) const
{
  LibertyPort *input, *output;
  buffer->bufferPorts(input, output);
  return output->driveResistance();
}


void
ClockMesh::createGrid()
{
  findBuffers();
  //add one buffer for now
  addBuffer();
}

void 
ClockMesh::makeGrid()
{
  debugPrint(logger_, utl::CMS, "Make", 1, "Build - begin");
  auto* block = db_->getChip()->getBlock();
  // dbSet<dbTrackGrid> tgs = block->getTrackGrids();
  Straps straps_(0, 0);
}

bool
ClockMesh::isLinkCell(LibertyCell* cell) const
{
  return network_->findLibertyCell(cell->name()) == cell;
}

double
ClockMesh::area(dbMaster* master)
{
  if (!master->isCoreAutoPlaceable()) {
    return 0;
  }
  return dbuToMeters(master->getWidth()) * dbuToMeters(master->getHeight());
}

double
ClockMesh::dbuToMeters(int dist) const
{
  return dist / (dbu_ * 1e+6);
}

void
ClockMesh::setLocation(dbInst* db_inst, const Point& pt)
{
  int x = pt.x();
  int y = pt.y();
  //do proper placement later
  db_inst->setPlacementStatus(dbPlacementStatus::PLACED);
  db_inst->setLocation(x, y);
}
} // namespace cms