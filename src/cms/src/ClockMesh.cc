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
#include "sta/StaMain.hh"


extern "C" {
extern int Cms_Init(Tcl_Interp *interp);
}

namespace cms {

using utl::CMS;

// Tcl files encoded into strings.
extern const char *cms_tcl_inits[];

ClockMesh::ClockMesh()
{
  this->value_ = 0;
  this->buffer_ptr_ = 0;
  this->buffers_ = nullptr;
}

ClockMesh::~ClockMesh()
{
  if (this->buffers_ != nullptr) {
    for (auto buffer : this->buffers_: buffers_) {
      removeBuffer(buffer);
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

void
ClockMesh::dumpValue()
{
  logger_->info(CMS, 001, "Dumping ClockMesh Value of {}",value_);
}

void
ClockMesh::setValue(int value)
{
  this->value_ = std::abs(value);
  logger_->info(CMS, 002, "Set ClockMesh Value to {}", value);
}

int
ClockMesh::createBufferArray(int amount)
{
  if (amount == 0) {
    logger_->info(CMS, 003, "Need to set CMS Buffer Amount to non zero");
    return 1;
  } else {
    this->buffers_ = new sta::Instance*[amount];
    this->point_ = new Point[amount];
    logger_->info(CMS, 004, "CMS Buffer and Point arrays initialized!");
    return 0;
  }
}

void
ClockMesh::addBuffer()
{
  this->point_[buffer_ptr_].setX(buffer_ptr_);
  this->point_[buffer_ptr_].setY(buffer_ptr_);
  string buffer_name = makeUniqueInstName("buffer")
  buffers_[buffer_ptr_] = makeBuffer(resizer_->buffer_lowest_drive, 
                          buffer_name.c_str(), 
                          nullptr, 
                          point_[buffer_ptr_]);
  logger_->info(CMS, 005, "CMS added buffer: {} at point X: {} Y: {}",buffer_name, point_[buffer_ptr_].getX(),point_[buffer_ptr_].getY());
  buffer_ptr_++;
}

} // namespace cms
