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

// Tcl files encoded into strings.
extern const char *cms_tcl_inits[];

ClockMesh::ClockMesh()
{
  this->value_ = 0;
}

ClockMesh::~ClockMesh()
{
}

void
ClockMesh::init(Tcl_Interp *tcl_interp,
	   odb::dbDatabase *db)
{
  db_ = db;

  // Define swig TCL commands.
  Cms_Init(tcl_interp);
  // Eval encoded cms TCL sources.
  sta::evalTclInit(tcl_interp, cms::cms_tcl_inits);
}

void
ClockMesh::dumpValue()
{
  printf("Clock Mesh Value: %d\n",this->value_);
}

void
ClockMesh::setValue(int value)
{
  this->value_ = value;
}

} //namespace cms
