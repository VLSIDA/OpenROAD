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

#pragma once

#include <tcl.h>
#include "odb/db.h"
#include "sta/Sdc.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "utl/Logger.h"
#include "rsz/Resizer.hh"

namespace utl {
class Logger;
}

namespace rsz {
class Resizer;
} // namespace rsz

namespace sta {
class dbSta;
class Clock;
class dbNetwork;
class Unit;
class LibertyCell;
class Vertex;
class Graph;
}  // namespace sta


namespace cms {

using utl::Logger;
using odb::Point;

class ClockMesh
{
public:
  ClockMesh();
  ~ClockMesh();
  void init(Tcl_Interp *tcl_interp,
	    odb::dbDatabase *db,
      sta::dbNetwork* network,
      rsz::Resizer* resizer,
      utl::Logger* logger);
  void dumpValue();
  void setValue(int value);
  void addBuffer(LibertyCell* bufferCell, 
                const char* name, 
                Instance* parent, 
                const Point& location);
  int createBufferArray(int amount);

private:
  Instance** buffers_ = nullptr; 
  odb::dbDatabase *db_ = nullptr;
  Point* point_ = nullptr;
  rsz::Resizer* resizer_ = nullptr;
  sta::dbNetwork* network_ = nullptr;
  utl::Logger* logger_ = nullptr;
  int value_;
  int buffer_ptr_;
};

} //  namespace cms
