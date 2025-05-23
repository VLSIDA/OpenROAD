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
#include <string>
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "rsz/Resizer.hh"
#include "sta/Sdc.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/Path.hh"
#include "sta/UnorderedSet.hh"
#include "utl/Logger.h"

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
class Vertex;
class Graph;
}  // namespace sta


namespace cms {

using utl::Logger;

using odb::Point;
using odb::dbMaster;
using odb::dbInst;
using odb::dbPlacementStatus;

using sta::Instance;
using sta::LibertyCell;
using sta::LibertyCellSeq;
using sta::LibertyLibrary;
using sta::LibertyLibrarySeq;


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
  int dump_value();
  int set_value(int value);
  void addBuffer();
  void createGrid();
  void makeGrid();

private:
  std::string makeUniqueInstName(const char* base_name, bool underscore);
  void findBuffers();
  int createBufferArray(int amount);
  bool isLinkCell(LibertyCell* cell) const;
  float bufferDriveResistance(const LibertyCell* buffer) const;
  double area(dbMaster* master);
  double dbuToMeters(int dist) const;
  void setLocation(dbInst* db_inst, const Point& pt);
  sta::Instance** buffers_ = nullptr; 
  odb::dbDatabase *db_ = nullptr;
  Point* point_ = nullptr;
  rsz::Resizer* resizer_ = nullptr;
  sta::dbNetwork* network_ = nullptr;
  utl::Logger* logger_ = nullptr;
  int value_;
  int buffer_ptr_;
  LibertyCellSeq buffer_cells_;
  int unique_inst_index_ = 1;
  int dbu_ = 0;
};

} //  namespace cms
