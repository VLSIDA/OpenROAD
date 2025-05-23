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


enum ExtensionMode
{
  CORE,
  RINGS,
  BOUNDARY,
  FIXED
};

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

class VoltageDomain;
class Grid;
class PowerCell;
class CMSRenderer;
class SRoute;

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


  void reset();
  void resetShapes();
  void report();

  // Power cells
  PowerCell* findSwitchedPowerCell(const std::string& name) const;
  void makeSwitchedPowerCell(odb::dbMaster* master,
                             odb::dbMTerm* control,
                             odb::dbMTerm* acknowledge,
                             odb::dbMTerm* switched_power,
                             odb::dbMTerm* alwayson_power,
                             odb::dbMTerm* ground);

  // Domains
  std::vector<VoltageDomain*> getDomains() const;
  VoltageDomain* findDomain(const std::string& name);
  void setCoreDomain(odb::dbNet* power,
                     odb::dbNet* switched_power,
                     odb::dbNet* ground,
                     const std::vector<odb::dbNet*>& secondary);
  void makeRegionVoltageDomain(const std::string& name,
                               odb::dbNet* power,
                               odb::dbNet* switched_power,
                               odb::dbNet* ground,
                               const std::vector<odb::dbNet*>& secondary_nets,
                               odb::dbRegion* region);

  // Grids
  void buildGrids(bool trim);
  std::vector<Grid*> findGrid(const std::string& name) const;
  void makeCoreGrid(VoltageDomain* domain,
                    const std::string& name,
                    StartsWith starts_with,
                    const std::vector<odb::dbTechLayer*>& pin_layers,
                    const std::vector<odb::dbTechLayer*>& generate_obstructions,
                    PowerCell* powercell,
                    odb::dbNet* powercontrol,
                    const char* powercontrolnetwork);
  void makeInstanceGrid(
      VoltageDomain* domain,
      const std::string& name,
      StartsWith starts_with,
      odb::dbInst* inst,
      const std::array<int, 4>& halo,
      bool pg_pins_to_boundary,
      bool default_grid,
      const std::vector<odb::dbTechLayer*>& generate_obstructions,
      bool is_bump);
  void makeExistingGrid(
      const std::string& name,
      const std::vector<odb::dbTechLayer*>& generate_obstructions);

  // Shapes
  void makeRing(Grid* grid,
                odb::dbTechLayer* layer0,
                int width0,
                int spacing0,
                odb::dbTechLayer* layer1,
                int width1,
                int spacing1,
                StartsWith starts_with,
                const std::array<int, 4>& offset,
                const std::array<int, 4>& pad_offset,
                bool extend,
                const std::vector<odb::dbTechLayer*>& pad_pin_layers,
                const std::vector<odb::dbNet*>& nets,
                bool allow_out_of_die);
  void makeFollowpin(Grid* grid,
                     odb::dbTechLayer* layer,
                     int width,
                     ExtensionMode extend);
  void makeStrap(Grid* grid,
                 odb::dbTechLayer* layer,
                 int width,
                 int spacing,
                 int pitch,
                 int offset,
                 int number_of_straps,
                 bool snap,
                 StartsWith starts_with,
                 ExtensionMode extend,
                 const std::vector<odb::dbNet*>& nets);
  void makeConnect(Grid* grid,
                   odb::dbTechLayer* layer0,
                   odb::dbTechLayer* layer1,
                   int cut_pitch_x,
                   int cut_pitch_y,
                   const std::vector<odb::dbTechViaGenerateRule*>& vias,
                   const std::vector<odb::dbTechVia*>& techvias,
                   int max_rows,
                   int max_columns,
                   const std::vector<odb::dbTechLayer*>& ongrid,
                   const std::map<odb::dbTechLayer*, int>& split_cuts,
                   const std::string& dont_use_vias);

  void writeToDb(bool add_pins, const std::string& report_file = "") const;
  void ripUp(odb::dbNet* net);

  void setDebugRenderer(bool on);
  void rendererRedraw();
  void setAllowRepairChannels(bool allow);
  void filterVias(const std::string& filter);

  void checkSetup() const;

  void repairVias(const std::set<odb::dbNet*>& nets);

  void createSrouteWires(const char* net,
                         const char* outerNet,
                         odb::dbTechLayer* layer0,
                         odb::dbTechLayer* layer1,
                         int cut_pitch_x,
                         int cut_pitch_y,
                         const std::vector<odb::dbTechViaGenerateRule*>& vias,
                         const std::vector<odb::dbTechVia*>& techvias,
                         int max_rows,
                         int max_columns,
                         const std::vector<odb::dbTechLayer*>& ongrid,
                         std::vector<int> metalWidths,
                         std::vector<int> metalspaces,
                         const std::vector<odb::dbInst*>& insts);

  PDNRenderer* getDebugRenderer() const { return debug_renderer_.get(); }
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

  
  void trimShapes();
  void updateVias();
  void cleanupVias();

  void checkDesign(odb::dbBlock* block) const;

  std::vector<Grid*> getGrids() const;
  Grid* instanceGrid(odb::dbInst* inst) const;

  VoltageDomain* getCoreDomain() const;
  void ensureCoreDomain();

  void updateRenderer() const;

  bool importUPF(VoltageDomain* domain);
  bool importUPF(Grid* grid, PowerSwitchNetworkType type) const;

  odb::dbDatabase* db_;
  utl::Logger* logger_;

  std::unique_ptr<SRoute> sroute_;
  std::unique_ptr<PDNRenderer> debug_renderer_;

  std::unique_ptr<VoltageDomain> core_domain_;
  std::vector<std::unique_ptr<VoltageDomain>> domains_;
  std::vector<std::unique_ptr<PowerCell>> switched_power_cells_;
};

} //  namespace cms
