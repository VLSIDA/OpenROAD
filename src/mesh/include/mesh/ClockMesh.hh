// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"

namespace utl {
class Logger;
}

namespace sta {
class dbSta;
class dbNetwork;
class Clock;
}  // namespace sta

namespace ord {
class OpenRoad;
}

namespace mesh {

// Data structure to hold clock sink information
struct ClockSink {
  std::string name;
  int x;
  int y;
  float inputCap;
  double insertionDelay;
  odb::dbITerm* iterm;
  bool isMacro;

  ClockSink(const std::string& n, int px, int py, float cap,
            double insDelay, odb::dbITerm* term, bool macro)
    : name(n), x(px), y(py), inputCap(cap),
      insertionDelay(insDelay), iterm(term), isMacro(macro) {}
};

// Data structure to hold mesh wire information (similar to PDN Shape)
struct MeshWire {
  odb::dbTechLayer* layer;
  odb::dbNet* net;
  odb::Rect rect;
  bool is_horizontal;
  odb::dbSBox* db_shape;  // Database shape created

  MeshWire(odb::dbTechLayer* l, odb::dbNet* n, const odb::Rect& r, bool horiz)
    : layer(l), net(n), rect(r), is_horizontal(horiz), db_shape(nullptr) {}
};

// Data structure to hold via information (similar to PDN Via)
struct MeshVia {
  odb::dbTechLayer* lower_layer;
  odb::dbTechLayer* upper_layer;
  odb::dbNet* net;
  odb::Rect area;  // Intersection area
  odb::dbSBox* db_via;  // Database via created

  MeshVia(odb::dbTechLayer* lower, odb::dbTechLayer* upper,
          odb::dbNet* n, const odb::Rect& a)
    : lower_layer(lower), upper_layer(upper), net(n), area(a), db_via(nullptr) {}
};

class ClockMesh
{
 public:
  ClockMesh();

  void init(ord::OpenRoad* openroad);
  void run(const char* name);
  bool meshGenerated() const { return mesh_generated_; }

  // Main mesh grid creation functions
  void createMeshGrid(const std::string& clock_name,
                      odb::dbTechLayer* h_layer,
                      odb::dbTechLayer* v_layer,
                      int wire_width,
                      int pitch,
                      const std::vector<std::string>& buffer_list = {});

  void findClockSinks();
  void testPrintSinks();

 private:

  void findClockRoots(sta::Clock* clk, std::set<odb::dbNet*>& clockNets);
  bool isSink(odb::dbITerm* iterm);
  float getInputPinCap(odb::dbITerm* iterm);
  bool hasInsertionDelay(odb::dbInst* inst, odb::dbMTerm* mterm);
  double computeInsertionDelay(const std::string& name,
                               odb::dbInst* inst,
                               odb::dbMTerm* mterm);
  void computeITermPosition(odb::dbITerm* term, int& x, int& y) const;

  // Helper for CTS-style sink finding
  bool separateSinks(odb::dbNet* net,
                     std::vector<ClockSink>& sinks);

  // Mesh grid creation helpers (following PDN pattern)
  odb::Rect calculateBoundingBox(const std::vector<ClockSink>& sinks);
  void createHorizontalWires(odb::dbNet* net,
                             odb::dbTechLayer* layer,
                             const odb::Rect& bbox,
                             int wire_width,
                             int pitch,
                             std::vector<MeshWire>& wires);
  void createVerticalWires(odb::dbNet* net,
                           odb::dbTechLayer* layer,
                           const odb::Rect& bbox,
                           int wire_width,
                           int pitch,
                           std::vector<MeshWire>& wires);
  void createViasAtIntersections(const std::vector<MeshWire>& h_wires,
                                 const std::vector<MeshWire>& v_wires,
                                 std::vector<MeshVia>& vias);
  void writeWiresToDb(const std::vector<MeshWire>& wires);
  void writeViasToDb(const std::vector<MeshVia>& vias);
  odb::dbNet* getOrCreateClockNet(const std::string& clock_name);

  // Member variables
  ord::OpenRoad* openroad_ = nullptr;
  bool mesh_generated_ = false;

  // Database pointers
  odb::dbDatabase* db_ = nullptr;
  odb::dbBlock* block_ = nullptr;
  sta::dbSta* sta_ = nullptr;
  sta::dbNetwork* network_ = nullptr;
  utl::Logger* logger_ = nullptr;

  // Storage for found sinks
  std::map<std::string, std::vector<ClockSink>> clockToSinks_;
  std::set<odb::dbNet*> staClockNets_;
  std::set<odb::dbNet*> visitedClockNets_;

  // Storage for mesh grid elements
  std::vector<MeshWire> mesh_wires_;
  std::vector<MeshVia> mesh_vias_;
  odb::Rect mesh_bbox_;  // Bounding box for position-aware load calculation
};

void initClockMesh(ord::OpenRoad* openroad);

}  // namespace mesh
