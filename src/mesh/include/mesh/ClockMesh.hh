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

// Clock sink information
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

// Mesh wire representation
struct MeshWire {
  odb::dbTechLayer* layer;
  odb::dbNet* net;
  odb::Rect rect;
  bool is_horizontal;
  odb::dbSBox* db_shape;

  MeshWire(odb::dbTechLayer* l, odb::dbNet* n, const odb::Rect& r, bool horiz)
    : layer(l), net(n), rect(r), is_horizontal(horiz), db_shape(nullptr) {}
};

// Mesh via representation
struct MeshVia {
  odb::dbTechLayer* lower_layer;
  odb::dbTechLayer* upper_layer;
  odb::dbNet* net;
  odb::Rect area;
  odb::dbSBox* db_via;

  MeshVia(odb::dbTechLayer* lower, odb::dbTechLayer* upper,
          odb::dbNet* n, const odb::Rect& a)
    : lower_layer(lower), upper_layer(upper), net(n), area(a), db_via(nullptr) {}
};

// Grid intersection representation
struct GridIntersection {
  int x;
  int y;
  odb::dbTechLayer* layer;
  bool has_buffer = false;
  odb::dbInst* buffer_inst = nullptr;

  GridIntersection(int px, int py, odb::dbTechLayer* l)
    : x(px), y(py), layer(l) {}
};

class ClockMesh
{
 public:
  ClockMesh();
  ~ClockMesh() = default;

  void init(ord::OpenRoad* openroad);
  void run(const char* name);
  bool meshGenerated() const { return mesh_generated_; }

  void createMeshGrid(const std::string& clock_name,
                      odb::dbTechLayer* h_layer,
                      odb::dbTechLayer* v_layer,
                      int wire_width,
                      int pitch,
                      const std::vector<std::string>& buffer_list = {},
                      odb::dbTechLayer* tree_layer = nullptr);

  void findClockSinks();
  void testPrintSinks();

  // Connect sinks to mesh
  void connectSinks(const std::string& clock_name);

  // Connect buffers to their intersections
  void connectBuffersToIntersections(const std::string& clock_name);

  // Get the tree net name for CTS integration
  std::string getTreeNetName(const std::string& clock_name) const {
    return clock_name + "_tree";
  }

 private:
  // Sink finding helpers
  void findClockRoots(sta::Clock* clk, std::set<odb::dbNet*>& clockNets);
  bool isSink(odb::dbITerm* iterm);
  float getInputPinCap(odb::dbITerm* iterm);
  bool hasInsertionDelay(odb::dbInst* inst, odb::dbMTerm* mterm);
  double computeInsertionDelay(const std::string& name,
                               odb::dbInst* inst,
                               odb::dbMTerm* mterm);
  void computeITermPosition(odb::dbITerm* term, int& x, int& y) const;
  bool separateSinks(odb::dbNet* net,
                     std::vector<ClockSink>& sinks);

  // Sink connection helpers (internal)
  void connectSinksToMesh(const std::string& clock_name,
                         odb::dbNet* mesh_net,
                         const std::vector<MeshWire>& h_wires,
                         const std::vector<MeshWire>& v_wires);

  // Mesh grid creation helpers
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

  // Sink connection helpers
  odb::Point findNearestGridWire(const odb::Point& loc,
                                const std::vector<MeshWire>& h_wires,
                                const std::vector<MeshWire>& v_wires,
                                odb::dbTechLayer** out_grid_layer);
  void createViaStackAtPoint(const odb::Point& location,
                            odb::dbTechLayer* from_layer,
                            odb::dbTechLayer* to_layer,
                            odb::dbNet* net);
  void createSinkStubWire(odb::dbITerm* sink_iterm,
                         const odb::Point& sink_loc,
                         const odb::Point& grid_point,
                         odb::dbTechLayer* sink_layer,
                         odb::dbTechLayer* grid_layer,
                         odb::dbNet* mesh_net);
  odb::dbTechLayer* getSinkPinLayer(odb::dbITerm* iterm);

  // Grid intersection helpers
  GridIntersection* findNearestGridIntersection(int x, int y);
  odb::dbTechLayer* selectBufferLayer(odb::dbTechLayer* h_layer,
                                      odb::dbTechLayer* v_layer);

  // Buffer placement and connection
  void placeBuffersAtIntersections(const std::string& buffer_master,
                                   odb::dbNet* mesh_net);
  void connectBuffersToNets(odb::dbNet* mesh_net, const std::string& clock_name);
  void createBufferStubWire(const GridIntersection& inter,
                            odb::dbNet* mesh_net,
                            odb::dbSWire* swire,
                            odb::dbTechLayer* li1_layer);
  odb::dbITerm* getBufferOutputPin(odb::dbInst* buffer);
  odb::dbITerm* getBufferInputPin(odb::dbInst* buffer);

  // CTS integration 
  void buildCtsTreeToBuffers(const std::string& clock_name);
  void createCtsTreeWires(const std::string& clock_name);

  ord::OpenRoad* openroad_ = nullptr;
  bool mesh_generated_ = false;

  odb::dbDatabase* db_ = nullptr;
  odb::dbBlock* block_ = nullptr;
  sta::dbSta* sta_ = nullptr;
  sta::dbNetwork* network_ = nullptr;
  utl::Logger* logger_ = nullptr;

  std::map<std::string, std::vector<ClockSink>> clockToSinks_;
  std::set<odb::dbNet*> staClockNets_;
  std::set<odb::dbNet*> visitedClockNets_;

  std::vector<MeshWire> mesh_wires_;
  std::vector<MeshVia> mesh_vias_;
  std::vector<MeshVia> connection_vias_;
  odb::Rect mesh_bbox_;
  int mesh_wire_width_ = 0;
  int mesh_pitch_ = 0;
  odb::dbTechLayer* mesh_h_layer_ = nullptr;
  odb::dbTechLayer* mesh_v_layer_ = nullptr;
  odb::dbTechLayer* tree_layer_ = nullptr;

  // Grid intersections for buffer placement
  std::vector<GridIntersection> grid_intersections_;
};

void initClockMesh(ord::OpenRoad* openroad);

}  // namespace mesh
