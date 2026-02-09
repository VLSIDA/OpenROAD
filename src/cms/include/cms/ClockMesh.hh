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

namespace cms {

struct ClockSink {
  std::string name;
  int x;
  int y;
  odb::dbITerm* iterm;
  bool isMacro;

  ClockSink(const std::string& n, int px, int py, odb::dbITerm* term, bool macro)
    : name(n), x(px), y(py), iterm(term), isMacro(macro) {}
};

struct MeshWire {
  odb::dbTechLayer* layer;
  odb::dbNet* net;
  odb::Rect rect;
  bool is_horizontal;

  MeshWire(odb::dbTechLayer* l, odb::dbNet* n, const odb::Rect& r, bool horiz)
    : layer(l), net(n), rect(r), is_horizontal(horiz) {}
};

struct MeshVia {
  odb::dbTechLayer* lower_layer;
  odb::dbTechLayer* upper_layer;
  odb::dbNet* net;
  odb::Rect area;

  MeshVia(odb::dbTechLayer* lower, odb::dbTechLayer* upper,
          odb::dbNet* n, const odb::Rect& a)
    : lower_layer(lower), upper_layer(upper), net(n), area(a) {}
};

struct GridIntersection {
  int x;
  int y;
  odb::dbTechLayer* layer;
  bool has_buffer = false;
  odb::dbInst* buffer_inst = nullptr;
  odb::dbBTerm* proxy_bterm = nullptr;

  GridIntersection(int px, int py, odb::dbTechLayer* l)
    : x(px), y(py), layer(l) {}
};

class ClockMesh
{
 public:
  ClockMesh();
  ~ClockMesh() = default;

  void init(ord::OpenRoad* openroad);
  bool meshGenerated() const { return mesh_generated_; }

  void createMeshGrid(const std::string& clock_name,
                      odb::dbTechLayer* h_layer,
                      odb::dbTechLayer* v_layer,
                      int pitch,
                      const std::vector<std::string>& buffer_list = {});

  void findClockSinks();
  void connectSinksViaRouter(const std::string& clock_name,
                              odb::dbTechLayer* proxy_layer);
  void setupProxyBTerms(const std::string& clock_name,
                        odb::dbTechLayer* proxy_layer);
  void connectProxyBTermsToMesh(const std::string& clock_name);
  void writeMeshVerilog(const std::string& clock_name,
                        const std::string& input_filename,
                        const std::string& output_filename);
  std::string getClockNetName() const { return mesh_net_name_; }

 private:
  void findClockRoots(sta::Clock* clk, std::set<odb::dbNet*>& clockNets);
  bool isSink(odb::dbITerm* iterm);
  void computeITermPosition(odb::dbITerm* term, int& x, int& y) const;
  bool separateSinks(odb::dbNet* net, std::vector<ClockSink>& sinks);

  odb::Rect calculateBoundingBox(const std::vector<ClockSink>& sinks);
  void createHorizontalWires(odb::dbNet* net, odb::dbTechLayer* layer,
                             const odb::Rect& bbox, int pitch,
                             std::vector<MeshWire>& wires);
  void createVerticalWires(odb::dbNet* net, odb::dbTechLayer* layer,
                           const odb::Rect& bbox, int pitch,
                           std::vector<MeshWire>& wires);
  void createViasAtIntersections(const std::vector<MeshWire>& h_wires,
                                 const std::vector<MeshWire>& v_wires,
                                 std::vector<MeshVia>& vias);
  void writeWiresToDb(const std::vector<MeshWire>& wires);
  void writeViasToDb(const std::vector<MeshVia>& vias);
  odb::dbNet* getOrCreateClockNet(const std::string& clock_name);

  odb::Point findNearestGridWire(const odb::Point& loc,
                                 const std::vector<MeshWire>& h_wires,
                                 const std::vector<MeshWire>& v_wires,
                                 odb::dbTechLayer** out_grid_layer);
  void createViaStackAtPoint(const odb::Point& location,
                             odb::dbTechLayer* from_layer,
                             odb::dbTechLayer* to_layer,
                             odb::dbNet* net);
  odb::dbTechLayer* selectBufferLayer(odb::dbTechLayer* h_layer,
                                      odb::dbTechLayer* v_layer);

  void placeBuffersAtIntersections(const std::string& buffer_master,
                                   odb::dbNet* mesh_net);
  void connectBuffersToNets(odb::dbNet* mesh_net, const std::string& clock_name);
  int createProxyBTermsWithSeparateNets(odb::dbNet* mesh_net,
                                        odb::dbTechLayer* proxy_layer);
  odb::dbITerm* getBufferOutputPin(odb::dbInst* buffer);
  odb::dbITerm* getBufferInputPin(odb::dbInst* buffer);
  void buildCtsTreeToBuffers(const std::string& clock_net_name);

  ord::OpenRoad* openroad_ = nullptr;
  bool mesh_generated_ = false;

  odb::dbDatabase* db_ = nullptr;
  odb::dbBlock* block_ = nullptr;
  sta::dbSta* sta_ = nullptr;
  sta::dbNetwork* network_ = nullptr;
  utl::Logger* logger_ = nullptr;

  std::map<std::string, std::vector<ClockSink>> clockToSinks_;
  std::set<odb::dbNet*> visitedClockNets_;

  std::vector<MeshWire> mesh_wires_;
  std::vector<MeshVia> connection_vias_;
  odb::dbTechLayer* mesh_h_layer_ = nullptr;
  odb::dbTechLayer* mesh_v_layer_ = nullptr;
  odb::dbTechLayer* bterm_layer_ = nullptr;
  odb::dbTechLayer* proxy_layer_ = nullptr;
  std::vector<GridIntersection> grid_intersections_;
  std::string mesh_net_name_;
  int sink_bterm_counter_ = 0;
};

void initClockMesh(ord::OpenRoad* openroad);

}  // namespace cms
