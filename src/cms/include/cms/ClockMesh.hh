// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "odb/PtrSetMap.h"  // odb::ODBPtrLess (odb deletes std::less<dbObject*>)
#include "odb/db.h"

namespace odb {
class dbWireEncoder;
}

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

struct ClockSink
{
  std::string name;
  int x;
  int y;
  odb::dbITerm* iterm;
  bool isMacro;

  ClockSink(const std::string& n,
            int px,
            int py,
            odb::dbITerm* term,
            bool macro)
      : name(n), x(px), y(py), iterm(term), isMacro(macro)
  {
  }
};

struct MeshWire
{
  odb::dbTechLayer* layer;
  odb::dbNet* net;
  odb::Rect rect;
  bool is_horizontal;

  MeshWire(odb::dbTechLayer* l, odb::dbNet* n, const odb::Rect& r, bool horiz)
      : layer(l), net(n), rect(r), is_horizontal(horiz)
  {
  }
};

struct MeshVia
{
  odb::dbTechLayer* lower_layer;
  odb::dbTechLayer* upper_layer;
  odb::dbNet* net;
  odb::Rect area;

  MeshVia(odb::dbTechLayer* lower,
          odb::dbTechLayer* upper,
          odb::dbNet* n,
          const odb::Rect& a)
      : lower_layer(lower), upper_layer(upper), net(n), area(a)
  {
  }
};

struct GridIntersection
{
  int x;
  int y;
  odb::dbTechLayer* layer;
  // Grid indices (h-wire row, v-wire column); -1 = unassigned. Used for the
  // checkerboard buffer pattern.
  int row = -1;
  int col = -1;
  bool has_buffer = false;
  odb::dbInst* buffer_inst = nullptr;
  odb::dbBTerm* proxy_bterm = nullptr;
  odb::dbInst* tsv_inst = nullptr;  // gt2_6t_TSV front<->back crossing cell

  GridIntersection(int px, int py, odb::dbTechLayer* l) : x(px), y(py), layer(l)
  {
  }
};

// Deterministic deformed/pruned mesh grid, shared by the reserve (floorplan,
// pre-PDN) and connect (post-CTS) phases so a TSV placed early lands on exactly
// the same vertical wire the mesh draws late. Geometry only -- the MeshWire net
// pointers are null; nets are bound when the mesh is actually drawn.
struct FrozenGrid
{
  std::vector<MeshWire> v_wires;  // vertical (v_layer), surviving
  std::vector<MeshWire> h_wires;  // horizontal (h_layer), notched+pruned
  std::vector<GridIntersection> intersections;
  std::vector<std::vector<int>>
      fragments;  // connected comps: indices into intersections
  int aligned_pitch = 0;
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
                      const std::vector<std::string>& buffer_list = {},
                      int macro_halo_dbu = 0,
                      const std::vector<std::string>& cts_buffer_list = {},
                      const std::string& mesh_strategy = "adaptive",
                      bool remove_colliding = false,
                      bool checkerboard_buffers = false);

  void findClockSinks();
  void connectSinksViaRouter(const std::string& clock_name,
                             odb::dbTechLayer* proxy_layer);
  void setupProxyBTerms(const std::string& clock_name,
                        odb::dbTechLayer* proxy_layer);
  void connectProxyBTermsToMesh(const std::string& clock_name);
  void captureLeafArrivals(const std::string& clock_name);
  void mergeNetsToMesh(const std::string& clock_name);
  void convertSWireToWire(const std::string& clock_name);
  void writeMeshSpice(const std::string& clock_name,
                      const std::string& spice_file,
                      float vdd_voltage = 0.0,
                      float rise_time_ns = 0.0,
                      float fall_time_ns = 0.0,
                      const std::vector<std::string>& spice_models = {},
                      bool zero_delay = false,
                      bool full_tree = false,
                      bool finfet = false,
                      float tsv_res = 149.0);
  void writeMeshVerilog(const std::string& clock_name,
                        const std::string& input_filename,
                        const std::string& output_filename);
  std::string getClockNetName() const { return mesh_net_name_; }

  // Verification helper: compute the frozen grid and log a per-fragment report.
  // All distances in dbu. Does not modify the design.
  void reportFrozenGrid(odb::dbTechLayer* h_layer,
                        odb::dbTechLayer* v_layer,
                        int pitch,
                        int v_strap_pitch,
                        int v_strap_offset,
                        int v_strap_width);

  // Phase 1 (RESERVE): runs at floorplan, before tapcell/PDN. Places TSV cells
  // + layer-selective keepouts at the frozen grid's intersections -- >=1 driver
  // per fragment plus a target spacing, each offset ~1 row off the V*H crossing
  // with Y on the vertical line. The keepout blocks placement + power
  // (BPR/BM3/BM4) but leaves M1/BM1/BM2 open so pins stay routable. All dbu.
  void reserveMeshTsvSites(odb::dbTechLayer* h_layer,
                           odb::dbTechLayer* v_layer,
                           int pitch,
                           int v_strap_pitch,
                           int v_strap_offset,
                           int v_strap_width,
                           const std::string& tsv_master,
                           int keepout_w,
                           int keepout_h,
                           int target_spacing);

  // SINK SIDE: place sink-taps that bring the mesh up to local sink-buffers.
  // For each gap between adjacent vertical mesh wires (on each horizontal wire)
  // a candidate tap sits on the midpoint; every FF clock pin is assigned to its
  // nearest tap with load < capacity (spill to next-nearest); only winning taps
  // are placed (sink-TSV + proxy BTerm on the mesh + sink-buffer driving the
  // tap's FFs). Keepouts/BPR breaks are left to break_bpr_at_tsvs(). All dbu.
  void createSinkTaps(odb::dbTechLayer* h_layer,
                      odb::dbTechLayer* v_layer,
                      const std::string& tsv_master,
                      const std::string& sink_buffer_master,
                      int capacity,
                      int halo_dbu);

  // Author sink_tap special wires (buffer.A -> TSV.A) at the buffers' FINAL
  // positions; call AFTER the post-break detailed_placement.
  void connectSinkTaps(bool use_router = false);

  // Re-map each FF clock pin to its NEAREST sink buffer using FINAL (post-
  // placement) positions, capacity-capped. Fixes stale FF->buffer assignments
  // left by the break_bpr_at_tsvs displacement (denser mesh -> more BPR breaks
  // -> more cell movement -> more staleness). Call AFTER the post-break
  // detailed_placement, BEFORE connect_sink_taps.
  void reassignSinkFFs(int capacity);

  // Break the BPR power rails at every front<->back TSV: cut window (+halo) and
  // a relocate_rows-deep placement blockage per TSV, trim the rails over the
  // windows, drop floating BPR stubs, delete stranded taps. Run after the TSVs
  // are placed (drive + sink); caller runs detailed_placement after. All dbu.
  void breakBprAtTsvs(odb::dbTechLayer* bpr_layer,
                      const std::string& tsv_master,
                      const std::string& tap_master,
                      int halo_dbu,
                      int relocate_rows);

 private:
  void findClockRoots(sta::Clock* clk,
                      std::set<odb::dbNet*, odb::ODBPtrLess>& clockNets);
  bool isSink(odb::dbITerm* iterm);
  void computeITermPosition(odb::dbITerm* term, int& x, int& y) const;
  bool separateSinks(odb::dbNet* net, std::vector<ClockSink>& sinks);

  odb::Rect calculateBoundingBox(const std::vector<ClockSink>& sinks);
  void createHorizontalWires(odb::dbNet* net,
                             odb::dbTechLayer* layer,
                             const odb::Rect& bbox,
                             int pitch,
                             std::vector<MeshWire>& wires);
  void createVerticalWires(odb::dbNet* net,
                           odb::dbTechLayer* layer,
                           const odb::Rect& bbox,
                           int pitch,
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
  bool findNearestGridIntersection(const odb::Point& loc,
                                   odb::Point& out_point,
                                   odb::dbTechLayer** out_layer) const;
  void createViaStackAtPoint(const odb::Point& location,
                             odb::dbTechLayer* from_layer,
                             odb::dbTechLayer* to_layer,
                             odb::dbNet* net);
  odb::dbTechLayer* selectBufferLayer(odb::dbTechLayer* h_layer,
                                      odb::dbTechLayer* v_layer);

  void placeBuffersAtIntersections(const std::string& buffer_master,
                                   odb::dbNet* mesh_net);
  odb::dbInst* placeTsvCell(odb::dbMaster* master,
                            const std::string& name,
                            int x,
                            int y,
                            odb::dbOrientType orient);
  // Snaps y to the nearest placement row (so a row-tall cell sits between that
  // row's BPR rails); returns the row y and its orientation.
  bool nearestRow(int y, int& row_y, odb::dbOrientType& orient) const;
  void connectBuffersToNets(odb::dbNet* mesh_net,
                            const std::string& clock_name);
  int createProxyBTermsWithSeparateNets(odb::dbNet* mesh_net,
                                        odb::dbTechLayer* proxy_layer);
  odb::dbITerm* getBufferOutputPin(odb::dbInst* buffer);
  odb::dbITerm* getBufferInputPin(odb::dbInst* buffer);
  // Straight, track-aligned SPECIAL route from a TSV's backside Y pad to the
  // mesh H wire at target_y: a mesh-V-layer stub on the nearest routing track
  // (plus a small patch if the track misses the pad) and an H<->V tech via at
  // (track_x, target_y). Marks the net special (router skips it -- the
  // connection is by construction). Returns the track x (proxy BTerm goes
  // there) or INT_MIN on failure.
  int drawTsvGridStub(odb::dbNet* net, const odb::Rect& ypad, int target_y);
  void buildCtsTreeToBuffers(const std::string& clock_net_name,
                             const std::vector<std::string>& buffer_list);
  void reencodeWireToMesh(odb::dbWire* src_wire, odb::dbWireEncoder& encoder);

  void collectBlockageRects(int halo_dbu);
  bool isBlocked(int x, int y) const;
  bool isBlocked(const odb::Rect& r) const;
  std::vector<odb::Rect> clipWireByBlockages(const odb::Rect& wire_rect,
                                             bool is_horizontal,
                                             int min_segment_length) const;

  // PDN-avoidance: collect vertical power straps, shift a vertical clock wire
  // clear of them, and notch a horizontal clock wire where it crosses one.
  void collectPdnVStraps();
  // Co-planar (fine-layer) PDN: horizontal power straps ON THE MESH H LAYER
  // (BM2). Only same-layer straps conflict -- BPR followpins and any other
  // layer's straps are crossed freely (no via columns punch through the mesh
  // layers with a BM1/BM2 PDN; power vias BV0/BV1 stay inside the strap
  // footprints). Horizontal clock wires SHIFT clear of these bands instead of
  // being notched, so the mesh stays one connected grid.
  void collectPdnHStraps();
  int shiftHClearOfPdn(int y_center, int half_w, int spacing) const;
  // Same merged-interval result as collectPdnVStraps(), but computed from the
  // (deterministic) PDN vertical-strap parameters instead of scanning a built
  // PDN -- so the frozen grid can be computed at floorplan time, before the PDN
  // exists. strap_offset is the distance from core.xMin() to the first strap
  // center; all in dbu.
  void collectPdnVStrapsFromParams(int strap_pitch,
                                   int strap_offset,
                                   int strap_width,
                                   const odb::Rect& core);
  // The single source of truth for the deformed mesh grid. Pure computation
  // (reads core box + track grids, no DB writes); both the reserve and connect
  // phases call this with identical inputs and get identical grids.
  FrozenGrid computeFrozenGrid(odb::dbTechLayer* h_layer,
                               odb::dbTechLayer* v_layer,
                               int pitch,
                               int v_strap_pitch,
                               int v_strap_offset,
                               int v_strap_width);
  int shiftVClearOfPdn(int x_center, int half_w, int spacing) const;
  std::vector<odb::Rect> notchHByPdn(const odb::Rect& seg,
                                     int spacing,
                                     int min_segment_length) const;
  void pruneOrphanHSegments(std::vector<MeshWire>& h_wires,
                            const std::vector<MeshWire>& v_wires) const;

  ord::OpenRoad* openroad_ = nullptr;
  bool mesh_generated_ = false;
  // -remove_colliding_wires: set in createMeshGrid, read in
  // placeBuffersAtIntersections to skip mesh drivers that land on power straps.
  bool remove_colliding_ = false;
  // -checkerboard_buffers: set in createMeshGrid, read in
  // placeBuffersAtIntersections — drivers only at (row+col) even intersections
  // (no driver above/below/left/right of any driver); mesh wires unchanged.
  bool checkerboard_buffers_ = false;
  // connect_sink_taps -use_router: leave sink_tap nets as ordinary routed nets
  // (GRT/DRT route them); skips both the special-wire authoring and the
  // SPICE-prep re-author, so the deck extracts the router's wire.
  bool taps_use_router_ = false;

  odb::dbDatabase* db_ = nullptr;
  odb::dbBlock* block_ = nullptr;
  sta::dbSta* sta_ = nullptr;
  sta::dbNetwork* network_ = nullptr;
  utl::Logger* logger_ = nullptr;

  std::map<std::string, std::vector<ClockSink>> clockToSinks_;
  std::set<odb::dbNet*, odb::ODBPtrLess> visitedClockNets_;

  std::vector<MeshWire> mesh_wires_;
  std::vector<MeshVia> connection_vias_;
  odb::dbTechLayer* mesh_h_layer_ = nullptr;
  odb::dbTechLayer* mesh_v_layer_ = nullptr;
  odb::dbTechLayer* bterm_layer_ = nullptr;
  odb::dbTechLayer* proxy_layer_ = nullptr;
  std::vector<GridIntersection> grid_intersections_;
  std::string mesh_net_name_;
  int sink_bterm_counter_ = 0;

  // Connection points where routed wires meet mesh grid wires (x, y,
  // routing_level) Used by convertSWireToWire to break mesh segments at these
  // points
  std::set<std::tuple<int, int, int>> mesh_connection_points_;

  // Map from (x, y, layer-level) on the mesh net's dbWire to the proxy bterm
  // name at that position. Populated by convertSWireToWire when emitting
  // break points. Used by writeMeshSpice (when merge_mesh_nets is skipped)
  // to alias the mesh-net's internal node at each proxy bterm position to
  // the bterm's SPICE name, so sub-net stubs electrically merge with the
  // mesh stripes at those points.
  std::map<std::tuple<int, int, int>, std::string> proxy_alias_;

  // Cached CTS leaf net arrival times (seconds → nanoseconds)
  // Populated by captureLeafArrivals() before merge, used by writeMeshSpice()
  std::map<std::string, float> leaf_arrivals_ns_;

  // Cached CTS leaf net rise slews (seconds → nanoseconds), same key scheme
  // as leaf_arrivals_ns_. Populated alongside arrivals. writeMeshSpice() uses
  // these as the per-buffer Vclk PULSE rise/fall when zero_delay=false and
  // the user did not pass an explicit -rise_time / -fall_time override.
  std::map<std::string, float> leaf_slews_ns_;

  // Halo-expanded bounding boxes of macros + dbBlockages.
  // Populated by collectBlockageRects() at the start of createMeshGrid().
  // Used to skip mesh wires, vias, intersections, and buffers inside macros.
  std::vector<odb::Rect> blockage_rects_;

  // Merged x-intervals of vertical PDN power straps (BM3). Populated by
  // collectPdnVStraps(). Vertical clock wires shift clear of these; horizontal
  // clock wires are notched at them. Empty => behavior unchanged.
  std::vector<std::pair<int, int>> pdn_vstrap_x_;

  // Merged y-intervals of horizontal PDN power straps ON the mesh H layer
  // (co-planar BM1/BM2 PDN). Populated by collectPdnHStraps(). Horizontal
  // clock wires shift clear of these. Empty => behavior unchanged.
  std::vector<std::pair<int, int>> pdn_hstrap_y_;
};

void initClockMesh(ord::OpenRoad* openroad);

}  // namespace cms
