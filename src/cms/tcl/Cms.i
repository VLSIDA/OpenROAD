%module cms

%{
#include "cms/ClockMesh.hh"
#include "ord/OpenRoad.hh"
#include "db_sta/dbSta.hh"
#include "sta/Scene.hh"
#include <tcl.h>

namespace ord {
extern cms::ClockMesh* getClockMesh();
}
%}

%include "../../Exception.i"

%inline %{

namespace cms {

// Create mesh grid command - creates clock mesh with horizontal and vertical wires
// Wire width is automatically taken from tech file (layer default width)
void create_mesh_grid_cmd(const char* clock_name,
                          const char* h_layer_name,
                          const char* v_layer_name,
                          int pitch,
                          Tcl_Obj* buffer_list_obj,
                          int macro_halo_dbu,
                          Tcl_Obj* cts_buffer_list_obj,
                          const char* mesh_strategy,
                          int remove_colliding,
                          int checkerboard_buffers)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }

  ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
  odb::dbDatabase* db = openroad->getDb();
  if (!db) {
    return;
  }

  odb::dbTech* tech = db->getTech();
  if (!tech) {
    return;
  }

  // Find horizontal layer
  odb::dbTechLayer* h_layer = nullptr;
  if (h_layer_name && h_layer_name[0] != '\0') {
    h_layer = tech->findLayer(h_layer_name);
    if (!h_layer) {
      openroad->getLogger()->error(utl::CMS, 200,
                                  "Horizontal layer '{}' not found", h_layer_name);
      return;
    }
  }

  // Find vertical layer
  odb::dbTechLayer* v_layer = nullptr;
  if (v_layer_name && v_layer_name[0] != '\0') {
    v_layer = tech->findLayer(v_layer_name);
    if (!v_layer) {
      openroad->getLogger()->error(utl::CMS, 201,
                                  "Vertical layer '{}' not found", v_layer_name);
      return;
    }
  }

  // Convert mesh buffer TCL list to C++ vector<string>
  std::vector<std::string> buffer_list;
  if (buffer_list_obj) {
    int list_len = 0;
    Tcl_Obj** list_elements = nullptr;
    if (Tcl_ListObjGetElements(nullptr, buffer_list_obj, &list_len, &list_elements) == TCL_OK) {
      for (int i = 0; i < list_len; ++i) {
        const char* buf_name = Tcl_GetString(list_elements[i]);
        if (buf_name && buf_name[0] != '\0') {
          buffer_list.push_back(buf_name);
        }
      }
    }
  }

  // Convert CTS buffer TCL list to C++ vector<string> (empty = auto-infer)
  std::vector<std::string> cts_buffer_list;
  if (cts_buffer_list_obj) {
    int list_len = 0;
    Tcl_Obj** list_elements = nullptr;
    if (Tcl_ListObjGetElements(nullptr, cts_buffer_list_obj, &list_len, &list_elements) == TCL_OK) {
      for (int i = 0; i < list_len; ++i) {
        const char* buf_name = Tcl_GetString(list_elements[i]);
        if (buf_name && buf_name[0] != '\0') {
          cts_buffer_list.push_back(buf_name);
        }
      }
    }
  }

  std::string strategy_str = (mesh_strategy && mesh_strategy[0] != '\0')
                                 ? mesh_strategy
                                 : "adaptive";

  // Call the main mesh grid creation function (wire width auto-computed from tech)
  mesh_obj->createMeshGrid(clock_name, h_layer, v_layer, pitch, buffer_list, macro_halo_dbu, cts_buffer_list, strategy_str, remove_colliding != 0, checkerboard_buffers != 0);
}

// Verification: compute the deterministic frozen grid and log a fragment report.
// All ints in dbu. Does not modify the design.
void compute_frozen_grid_cmd(const char* h_layer_name, const char* v_layer_name,
                             int pitch, int v_strap_pitch, int v_strap_offset,
                             int v_strap_width)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
  odb::dbDatabase* db = openroad->getDb();
  if (!db) {
    return;
  }
  odb::dbTech* tech = db->getTech();
  if (!tech) {
    return;
  }
  odb::dbTechLayer* h_layer = tech->findLayer(h_layer_name);
  odb::dbTechLayer* v_layer = tech->findLayer(v_layer_name);
  if (!h_layer || !v_layer) {
    openroad->getLogger()->error(utl::CMS, 202, "mesh layer not found");
    return;
  }
  mesh_obj->reportFrozenGrid(h_layer, v_layer, pitch, v_strap_pitch,
                             v_strap_offset, v_strap_width);
}

// Phase 1 reserve: place TSV cells + layer-selective keepouts on the frozen
// grid. All ints in dbu. Run at floorplan, before tapcell/PDN.
void reserve_clock_mesh_cmd(const char* h_layer_name, const char* v_layer_name,
                            int pitch, int v_strap_pitch, int v_strap_offset,
                            int v_strap_width, const char* tsv_master,
                            int keepout_w, int keepout_h, int target_spacing)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
  odb::dbDatabase* db = openroad->getDb();
  if (!db) {
    return;
  }
  odb::dbTech* tech = db->getTech();
  if (!tech) {
    return;
  }
  odb::dbTechLayer* h_layer = tech->findLayer(h_layer_name);
  odb::dbTechLayer* v_layer = tech->findLayer(v_layer_name);
  if (!h_layer || !v_layer) {
    openroad->getLogger()->error(utl::CMS, 203, "mesh layer not found");
    return;
  }
  mesh_obj->reserveMeshTsvSites(h_layer, v_layer, pitch, v_strap_pitch,
                                v_strap_offset, v_strap_width, tsv_master,
                                keepout_w, keepout_h, target_spacing);
}

// Sink-side: place sink-taps + assign FFs to nearest tap (capacity-bounded).
// Run after create_clock_mesh + setup_proxy_bterms; before break_bpr_at_tsvs.
void create_sink_taps_cmd(const char* h_layer_name, const char* v_layer_name,
                          const char* tsv_master, const char* sink_buffer,
                          int capacity, int halo)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
  odb::dbDatabase* db = openroad->getDb();
  if (!db) {
    return;
  }
  odb::dbTech* tech = db->getTech();
  if (!tech) {
    return;
  }
  odb::dbTechLayer* h_layer = tech->findLayer(h_layer_name);
  odb::dbTechLayer* v_layer = tech->findLayer(v_layer_name);
  if (!h_layer || !v_layer) {
    openroad->getLogger()->error(utl::CMS, 704, "mesh layer not found");
    return;
  }
  mesh_obj->createSinkTaps(h_layer, v_layer, tsv_master, sink_buffer,
                           capacity, halo);
}

// Break BPR power rails at every front<->back TSV (drive + sink), relocate, and
// drop stranded taps. Run after all TSVs are placed; detailed_placement after.
void break_bpr_at_tsvs_cmd(const char* bpr_layer_name, const char* tsv_master,
                           const char* tap_master, int halo, int relocate_rows)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
  odb::dbDatabase* db = openroad->getDb();
  if (!db) {
    return;
  }
  odb::dbTech* tech = db->getTech();
  if (!tech) {
    return;
  }
  odb::dbTechLayer* bpr = tech->findLayer(bpr_layer_name);
  if (!bpr) {
    openroad->getLogger()->error(utl::CMS, 712, "BPR layer not found");
    return;
  }
  mesh_obj->breakBprAtTsvs(bpr, tsv_master, tap_master, halo, relocate_rows);
}

// Connect sinks via router - places BTerms at grid intersections for router-based connections
// Call AFTER detailed_placement to legalize buffers, and AFTER setup_proxy_bterms for buffer BTerms
void connect_sinks_cmd(const char* clock_name, const char* proxy_layer_name)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }

  ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
  odb::dbDatabase* db = openroad->getDb();
  if (!db) {
    return;
  }

  odb::dbTech* tech = db->getTech();
  if (!tech) {
    return;
  }

  odb::dbTechLayer* proxy_layer = nullptr;
  if (proxy_layer_name && proxy_layer_name[0] != '\0') {
    proxy_layer = tech->findLayer(proxy_layer_name);
    if (!proxy_layer) {
      openroad->getLogger()->error(utl::CMS, 507,
                                  "Proxy layer '{}' not found", proxy_layer_name);
      return;
    }
  } else {
    openroad->getLogger()->error(utl::CMS, 508,
                                "Proxy layer must be specified for sink BTerm placement");
    return;
  }

  mesh_obj->connectSinksViaRouter(clock_name, proxy_layer);
}

// Setup proxy BTERMs at intersections for router-based buffer connections
// This creates BTERMs on the proxy_layer with via stacks down to the mesh
// The router will then connect buffer outputs to these BTERMs
void setup_proxy_bterms_cmd(const char* clock_name, const char* proxy_layer_name)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }

  ord::OpenRoad* openroad = ord::OpenRoad::openRoad();
  odb::dbDatabase* db = openroad->getDb();
  if (!db) {
    return;
  }

  odb::dbTech* tech = db->getTech();
  if (!tech) {
    return;
  }

  odb::dbTechLayer* proxy_layer = nullptr;
  if (proxy_layer_name && proxy_layer_name[0] != '\0') {
    proxy_layer = tech->findLayer(proxy_layer_name);
    if (!proxy_layer) {
      openroad->getLogger()->error(utl::CMS, 610,
                                  "Proxy layer '{}' not found", proxy_layer_name);
      return;
    }
  } else {
    openroad->getLogger()->error(utl::CMS, 611,
                                "Proxy layer must be specified");
    return;
  }

  mesh_obj->setupProxyBTerms(clock_name, proxy_layer);
}

// Connect proxy BTERMs to mesh after routing
// This creates via stacks from the routed BTERM locations down to the mesh grid
void connect_proxy_bterms_to_mesh_cmd(const char* clock_name)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  mesh_obj->connectProxyBTermsToMesh(clock_name);
}

// Author sink_tap special wires at the sink buffers' FINAL positions
// (call after the post-break detailed_placement). use_router=true leaves the
// nets to GRT/DRT instead (no special wire, no SPICE-prep re-author).
void connect_sink_taps_cmd(bool use_router)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  mesh_obj->connectSinkTaps(use_router);
}

// Re-map FFs to nearest sink buffer at FINAL positions (post-placement).
void reassign_sink_ffs_cmd(int capacity)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  mesh_obj->reassignSinkFFs(capacity);
}

// Capture CTS leaf arrival times from STA (call before merge)
void capture_leaf_arrivals_cmd(const char* clock_name)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  mesh_obj->captureLeafArrivals(clock_name);
}

// Merge buffer and sink nets into clk_mesh for parasitic extraction
void merge_nets_to_mesh_cmd(const char* clock_name)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  mesh_obj->mergeNetsToMesh(clock_name);
}

// Convert mesh grid SWires to regular dbWire for OpenRCX extraction
void convert_swire_to_wire_cmd(const char* clock_name)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  mesh_obj->convertSWireToWire(clock_name);
}

// Write SPICE netlist from extracted parasitics
void write_mesh_spice_cmd(const char* clock_name, const char* spice_file,
                          float vdd_voltage, float rise_time, float fall_time,
                          Tcl_Obj* spice_models_obj, bool zero_delay,
                          bool full_tree, bool finfet, float tsv_res)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }

  // Convert TCL list to C++ vector<string>
  std::vector<std::string> spice_models;
  if (spice_models_obj) {
    int list_len = 0;
    Tcl_Obj** list_elements = nullptr;
    if (Tcl_ListObjGetElements(nullptr, spice_models_obj, &list_len, &list_elements) == TCL_OK) {
      for (int i = 0; i < list_len; ++i) {
        const char* path = Tcl_GetString(list_elements[i]);
        if (path && path[0] != '\0') {
          spice_models.push_back(path);
        }
      }
    }
  }

  mesh_obj->writeMeshSpice(clock_name, spice_file,
                           vdd_voltage, rise_time, fall_time, spice_models, zero_delay,
                           full_tree, finfet, tsv_res);
}

// Write mesh-merged Verilog netlist with correct connectivity
// Reads input_file (from write_verilog), modifies net names, writes to output_file
void write_mesh_verilog_cmd(const char* clock_name,
                            const char* input_file,
                            const char* output_file)
{
  cms::ClockMesh* mesh_obj = ord::getClockMesh();
  if (!mesh_obj) {
    return;
  }
  mesh_obj->writeMeshVerilog(clock_name, input_file, output_file);
}

}  // namespace cms

%}
