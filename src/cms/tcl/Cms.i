%module cms

%{
#include "cms/ClockMesh.hh"
#include "ord/OpenRoad.hh"
#include "db_sta/dbSta.hh"
#include "sta/Corner.hh"
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
                          Tcl_Obj* buffer_list_obj)
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

  // Convert TCL list to C++ vector<string>
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

  // Call the main mesh grid creation function (wire width auto-computed from tech)
  mesh_obj->createMeshGrid(clock_name, h_layer, v_layer, pitch, buffer_list);
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
