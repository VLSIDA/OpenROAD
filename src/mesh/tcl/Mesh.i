%module mesh

%{
#include "mesh/ClockMesh.hh"
#include "ord/OpenRoad.hh"
#include <tcl.h>

namespace ord {
extern mesh::ClockMesh* getClockMesh();
}
%}

%include "../../Exception.i"

%inline %{

namespace mesh {

// Main command - runs sink finding and test print (same as CTS structure)
void run_mesh_cmd(const char* name)
{
  mesh::ClockMesh* mesh = ord::getClockMesh();
  if (mesh) {
    mesh->run(name);
  }
}

// Create mesh grid command - creates clock mesh with horizontal and vertical wires
void create_mesh_grid_cmd(const char* clock_name,
                          const char* h_layer_name,
                          const char* v_layer_name,
                          int wire_width,
                          int pitch,
                          Tcl_Obj* buffer_list_obj)
{
  mesh::ClockMesh* mesh_obj = ord::getClockMesh();
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
      openroad->getLogger()->error(utl::MESH, 200,
                                  "Horizontal layer '{}' not found", h_layer_name);
      return;
    }
  }

  // Find vertical layer
  odb::dbTechLayer* v_layer = nullptr;
  if (v_layer_name && v_layer_name[0] != '\0') {
    v_layer = tech->findLayer(v_layer_name);
    if (!v_layer) {
      openroad->getLogger()->error(utl::MESH, 201,
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

  // Call the main mesh grid creation function with buffer list
  mesh_obj->createMeshGrid(clock_name, h_layer, v_layer, wire_width, pitch, buffer_list);
}

}  // namespace mesh

%}
