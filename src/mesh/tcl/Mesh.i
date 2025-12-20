%module mesh

%{
#include "mesh/ClockMesh.hh"
#include "ord/OpenRoad.hh"

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

}  // namespace mesh

%}
