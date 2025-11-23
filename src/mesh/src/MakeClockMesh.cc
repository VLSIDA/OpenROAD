// SPDX-License-Identifier: BSD-3-Clause

#include <tcl.h>

#include "ord/OpenRoad.hh"
#include "utl/decode.h"

extern "C" {
extern int Mesh_Init(Tcl_Interp* interp);
}

namespace mesh {

extern const char* mesh_tcl_inits[];

void initClockMesh(ord::OpenRoad* openroad)
{
  if (openroad == nullptr) {
    return;
  }

  Tcl_Interp* tcl_interp = openroad->tclInterp();
  Mesh_Init(tcl_interp);
  utl::evalTclInit(tcl_interp, mesh::mesh_tcl_inits);

  // Minimal implementation has no mesh object to initialize.
}

}  // namespace mesh
