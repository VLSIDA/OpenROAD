// SPDX-License-Identifier: BSD-3-Clause

#include <tcl.h>

#include "cms/ClockMesh.hh"
#include "ord/OpenRoad.hh"
#include "utl/decode.h"

extern "C" {
extern int Cms_Init(Tcl_Interp* interp);
}

namespace ord {
// Define the global mesh object
static cms::ClockMesh* mesh_obj = nullptr;

cms::ClockMesh* getClockMesh()
{
  return mesh_obj;
}

}  // namespace ord

namespace cms {

extern const char* cms_tcl_inits[];

void initClockMesh(ord::OpenRoad* openroad)
{
  if (openroad == nullptr) {
    return;
  }

  Tcl_Interp* tcl_interp = openroad->tclInterp();
  Cms_Init(tcl_interp);
  utl::evalTclInit(tcl_interp, cms::cms_tcl_inits);

  // Create and initialize the mesh object
  ord::mesh_obj = new ClockMesh();
  ord::mesh_obj->init(openroad);
}

void deleteClockMesh()
{
  delete ord::mesh_obj;
  ord::mesh_obj = nullptr;
}

}  // namespace cms
