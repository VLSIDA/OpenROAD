// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, The OpenROAD Authors

#include "cms/MakeClockMesh.h"

#include "tcl.h"
#include "utl/decode.h"

extern "C" {
extern int Cms_Init(Tcl_Interp* interp);
}

namespace cms {

extern const char* cms_tcl_inits[];

void initClockMesh(Tcl_Interp* tcl_interp)
{
  // Define swig TCL commands.
  Cms_Init(tcl_interp);
  utl::evalTclInit(tcl_interp, cms::cms_tcl_inits);
}

}  // namespace cms
