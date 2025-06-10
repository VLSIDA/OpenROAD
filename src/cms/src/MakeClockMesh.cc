// Copyright (c) 2021, The Regents of the University of California
// All rights reserved.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "ord/OpenRoad.hh"
#include "cms/ClockMesh.hh"
#include "cms/MakeClockMesh.hh"
#include "utl/decode.h"

namespace cms {
  extern const char* cms_tcl_inits[];
  
  extern "C" {
  extern int Cms_Init(Tcl_Interp* interp);
  }
  }  // namespace cms
  
namespace ord {

cms::ClockMesh *
makeClockMesh()
{
  return new cms::ClockMesh;
}

void
deleteClockMesh(cms::ClockMesh *mesh)
{
  delete mesh;
}

void
initClockMesh(OpenRoad *openroad)
{
  Tcl_Interp* interp = openroad->tclInterp();
  // Define swig TCL commands.
  cms::Cms_Init(interp);
  // Eval encoded sta TCL sources.
  utl::evalTclInit(interp, cms::cms_tcl_inits);


  openroad->getCMS()->init(interp,
			    openroad->getDb(),
          openroad->getDbNetwork(),
          openroad->getLogger());
}

}
