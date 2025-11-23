// SPDX-License-Identifier: BSD-3-Clause

#include "mesh/ClockMesh.hh"

#include "ord/OpenRoad.hh"
#include "utl/Logger.h"

namespace mesh {

ClockMesh::ClockMesh() = default;

void ClockMesh::init(ord::OpenRoad* openroad)
{
  openroad_ = openroad;
  if (openroad_ != nullptr && openroad_->getLogger() != nullptr) {
    openroad_->getLogger()->report("Minimal ClockMesh initialized.");
  }
}

void ClockMesh::run(const char* name)
{
  if (openroad_ == nullptr || openroad_->getLogger() == nullptr) {
    return;
  }

  const char* mesh_name = (name != nullptr && name[0] != '\0') ? name : "default";
  openroad_->getLogger()->report(
      "Minimal ClockMesh invoked with name: %s", mesh_name);

  mesh_generated_ = true;
}

}  // namespace mesh
