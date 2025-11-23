// SPDX-License-Identifier: BSD-3-Clause

#pragma once

namespace ord {
class OpenRoad;
}

namespace mesh {

class ClockMesh
{
 public:
  ClockMesh();

  void init(ord::OpenRoad* openroad);
  void run(const char* name);
  bool meshGenerated() const { return mesh_generated_; }

 private:
  ord::OpenRoad* openroad_ = nullptr;
  bool mesh_generated_ = false;
};

void initClockMesh(ord::OpenRoad* openroad);

}  // namespace mesh
