// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#include "straps.h"

#include <algorithm>
#include <boost/geometry.hpp>
#include <boost/polygon/polygon.hpp>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace cms {

Straps::Straps(odb::dbTechLayer* layer,
               int width,
               int pitch,
               int spacing,
               int number_of_straps)
    : layer_(layer),
      width_(width),
      spacing_(spacing),
      pitch_(pitch),
      number_of_straps_(number_of_straps)
{
  
}

Straps::Straps(int width,
               int pitch,
               int spacing,
               int number_of_straps)
    : width_(width),
      spacing_(spacing),
      pitch_(pitch),
      number_of_straps_(number_of_straps)
{
  
}

// void Straps::makeStraps(int x_start,
//                         int y_start,
//                         int x_end,
//                         int y_end,
//                         int abs_start,
//                         int abs_end,
//                         bool is_delta_x,
//                         const TechLayer& layer,
//                         const Shape::ObstructionTree& avoid)
// {

// }

void Straps::makeStraps(int x_start,
                        int y_start,
                        int x_end,
                        int y_end,
                        int abs_start,
                        int abs_end,
                        bool is_delta_x)
{
  const int half_width = width_ / 2;
  int strap_count = 0;

  int pos = is_delta_x ? x_start : y_start;
  const int pos_end = is_delta_x ? x_end : y_end;

  std::set<odb::dbNet*> nets_;
  // TODO: getting nets logic



  const std::vector<odb::dbNet*> nets(nets_.begin(), nets_.end());
  const int group_pitch = spacing_ + width_;

  // debugPrint(getLogger(),
  //            utl::PDN,
  //            "Straps",
  //            2,
  //            "Generating straps on {} from ({:.4f}, {:.4f}) to ({:.4f}, "
  //            "{:.4f}) with an {}-offset of {:.4f} and must be within {:.4f} "
  //            "and {:.4f}",
  //            layer_->getName(),
  //            layer.dbuToMicron(x_start),
  //            layer.dbuToMicron(y_start),
  //            layer.dbuToMicron(x_end),
  //            layer.dbuToMicron(y_end),
  //            is_delta_x ? "x" : "y",
  //            layer.dbuToMicron(offset_),
  //            layer.dbuToMicron(abs_start),
  //            layer.dbuToMicron(abs_end));
  

  int next_minimum_track = std::numeric_limits<int>::lowest();
  for (pos += offset_; pos <= pos_end; pos += pitch_) {
    int group_pos = pos;
    
    for (auto* net : nets) {
      // snap to grid if needed
      const int org_group_pos = group_pos;
      // group_pos = layer.snapToGrid(org_group_pos, next_minimum_track);
      const int strap_start = group_pos - half_width;
      const int strap_end = strap_start + width_;
      // debugPrint(getLogger(),
      //            utl::PDN,
      //            "Straps",
      //            3,
      //            "Snapped from {:.4f} -> {:.4f} resulting in strap from {:.4f} "
      //            "to {:.4f}",
      //            layer.dbuToMicron(org_group_pos),
      //            layer.dbuToMicron(group_pos),
      //            layer.dbuToMicron(strap_start),
      //            layer.dbuToMicron(strap_end));

      if (strap_start >= pos_end) {
        // no portion of the strap is inside the limit
        return;
      }
      if (group_pos > pos_end) {
        // strap center is outside of alotted area
        return;
      }

      odb::Rect strap_rect;
      if (is_delta_x) {
        strap_rect = odb::Rect(strap_start, y_start, strap_end, y_end);
      } else {
        strap_rect = odb::Rect(x_start, strap_start, x_end, strap_end);
      }
      group_pos += group_pitch;
      next_minimum_track = group_pos;

      // if (avoid.qbegin(bgi::intersects(strap_rect)) != avoid.qend()) {
      //   // dont add this strap as it intersects an avoidance
      //   continue;
      // }

      if (is_delta_x) {
        if (strap_rect.xMin() < abs_start || strap_rect.xMax() > abs_end) {
          continue;
        }
      } else {
        if (strap_rect.yMin() < abs_start || strap_rect.yMax() > abs_end) {
          continue;
        }
      }

      // addShape(
      //     new Shape(layer_, net, strap_rect, odb::dbWireShapeType::STRIPE));
    }
    strap_count++;
    if (number_of_straps_ != 0 && strap_count == number_of_straps_) {
      // if number of straps is met, stop adding
      return;
    }
  }
}

int Straps::getNumberOfStraps() {
  return number_of_straps_;
}
}  // namespace cms
