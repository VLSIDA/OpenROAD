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

  // const auto nets = getNets();

  const int group_pitch = spacing_ + width_;

  // debugPrint(getLogger(),
  //            utl::CMS,
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
}
}  // namespace cms
