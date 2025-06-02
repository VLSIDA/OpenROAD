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

  const std::vector<odb::dbNet*> nets;

  const int group_pitch = spacing_ + width_;

  

  int next_minimum_track = std::numeric_limits<int>::lowest();
  for (pos += offset_; pos <= pos_end; pos += pitch_) {
    int group_pos = pos;
    
    
    
    strap_count++;
    if (number_of_straps_ != 0 && strap_count == number_of_straps_) {
      // if number of straps is met, stop adding
      return;
    }
  }
}
}  // namespace cms
