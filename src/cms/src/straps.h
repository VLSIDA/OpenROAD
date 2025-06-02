// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#pragma once

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"
// #include "odb/dbBlock.h"

namespace cms {

class Straps
{
 public:
  Straps(odb::dbTechLayer* layer,
         int width,
         int pitch,
         int spacing = 0,
         int number_of_straps = 0);

  Straps(int width,
         int pitch,
         int spacing = 0,
         int number_of_straps = 0);
  
  // void makeStraps(int x_start,
  //                 int y_start,
  //                 int x_end,
  //                 int y_end,
  //                 int abs_start,
  //                 int abs_end,
  //                 bool is_delta_x,
  //                 const TechLayer& layer,
  //                 const Shape::ObstructionTree& avoid);
  
  void makeStraps(int x_start,
                  int y_start,
                  int x_end,
                  int y_end,
                  int abs_start,
                  int abs_end,
                  bool is_delta_x);

 private:
  odb::dbTechLayer* layer_;
  int width_;
  int spacing_;
  int pitch_;
  int offset_ = 0;
  int number_of_straps_;
};

}  // namespace cms
