// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <functional>
#include <memory>
#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "odb/geom_boost.h"
// #include "odb/dbBlock.h"

namespace cms {

namespace bgi = boost::geometry::index;

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

  int getNumberOfStraps();

 private:
  odb::dbTechLayer* layer_;
  int width_;
  int spacing_;
  int pitch_;
  int offset_ = 0;
  int number_of_straps_;
};

}  // namespace cms
