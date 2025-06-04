// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"
#include "odb/dbTypes.h"
#include "odb/geom.h"
#include "odb/geom_boost.h"

namespace cms {

namespace bgi = boost::geometry::index;

class Shape
{
  public:
    Shape(){
      obs_ = odb::Rect(0, 0, 0, 0);
    };
    
    const odb::Rect& getObstruction() const { return obs_; }
  private:
    odb::Rect obs_;
};

class Shape;

using ShapePtr = std::shared_ptr<Shape>;

struct ObstructionIndexableGetter
{
  using result_type = odb::Rect;
  odb::Rect operator()(const ShapePtr& t) const
  {
    return t->getObstruction();
  }
};

using ObstructionTree = bgi::
    rtree<ShapePtr, bgi::quadratic<16>, ObstructionIndexableGetter>;

using ObstructionTreeMap = std::map<odb::dbTechLayer*, ObstructionTree>;


} // namespace cms