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
}  // namespace cms
