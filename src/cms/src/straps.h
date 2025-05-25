// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#pragma once

#include <array>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"

namespace cms {

class Straps
{
 public:
  Straps(odb::dbTechLayer* layer,
         int width,
         int pitch,
         int spacing = 0,
         int number_of_straps = 0);

 private:
  odb::dbTechLayer* layer_;
  int width_;
  int spacing_;
  int pitch_;
  int offset_ = 0;
  int number_of_straps_;
};

}  // namespace cms
