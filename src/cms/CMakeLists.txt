# Copyright (c) 2021, The Regents of the University of California
# All rights reserved.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

include("openroad")

project(cms)

swig_lib(NAME      cms
         NAMESPACE cms
         I_FILE    src/ClockMesh.i
         SCRIPTS   src/ClockMesh.tcl
)

target_sources(cms
  PRIVATE
  src/MakeClockMesh.cc
  src/ClockMesh.cc
)

target_include_directories(cms
  PUBLIC
    include
)

target_link_libraries(cms
    OpenSTA
    odb
)

messages(
  TARGET cms
)
