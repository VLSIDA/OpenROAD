# SPDX-License-Identifier: BSD-3-Clause

include("openroad")

set(MESH_TCL_DIR ${CMAKE_SOURCE_DIR}/src/mesh/tcl)

add_library(mesh_lib
  ClockMesh.cc
)

target_include_directories(mesh_lib
  PUBLIC
    ../include
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/odb/include
)

target_link_libraries(mesh_lib
  PUBLIC
    utl_lib
  PRIVATE
    odb
    dbSta_lib
    OpenSTA
)

swig_lib(
  NAME mesh
  NAMESPACE mesh
  I_FILE ${MESH_TCL_DIR}/Mesh.i
  SCRIPTS ${MESH_TCL_DIR}/Mesh.tcl
)

target_sources(mesh
  PRIVATE
    MakeClockMesh.cc
)

target_include_directories(mesh
  PUBLIC
    ../include
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/odb/include
)

target_link_libraries(mesh
  PUBLIC
    mesh_lib
  PRIVATE
    utl_lib
)
