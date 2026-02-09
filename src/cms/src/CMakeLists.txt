# SPDX-License-Identifier: BSD-3-Clause

include("openroad")

set(CMS_TCL_DIR ${CMAKE_SOURCE_DIR}/src/cms/tcl)

add_library(cms_lib
  ClockMesh.cc
)

target_include_directories(cms_lib
  PUBLIC
    ../include
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/odb/include
    ${CMAKE_SOURCE_DIR}/src/sta
    ${CMAKE_SOURCE_DIR}/src/sta/include
    ${CMAKE_SOURCE_DIR}/src/sta/include/sta
    ${CMAKE_BINARY_DIR}/src/sta/include/sta
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(cms_lib
  PUBLIC
    utl_lib
  PRIVATE
    odb
    dbSta_lib
    OpenSTA
    cts_lib
)

swig_lib(
  NAME cms
  NAMESPACE cms
  I_FILE ${CMS_TCL_DIR}/Cms.i
  SCRIPTS ${CMS_TCL_DIR}/cms.tcl
)

target_sources(cms
  PRIVATE
    MakeClockMesh.cc
)

target_include_directories(cms
  PUBLIC
    ../include
  PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src/odb/include
    ${CMAKE_SOURCE_DIR}/src/sta
    ${CMAKE_SOURCE_DIR}/src/sta/include
    ${CMAKE_SOURCE_DIR}/src/sta/include/sta
    ${CMAKE_BINARY_DIR}/src/sta/include/sta
    ${CMAKE_SOURCE_DIR}/src/dbSta/include
)

target_link_libraries(cms
  PUBLIC
    cms_lib
  PRIVATE
    utl_lib
)
