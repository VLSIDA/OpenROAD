# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, The OpenROAD Authors

load("//bazel:tcl_encode_or.bzl", "tcl_encode")
load("//bazel:tcl_wrap_cc.bzl", "tcl_wrap_cc")

package(
    default_visibility = ["//:__subpackages__"],
    features = ["layering_check"],
)

cc_library(
    name = "pdn",
    srcs = [
        "src/MakePdnGen.cc",
        "src/PdnGen.cc",
        "src/connect.cpp",
        "src/connect.h",
        "src/domain.cpp",
        "src/domain.h",
        "src/grid.cpp",
        "src/grid.h",
        "src/grid_component.cpp",
        "src/grid_component.h",
        "src/power_cells.cpp",
        "src/power_cells.h",
        "src/renderer.cpp",
        "src/renderer.h",
        "src/rings.cpp",
        "src/rings.h",
        "src/shape.cpp",
        "src/shape.h",
        "src/sroute.cpp",
        "src/sroute.h",
        "src/straps.cpp",
        "src/straps.h",
        "src/techlayer.cpp",
        "src/techlayer.h",
        "src/via.cpp",
        "src/via.h",
        "src/via_repair.cpp",
        "src/via_repair.h",
        ":swig",
        ":tcl",
    ],
    hdrs = [
        "include/pdn/MakePdnGen.hh",
        "include/pdn/PdnGen.hh",
    ],
    copts = [
        "-Wno-missing-braces",  # from TCL swigging
    ],
    includes = [
        "include",
    ],
    deps = [
        "//:ord",
        "//src/gui",
        "//src/odb",
        "//src/utl",
        "@boost.geometry",
        "@boost.polygon",
        "@boost.stacktrace",
        "@tk_tcl//:tcl",
    ],
)

tcl_encode(
    name = "tcl",
    srcs = [
        "src/pdn.tcl",
    ],
    char_array_name = "pdn_tcl_inits",
    namespace = "pdn",
)

tcl_wrap_cc(
    name = "swig",
    srcs = [
        "src/PdnGen.i",
        "//:error_swig",
    ],
    module = "pdn",
    namespace_prefix = "pdn",
    root_swig_src = "src/PdnGen.i",
    swig_includes = [
        "src/pdn/src",
    ],
    deps = [
        "//src/odb:swig",
    ],
)
