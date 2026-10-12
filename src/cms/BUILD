# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, The OpenROAD Authors

load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("//bazel:tcl_encode_or.bzl", "tcl_encode")
load("//bazel:tcl_wrap_cc.bzl", "tcl_wrap_cc")
load("//test:regression.bzl", "messages_txt")

package(
    default_visibility = ["//:__subpackages__"],
    features = ["layering_check"],
)

cc_library(
    name = "cms",
    srcs = [
        "src/ClockMesh.cc",
        "src/LoadAdaptiveBuffer.cc",
    ],
    hdrs = [
        "include/cms/ClockMesh.hh",
        "include/cms/LoadAdaptiveBuffer.hh",
    ],
    includes = [
        "include",
    ],
    deps = [
        "//src/cts",
        "//src/dbSta",
        "//src/dbSta:dbNetwork",
        "//src/odb/src/db",
        "//src/sta:opensta_lib",
        "//src/utl",
    ],
)

cc_library(
    name = "ui",
    srcs = [
        "src/MakeClockMesh.cc",
        ":swig",
        ":tcl",
    ],
    hdrs = [
        "include/cms/MakeClockMesh.h",
    ],
    includes = [
        "include",
    ],
    deps = [
        ":cms",
        "//:ord",
        "//src/dbSta",
        "//src/odb/src/db",
        "//src/sta:opensta_lib",
        "//src/utl",
        "@boost.stacktrace",
        "@tcl_lang//:tcl",
    ],
)

tcl_encode(
    name = "tcl",
    srcs = [
        "src/cms.tcl",
    ],
    char_array_name = "cms_tcl_inits",
    namespace = "cms",
)

tcl_wrap_cc(
    name = "swig",
    srcs = [
        "src/Cms.i",
        "//:error_swig",
    ],
    module = "cms",
    namespace_prefix = "cms",
    root_swig_src = "src/Cms.i",
    swig_includes = [
        "src/cms/src",
    ],
)

filegroup(
    name = "doc_files",
    srcs = [
        "README.md",
        "src/cms.tcl",
    ],
    visibility = ["//src/cms/test:__pkg__"],
)

messages_txt(
    visibility = [
        "//docs:__pkg__",
        "//src/cms/test:__pkg__",
    ],
)
