# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2025, The OpenROAD Authors

load("@rules_python//python:defs.bzl", "py_test")
load("//test:regression.bzl", "doc_check_test", "regression_test")

package(features = ["layering_check"])

# From CMakeLists.txt or_integration_tests(TESTS
TESTS = [
    "connect_sinks",
    "create_mesh",
    "create_mesh_adaptive",
    "create_mesh_set_cover",
]

filegroup(
    name = "regression_resources",
    srcs = [
        "16sinks.def",
        "Nangate45/Nangate45.lef",
        "Nangate45/Nangate45.tracks",
        "Nangate45/Nangate45_typ.lib",
        "helpers.tcl",
    ],
)

[
    filegroup(
        name = test_name + "_resources",
        srcs = [":regression_resources"] + glob([
            test_name + ".*",
            test_name + "_out.defok",
        ]),
    )
    for test_name in TESTS
]

[
    regression_test(
        name = test_name,
        data = [":" + test_name + "_resources"],
        tags = [],
        visibility = ["//visibility:public"],
    )
    for test_name in TESTS
]

py_test(
    name = "cms_man_tcl_check",
    srcs = ["cms_man_tcl_check.py"],
    data = ["//src/cms:doc_files"],
    tags = [
        "doc_check",
        "py",
    ],
    deps = ["//docs/src/scripts:extract_utils"],
)

doc_check_test(
    name = "cms_readme_msgs_check",
    data = [
        "//src/cms:doc_files",
        "//src/cms:messages_txt",
    ],
    visibility = ["//visibility:public"],
)
