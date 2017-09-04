# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

filegroup(
    name = "models",
    srcs = glob([
        "**/*.obj",
        "**/*.sdf",
        "**/*.urdf",
        "**/*.xml",
    ]),
    visibility = ["//visibility:public"],
)

drake_cc_library(
    name = "ilqr",
    srcs = ["ilqr.cc"],
    hdrs = ["ilqr.h"],
    deps = [
        "//drake/examples/Quadrotor/ilqr:matrix_seq",
        "//drake/examples/Quadrotor/ilqr:quad_model",
        "//drake/examples/Quadrotor/ilqr:trajectory",
        "@eigen",
    ],
)

drake_cc_library(
    name = "quadrotor_lcm_msg_handler",
    srcs = [],
    hdrs = ["quadrotor_lcm_msg_handler.h"],
    deps = [
        "//drake/lcm",
        "//drake/lcmtypes:quadrotor",
    ],
)

drake_cc_library(
    name = "quad_model",
    srcs = ["quad_model.cc"],
    hdrs = ["quad_model.h"],
    deps = [
        "@eigen",
    ],
)

drake_cc_library(
    name = "trajectory",
    srcs = ["trajectory.cc"],
    hdrs = ["trajectory.h"],
    deps = [
        "@eigen",
    ],
)

drake_cc_library(
    name = "matrix_seq",
    srcs = ["matrix_seq.cc"],
    hdrs = ["matrix_seq.h"],
    deps = [
        "@eigen",
    ],
)

drake_cc_library(
    name = "quad_model_bodyrates",
    srcs = ["quad_model_bodyrates.cc"],
    hdrs = ["quad_model_bodyrates.h"],
    deps = [
        "//drake/examples/Quadrotor/ilqr:quad_model",
        "@eigen",
    ],
)

drake_cc_library(
    name = "quad_model_drake",
    srcs = ["quad_model_drake.cc"],
    hdrs = ["quad_model_drake.h"],
    data = [
        ":models",
        "//drake/examples/Quadrotor:models",
    ],
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/Quadrotor:state_conversion",
        "//drake/examples/Quadrotor/ilqr:quad_model",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:rigid_body_plant_autodiff",
        "//drake/systems/framework:diagram",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

drake_cc_library(
    name = "tilqr",
    srcs = [
        "tilqr.cc",
    ],
    hdrs = [
        "tilqr.h",
    ],
    deps = [
        "//drake/examples/Quadrotor/ilqr:quad_model",
        "//drake/examples/Quadrotor/ilqr:trajectory",
        "@eigen",
    ],
)

drake_cc_binary(
    name = "controller",
    srcs = ["controller.cc"],
    add_test_rule = 1,
    data = [":models"],
    test_rule_args = ["--duration=1.0"],
    test_rule_size = "medium",
    deps = [
        "//drake/examples/Quadrotor/ilqr",
        "//drake/examples/Quadrotor/ilqr:quad_model_bodyrates",
        "//drake/examples/Quadrotor/ilqr:tilqr",
        "//drake/examples/Quadrotor/ilqr:trajectory",
        "@com_github_gflags_gflags//:gflags",
    ],
)

drake_cc_binary(
    name = "run_controller",
    srcs = ["run_controller.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/examples/Quadrotor:models",
    ],
    test_rule_args = ["--duration=1.0"],
    test_rule_size = "medium",
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/Quadrotor:state_conversion",
        "//drake/examples/Quadrotor/ilqr",
        "//drake/examples/Quadrotor/ilqr:quad_model",
        "//drake/examples/Quadrotor/ilqr:quad_model_bodyrates",
        "//drake/examples/Quadrotor/ilqr:quad_model_drake",
        "//drake/examples/Quadrotor/ilqr:quadrotor_lcm_msg_handler",
        "//drake/examples/Quadrotor/ilqr:tilqr",
        "//drake/examples/Quadrotor/ilqr:trajectory",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:rigid_body_plant_autodiff",
        "//drake/systems/framework:diagram",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

drake_cc_binary(
    name = "test_controller",
    srcs = ["test_controller.cc"],
    add_test_rule = 1,
    data = [
        ":models",
        "//drake/examples/Quadrotor:models",
    ],
    test_rule_args = ["--duration=1.0"],
    test_rule_size = "medium",
    deps = [
        "//drake/common:find_resource",
        "//drake/examples/Quadrotor:state_conversion",
        "//drake/examples/Quadrotor/ilqr",
        "//drake/examples/Quadrotor/ilqr:quad_model",
        "//drake/examples/Quadrotor/ilqr:quad_model_bodyrates",
        "//drake/examples/Quadrotor/ilqr:quad_model_drake",
        "//drake/examples/Quadrotor/ilqr:quadrotor_lcm_msg_handler",
        "//drake/examples/Quadrotor/ilqr:tilqr",
        "//drake/examples/Quadrotor/ilqr:trajectory",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:rigid_body_plant_autodiff",
        "//drake/systems/framework:diagram",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cpplint()
