# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "world_sim_diagram_factory_quad",
    srcs = [
        "world_sim_diagram_factory_quad.cc",
    ],
    hdrs = [
        "world_sim_diagram_factory_quad.h",
    ],
    visibility = ["//drake/examples:__subpackages__"],
    deps = [
        "//drake/common:drake_path",
        "//drake/common/trajectories:piecewise_polynomial_trajectory",
        "//drake/lcm",
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/systems/analysis:simulator",
        "//drake/systems/controllers:inverse_dynamics_controller",
        "//drake/systems/framework",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:demultiplexer",
        "//drake/systems/primitives:multiplexer",
        "//drake/systems/primitives:trajectory_source",
    ],
)

drake_cc_binary(
    name = "quadrotor_demo",
    srcs = ["quadrotor_demo.cc"],
    add_test_rule = 1,
    data = [
        "//drake/examples/Quadrotor:models",
    ],
    test_rule_args = ["--simulation_sec=0.1"],
    deps = [
        "@gflags//:gflags",
        ":world_sim_diagram_factory_quad",
        ":world_sim_tree_builder_quad",
        "//drake/common",
    ],
)

drake_cc_library(
    name = "world_sim_tree_builder_quad",
    srcs = ["world_sim_tree_builder_quad.cc"],
    hdrs = ["world_sim_tree_builder_quad.h"],
    visibility = ["//drake/examples:__subpackages__"],
    deps = [
        "//drake/multibody:rigid_body_tree",
        "//drake/multibody:rigid_body_tree_construction",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
    ],
)

cpplint()
