# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_library(
    name = "helloworld",
    srcs = [
        "helloworld.cc",
    ],
    hdrs = [
        "helloworld.h",
    ],
    visibility = ["//drake/examples:__subpackages__"],
    deps = [
    ],
)

drake_cc_binary(
    name = "run_hello_demo",
    srcs = ["run_hello_demo.cc"],
    deps = [
        "@gflags//:gflags",
        ":helloworld",
        "//drake/common",
    ],
)

cpplint()
