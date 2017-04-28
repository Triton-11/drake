//
// Created by robin on 27.04.17.
//
#pragma once

namespace hello_namespace {

class Hello {
 public:
  Hello();
  int getter();
  void setter(int v);
 private:
  int internal_variable_;
  int new_var_;
};

} // namespace hello