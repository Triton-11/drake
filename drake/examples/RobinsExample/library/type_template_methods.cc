#include <string>

#include "type_template_methods.h"

// Specialization for T=int
template <> void get_type<int>(int input) {
  cout << "Hello there, I can operate with int values\n";
}

// Specialization for T=float
template <> void get_type<float>(float input) {
  cout << "Hello there, I can juggle with floats\n";
}

// Specialization for T=double
template <> void get_type<double>(double input) {
  cout << "Hello there, I work with doubles\n";
}

// Specialization for T=char
template <> void get_type<char>(char input) {
  cout << "Hello there, I am used to work with chars\n";
}

// Specialization for T=string
template <> void get_type<string>(string input) {
  cout << "Hello there, I am all about strings\n";
}

// Specialization for T=bool
template <> void get_type<bool>(bool input) {
  cout << "Hello there, I am the one for bools\n";
}
