#include <iostream>
#include <typeinfo>

#ifndef HELLO_TYPE_TEMPLATE_METHODS_H
#define HELLO_TYPE_TEMPLATE_METHODS_H

#endif //HELLO_TYPE_TEMPLATE_METHODS_H

using namespace std;

// General template for a function printing the data type
template <typename T> void get_type(T input) {
  cout << "Hello there, I am generic for not specially defined data types.\n";
  cout << "The type of your data is: " << typeid(input).name() << endl;
}
