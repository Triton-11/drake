add_executable(helloworld helloworld.cc)
add_subdirectory(library)
add_subdirectory(test)
target_link_libraries(helloworld LibraryRobin)
