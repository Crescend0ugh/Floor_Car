add_executable(raylib_test raylib_test.cpp)
target_include_directories(raylib_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(raylib_test raylib_static)