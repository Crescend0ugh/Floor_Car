add_executable(pcd_test pcd_test.cpp)
target_include_directories(pcd_test PRIVATE ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(pcd_test raylib_static)