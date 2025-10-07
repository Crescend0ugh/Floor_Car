add_executable(navmesh_test navmesh_test.cpp ${CMAKE_SOURCE_DIR}/source/navmesh.cpp ${CMAKE_SOURCE_DIR}/source/path.cpp ${CMAKE_SOURCE_DIR}/source/navcontext.cpp)
target_link_libraries(navmesh_test Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo)
target_include_directories(navmesh_test PRIVATE ${CMAKE_SOURCE_DIR}/include)