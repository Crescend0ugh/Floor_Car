file(GLOB TEST_SOURCES ${CMAKE_SOURCE_DIR}/source/*.cpp)
list(FILTER TEST_SOURCES EXCLUDE REGEX "main\\.cpp$")
list(APPEND TEST_SOURCES "navmesh_test.cpp")

add_executable(navmesh_test ${TEST_SOURCES})
target_link_libraries(navmesh_test Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo)
target_include_directories(navmesh_test PRIVATE ${CMAKE_SOURCE_DIR}/include)