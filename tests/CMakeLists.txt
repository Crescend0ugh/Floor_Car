set (TEST_SUBDIRECTORIES math_test navmesh_test pcd_test raylib_test)

foreach(SUBDIRECTORY IN LISTS TEST_SUBDIRECTORIES)
	file(GLOB TEST_SOURCES ${CMAKE_SOURCE_DIR}/source/*.cpp)
	list(FILTER TEST_SOURCES EXCLUDE REGEX "main\\.cpp$")
	list(APPEND TEST_SOURCES ${SUBDIRECTORY}/${SUBDIRECTORY}.cpp)

	add_executable(${SUBDIRECTORY} ${TEST_SOURCES})
	target_include_directories(${SUBDIRECTORY} PRIVATE ${CMAKE_SOURCE_DIR}/include)
	target_link_libraries(${SUBDIRECTORY} Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static)
	
endforeach()
