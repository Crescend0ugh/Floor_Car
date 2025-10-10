set(TEST_SUBDIRECTORIES math_test navmesh_test pcd_test raylib_test networking_test)

foreach(SUBDIRECTORY IN LISTS TEST_SUBDIRECTORIES)
	file(GLOB TEST_SOURCES ${CMAKE_SOURCE_DIR}/source/*.cpp)
	list(FILTER TEST_SOURCES EXCLUDE REGEX "main\\.cpp$")
	list(APPEND TEST_SOURCES ${SUBDIRECTORY}/${SUBDIRECTORY}.cpp)

	add_executable(${SUBDIRECTORY} ${TEST_SOURCES})
	target_include_directories(${SUBDIRECTORY} PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/third_party/asio/include)
	target_compile_definitions(${SUBDIRECTORY} PRIVATE ASIO_STANDALONE _WIN32_WINNT=0x601)
	target_link_libraries(${SUBDIRECTORY} PRIVATE Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static)
endforeach()
 