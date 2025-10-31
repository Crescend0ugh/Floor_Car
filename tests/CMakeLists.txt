set(TEST_SUBDIRECTORIES 
	math_test 
	navmesh_test
	pcd_test 
	raylib_test 
	networking_test
	opencv_test
	#controller_test
	bounding_boxes_test
)

if(WIN32)
	set(OpenCV_DIR "C:/opencv/build")
endif()

find_package(OpenCV REQUIRED)

foreach(SUBDIRECTORY IN LISTS TEST_SUBDIRECTORIES)
	file(GLOB TEST_SOURCES ${CMAKE_SOURCE_DIR}/source/*.cpp)
	list(FILTER TEST_SOURCES EXCLUDE REGEX "main\\.cpp$")
	list(APPEND TEST_SOURCES ${SUBDIRECTORY}/${SUBDIRECTORY}.cpp)

	add_executable(${SUBDIRECTORY} ${TEST_SOURCES})
	
	add_custom_command(TARGET ${SUBDIRECTORY} POST_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                       ${CMAKE_SOURCE_DIR}/content/ $<TARGET_FILE_DIR:${SUBDIRECTORY}>/content) 

	target_include_directories(${SUBDIRECTORY} PRIVATE 
		${CMAKE_SOURCE_DIR}/include 
		${CMAKE_SOURCE_DIR}/third_party/asio/include 
		${CMAKE_SOURCE_DIR}/third_party/zpp_bits
	)

	target_compile_definitions(${SUBDIRECTORY} PRIVATE ASIO_STANDALONE _WIN32_WINNT=0x601)
	target_link_libraries(${SUBDIRECTORY} PRIVATE Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static ${OpenCV_LIBS} ncnn ${PCL_LIBRARIES} serialib)
endforeach()
 