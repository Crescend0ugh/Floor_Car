file(GLOB_RECURSE
    CONTENT_FILES
    RELATIVE ${CMAKE_SOURCE_DIR} 
    CONFIGURE_DEPENDS

    ${CMAKE_SOURCE_DIR}/content/*
)

foreach(CONTENT_FILE ${CONTENT_FILES})
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${CONTENT_FILE}
        COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/${CONTENT_FILE}
            ${CMAKE_CURRENT_BINARY_DIR}/${CONTENT_FILE}
        DEPENDS ${CMAKE_SOURCE_DIR}/${CONTENT_FILE}
	)
endforeach()

list(TRANSFORM CONTENT_FILES PREPEND ${CMAKE_CURRENT_BINARY_DIR}/)

set(TEST_SUBDIRECTORIES 
	math_test 
	navmesh_test
	pcd_test 
	raylib_test 
	networking_test
	opencv_test
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
	target_include_directories(${SUBDIRECTORY} PRIVATE 
		${CMAKE_SOURCE_DIR}/include 
		${CMAKE_SOURCE_DIR}/third_party/asio/include 
		${CMAKE_SOURCE_DIR}/third_party/zpp_bits
	)

	target_compile_definitions(${SUBDIRECTORY} PRIVATE ASIO_STANDALONE _WIN32_WINNT=0x601)
	target_link_libraries(${SUBDIRECTORY} PRIVATE Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static ${OpenCV_LIBS} ncnn)
endforeach()
 