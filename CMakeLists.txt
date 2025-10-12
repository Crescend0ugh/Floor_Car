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

file(GLOB SOURCES
    source/*.cpp
)

cmake_minimum_required(VERSION 3.28)
project(Floor_Car)

set(CMAKE_CXX_STANDARD 20)

if(WIN32)
    set(OpenCV_DIR "C:/opencv/build")
endif()

find_package(OpenCV REQUIRED)

add_subdirectory(third_party)
add_subdirectory(tests)

add_executable(FloorCar ${SOURCES} source/server/main.cpp)
target_include_directories(FloorCar PRIVATE include/ third_party/asio/include third_party/zpp_bits)
target_compile_definitions(FloorCar PRIVATE ASIO_STANDALONE _WIN32_WINNT=0x601)
target_link_libraries(FloorCar Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static)

add_executable(FloorCarClient ${SOURCES} source/client/main.cpp)
target_include_directories(FloorCarClient PRIVATE include/ third_party/asio/include third_party/zpp_bits)
target_compile_definitions(FloorCarClient PRIVATE ASIO_STANDALONE)
target_link_libraries(FloorCarClient Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static raylib_static)
