file(GLOB_RECURSE SOURCES
    source/*.cpp
)
list(FILTER SOURCES EXCLUDE REGEX "main.cpp")

cmake_minimum_required(VERSION 3.28)
project(Floor_Car)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") 
        add_compile_definitions(RPI_UBUNTU)
        message(STATUS "Compiling for Raspberry Pi Ubuntu.")
    endif()
else()
        message(STATUS "Compiling for non-Linux")
endif()

if(WIN32)
    set(OpenCV_DIR "C:/opencv/build")
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

add_subdirectory(third_party)

if(WIN32)
    message(STATUS "Compiling tests.")
    #add_subdirectory(tests)
endif()

add_executable(FloorCar ${SOURCES} source/server/main.cpp)
add_custom_command(TARGET FloorCar POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                    ${CMAKE_SOURCE_DIR}/content/ $<TARGET_FILE_DIR:FloorCar>/content)
                    
target_include_directories(FloorCar PRIVATE include/ third_party/asio/include third_party/zpp_bits)
target_compile_definitions(FloorCar PRIVATE ASIO_STANDALONE _WIN32_WINNT=0x601)
target_link_libraries(FloorCar Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static ${OpenCV_LIBS} ncnn ${PCL_LIBRARIES} serialib)

#####################

if(WIN32)
    message(STATUS "Building client app.")
    add_executable(FloorCarClient ${SOURCES} source/client/main.cpp)
    add_custom_command(TARGET FloorCarClient POST_BUILD
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                        ${CMAKE_SOURCE_DIR}/content/ $<TARGET_FILE_DIR:FloorCarClient>/content)

    target_include_directories(FloorCarClient PRIVATE include/ third_party/asio/include third_party/zpp_bits)
    target_compile_definitions(FloorCarClient PRIVATE ASIO_STANDALONE)
    target_link_libraries(FloorCarClient Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo raylib_static ${OpenCV_LIBS} ncnn ${PCL_LIBRARIES} serialib)
endif()

add_executable(Calibration source/calibration/main.cpp)
target_link_libraries(Calibration ${OpenCV_LIBS})