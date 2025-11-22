cmake_minimum_required(VERSION 3.28)
project(Floor_Car)

set(CMAKE_CXX_STANDARD 20)

if(WIN32)
    set(OpenCV_DIR "C:/opencv/build")
endif()

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(SDL2 REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_library(GPIOD_LIBRARY gpiod)
endif()

#for Adithya
if(NOT TARGET JsonCpp::JsonCpp)
    # Attempt to find JsonCpp library - modify paths as needed
    find_library(JSONCPP_LIBRARY NAMES jsoncpp PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/lib /usr/local/lib)
    find_path(JSONCPP_INCLUDE_DIR NAMES json/json.h PATHS /opt/homebrew/Cellar/jsoncpp/1.9.5/include /usr/local/include)

    # Create an imported target
    add_library(JsonCpp::JsonCpp SHARED IMPORTED)
    set_target_properties(JsonCpp::JsonCpp PROPERTIES
            IMPORTED_LOCATION "${JSONCPP_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${JSONCPP_INCLUDE_DIR}")
endif()

# Copy the "content" directory into binaries' output folder so files can access using "../content/"
# This is required for some of the tests, so it should come before the tests add_subdirectory call
macro(ADD_CONTENT_POST_BUILD target_name)
    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/content/"
            "$<TARGET_FILE_DIR:${target_name}>/content"
        COMMENT "Copying content directory to ${target_name} output directory"
    )
endmacro()

add_subdirectory(third_party)

if(WIN32 OR APPLE)
    message(STATUS "Compiling tests.")
    add_subdirectory(tests)
endif()

########################################################################
# Server static library and executable target
########################################################################

set(SERVER_SOURCES
    source/arduino_serial.cpp
    source/controller.cpp
    source/ekf.cpp
    source/mapgen.cpp
    source/navgeometry.cpp
    source/navmesh.cpp
    source/network.cpp
    source/object_tracker.cpp
    source/path.cpp
    source/vision.cpp
    source/voice_detection.cpp
    source/yolo_model.cpp
)

add_library(FloorCar_lib STATIC ${SERVER_SOURCES})

target_include_directories(FloorCar_lib
    PUBLIC
        include/
        third_party/asio/include
        third_party/zpp_bits
        third_party/kalman
)

target_compile_definitions(FloorCar_lib
    PUBLIC
        ASIO_STANDALONE
)

target_link_libraries(FloorCar_lib
    PUBLIC
        Recast
        Detour
        DetourCrowd
        DetourTileCache
        DebugUtils
        RecastDemo
        raylib_static
        ${OpenCV_LIBS}
        ncnn
        ${PCL_LIBRARIES}
        serialib
        serial_transfer
        whisper-common
        whisper-common-sdl
        whisper
        ${CMAKE_THREAD_LIBS_INIT}
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	target_link_libraries(FloorCar_lib PUBLIC gpiod)
endif()

add_executable(FloorCar ${SERVER_SOURCES} source/server/main.cpp)
ADD_CONTENT_POST_BUILD(FloorCar)
target_compile_definitions(FloorCar PRIVATE _WIN32_WINNT=0x601)
target_link_libraries(FloorCar PRIVATE FloorCar_lib)

########################################################################
# Client static library and executable target
########################################################################

if(WIN32 OR APPLE)
    message(STATUS "Building client app.")
    set(CLIENT_SOURCES
        source/network.cpp
        source/client/camera_feed.cpp
        source/client/connection_screen.cpp
        source/client/point_cloud_visualizer.cpp
    )

    add_library(FloorCarClient_lib STATIC ${CLIENT_SOURCES})

    # Client static library
    target_include_directories(FloorCarClient_lib
        PUBLIC
            include/
            third_party/asio/include
            third_party/zpp_bits
    )

    target_compile_definitions(FloorCarClient_lib
        PUBLIC
            ASIO_STANDALONE
    )

    target_link_libraries(FloorCarClient_lib
        PUBLIC
            raylib_static
            ${OpenCV_LIBS}
    )

    add_executable(FloorCarClient ${CLIENT_SOURCES} source/client/main.cpp)
    target_compile_definitions(FloorCarClient PRIVATE _WIN32_WINNT=0x601)
    target_link_libraries(FloorCarClient FloorCarClient_lib)
endif()

########################################################################
# Camera calibration executable
########################################################################

add_executable(Calibration source/calibration/main.cpp)
target_link_libraries(Calibration PRIVATE ${OpenCV_LIBS})

########################################################################
# IP display on LCD
########################################################################

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(IpDisplay source/ip_display/main.cpp)
    target_link_libraries(IpDisplay PRIVATE gpiod)
endif()

message(STATUS "Finished building project.")