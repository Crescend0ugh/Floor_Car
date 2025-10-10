file(GLOB SOURCES
    source/*.cpp
)

cmake_minimum_required(VERSION 3.28)
project(Floor_Car)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(third_party)
add_subdirectory(tests)

add_executable(FloorCar ${SOURCES})
target_include_directories(FloorCar PRIVATE include/ third_party/asio/include)
target_compile_definitions(FloorCar PRIVATE ASIO_STANDALONE)
target_link_libraries(FloorCar Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo)
