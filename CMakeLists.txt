file(GLOB SOURCES
    source/*.cpp
)

cmake_minimum_required(VERSION 3.28)
project(Floor_Car)

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(third_party/recastnavigation/Recast)
add_subdirectory(third_party/recastnavigation/Detour)
add_subdirectory(third_party/recastnavigation/DetourCrowd)
add_subdirectory(third_party/recastnavigation/DetourTileCache)
add_subdirectory(third_party/recastnavigation/DebugUtils)
add_subdirectory(third_party/recastnavigation/RecastDemo)

add_executable(FloorCar ${SOURCES})
target_link_libraries(FloorCar Recast Detour DetourCrowd DetourTileCache DebugUtils RecastDemo)
target_include_directories(FloorCar PRIVATE include/)