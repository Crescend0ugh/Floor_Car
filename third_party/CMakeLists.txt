# Raylib (fetched directly from GitHub)
include(FetchContent)
set(FETCHCONTENT_QUIET FALSE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE) # don't build the supplied examples
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE) # don't build the supplied example games

FetchContent_Declare(
    raylib
    GIT_REPOSITORY "https://github.com/raysan5/raylib.git"
    GIT_TAG "master"
    GIT_PROGRESS TRUE
)

FetchContent_MakeAvailable(raylib)

# Recast

add_subdirectory(recastnavigation/Recast)
add_subdirectory(recastnavigation/Detour)
add_subdirectory(recastnavigation/DetourCrowd)
add_subdirectory(recastnavigation/DetourTileCache)
add_subdirectory(recastnavigation/DebugUtils)
add_subdirectory(recastnavigation/RecastDemo)