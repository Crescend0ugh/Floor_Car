# dependencies

find_package(Threads REQUIRED)

# third-party


# SDL2
find_package(SDL2 REQUIRED)

string(STRIP "${SDL2_LIBRARIES}" SDL2_LIBRARIES)

message(STATUS "SDL2_INCLUDE_DIRS = ${SDL2_INCLUDE_DIRS}")
message(STATUS "SDL2_LIBRARIES    = ${SDL2_LIBRARIES}")


# common

set(TARGET whisper-common)

unset(COMMON_EXTRA_LIBS)


add_library(${TARGET} STATIC
    common.h
    common.cpp
    common-ggml.h
    common-ggml.cpp
    common-whisper.h
    common-whisper.cpp
    grammar-parser.h
    grammar-parser.cpp
    )

#include(DefaultTargetOptions)

target_include_directories(${TARGET} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${TARGET} PRIVATE whisper ${COMMON_EXTRA_LIBS} ${CMAKE_DL_LIBS})

set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)
set_target_properties(${TARGET} PROPERTIES FOLDER "libs")

# common-sdl

set(TARGET whisper-common-sdl)

add_library(${TARGET} STATIC
    common-sdl.h
    common-sdl.cpp
    )

#include(DefaultTargetOptions)

target_include_directories(${TARGET} PUBLIC  ${SDL2_INCLUDE_DIRS})
target_link_libraries     (${TARGET} PRIVATE ${SDL2_LIBRARIES})

set_target_properties(${TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)

# examples

include_directories(${CMAKE_CURRENT_SOURCE_DIR})