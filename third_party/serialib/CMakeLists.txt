add_library(serialib STATIC
    serialib.cpp
)

target_include_directories(serialib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)