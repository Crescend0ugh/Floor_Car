add_library(serial_transfer STATIC
    Packet.cpp
)

target_include_directories(serial_transfer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)