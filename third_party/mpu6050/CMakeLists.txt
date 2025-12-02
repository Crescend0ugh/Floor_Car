add_library(mpu6050 STATIC
    MPU6050.cpp
)

target_include_directories(mpu6050 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(mpu6050 PUBLIC i2c)