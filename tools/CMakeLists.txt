# Camera calibration tool
add_executable(CameraCalibration ${CMAKE_CURRENT_SOURCE_DIR}/camera_calibration/main.cpp)
target_link_libraries(CameraCalibration PRIVATE ${OpenCV_LIBS})

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	add_executable(ImuBias ${CMAKE_CURRENT_SOURCE_DIR}/imu_bias/main.cpp)
	target_link_libraries(ImuBias PRIVATE mpu6050)
endif()