add_executable(CameraCalibration ${CMAKE_CURRENT_SOURCE_DIR}/camera_calibration.cpp)
target_link_libraries(CameraCalibration PRIVATE ${OpenCV_LIBS})

add_executable(LidarCameraCalibration ${CMAKE_CURRENT_SOURCE_DIR}/lidar_camera_calibration.cpp)
target_link_libraries(LidarCameraCalibration PRIVATE ${OpenCV_LIBS})

# Anything involving the IMU is Linux only
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	add_executable(ImuBias ${CMAKE_CURRENT_SOURCE_DIR}/imu_bias.cpp)
	target_link_libraries(ImuBias PRIVATE mpu6050)

	add_executable(ImuStaticData ${CMAKE_CURRENT_SOURCE_DIR}/imu_static_data.cpp)
	target_link_libraries(ImuStaticData PRIVATE mpu6050)

	add_executable(CameraImuCalibration ${CMAKE_CURRENT_SOURCE_DIR}/camera_imu_calibration.cpp)
	target_link_libraries(CameraImuCalibration PRIVATE mpu6050 ${OpenCV_LIBS})
endif()