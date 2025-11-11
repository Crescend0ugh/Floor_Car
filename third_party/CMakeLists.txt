# Recast
add_subdirectory(recastnavigation/Recast)
add_subdirectory(recastnavigation/Detour)
add_subdirectory(recastnavigation/DetourCrowd)
add_subdirectory(recastnavigation/DetourTileCache)
add_subdirectory(recastnavigation/DebugUtils)
add_subdirectory(recastnavigation/RecastDemo)

# Raylib
add_subdirectory(raylib) 

# ncnn
add_subdirectory(ncnn)

add_subdirectory(serialib)

add_subdirectory(serial_transfer)

add_subdirectory(whisper.cpp) 

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
	add_subdirectory(pigpio)
endif()