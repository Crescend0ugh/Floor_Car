set(TEST_SUBDIRECTORIES 
	math_test 
	navmesh_test
	pcd_test 
	raylib_test 
	networking_test
	opencv_test
	bounding_boxes_test
	mesh_reconstruction_test
)

foreach(SUBDIRECTORY IN LISTS TEST_SUBDIRECTORIES)
    add_executable(${SUBDIRECTORY} 
        ${SUBDIRECTORY}/${SUBDIRECTORY}.cpp
    )
	
    ADD_CONTENT_POST_BUILD(${SUBDIRECTORY}) 

    target_link_libraries(${SUBDIRECTORY} 
		PRIVATE 
			FloorCar_lib
    )
endforeach()