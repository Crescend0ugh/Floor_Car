file(GLOB SOURCES Source/*.cpp)
add_library(RecastDemo ${SOURCES})

add_library(RecastNavigation::RecastDemo ALIAS RecastDemo)
set_target_properties(RecastDemo PROPERTIES DEBUG_POSTFIX -d)

set(RecastDemo_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Include")

target_include_directories(RecastDemo PUBLIC
    "$<BUILD_INTERFACE:${RecastDemo_INCLUDE_DIR}>"
)

target_link_libraries(RecastDemo
    Detour
    Recast
    DebugUtils
    DetourCrowd
    DetourTileCache
)

set_target_properties(RecastDemo PROPERTIES
        SOVERSION ${SOVERSION}
        VERSION ${LIB_VERSION}
        COMPILE_PDB_OUTPUT_DIRECTORY .
        COMPILE_PDB_NAME "RecastDemo-d"
        )

install(TARGETS RecastDemo
        EXPORT recastnavigation-targets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT library
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation
        )

file(GLOB INCLUDES Include/*.h)
install(FILES ${INCLUDES} DESTINATION
    ${CMAKE_INSTALL_INCLUDEDIR}/recastnavigation)
if(MSVC)
    install(FILES "$<TARGET_FILE_DIR:RecastDemo>/RecastDemo-d.pdb" CONFIGURATIONS "Debug" DESTINATION "lib" OPTIONAL)
endif()