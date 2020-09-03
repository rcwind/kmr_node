##############################################################################
# SOURCES
##############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# LIBRARY
##############################################################################

add_library(kmr_ros ${SOURCES})
add_dependencies(kmr_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(kmr_ros ${catkin_LIBRARIES})

install(TARGETS kmr_ros
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
