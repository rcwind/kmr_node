##############################################################################
# NODELET
##############################################################################

add_library(kmr_nodelet kmr_nodelet.cpp)
add_dependencies(kmr_nodelet kmr_ros)
target_link_libraries(kmr_nodelet kmr_ros)

install(TARGETS kmr_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)