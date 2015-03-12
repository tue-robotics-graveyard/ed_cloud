cmake_minimum_required(VERSION 2.8.3)
project(ed_cloud)

find_package(catkin REQUIRED COMPONENTS
    ed
)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
add_message_files(
   FILES
   EntityUpdateInfo.msg
   EntityCreateInfo.msg
   Polygon.msg
   Mesh.msg
   WorldModelDelta.msg
)

# Generate services
add_service_files(
    FILES
    GetWorldModel.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
)

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

add_library(ed_world_writer src/world_writer.cpp)
add_library(ed_sync_server_plugin src/sync_server_plugin.cpp)
add_library(ed_sync_client_plugin src/sync_client_plugin.cpp)
add_dependencies(ed_sync_server_plugin ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)
add_dependencies(ed_sync_client_plugin ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)
target_link_libraries(ed_sync_server_plugin ed_world_writer)
target_link_libraries(ed_sync_client_plugin ed_world_writer)

# ------------------------------------------------------------------------------------------------
#                                              TEST
# ------------------------------------------------------------------------------------------------

add_library(ed_bouncing_cubes test/bouncing_cubes.cpp)
target_link_libraries(ed_bouncing_cubes ${catkin_LIBRARIES})

