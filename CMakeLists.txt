cmake_minimum_required(VERSION 2.8.3)
project(ed_cloud)

find_package(catkin REQUIRED COMPONENTS
    ed
)

#set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/hypertable/current/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
LINK_DIRECTORIES(/opt/hypertable/current/lib)
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
        /opt/hypertable/current/include/
        /opt/hypertable/current/include/thrift
        ${catkin_INCLUDE_DIRS}
)

add_library(ed_world_writer src/world_writer.cpp)
add_library(ed_world_reader src/world_reader.cpp)
add_library(ed_sync_server_plugin src/sync_server_plugin.cpp)
add_library(ed_sync_client_plugin src/sync_client_plugin.cpp)
add_library(ed_hypertable_writer_plugin src/hypertable_writer.cpp)
add_library(ed_hypertable_reader_plugin src/hypertable_reader.cpp)

add_dependencies(ed_sync_server_plugin ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)
add_dependencies(ed_sync_client_plugin ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencpp)

target_link_libraries(ed_sync_server_plugin ed_world_writer)
target_link_libraries(ed_sync_client_plugin ed_world_writer)
target_link_libraries(ed_hypertable_writer_plugin
    /opt/hypertable/current/lib/libHyperThrift.a
    /opt/hypertable/current/lib/libHyperCommon.a
    /opt/hypertable/current/lib/libHypertable.a
    /opt/hypertable/current/lib/libthrift.so
    /opt/hypertable/current/lib/libevent.so
    /opt/hypertable/current/lib/libboost_system.so
    /opt/hypertable/current/lib/libboost_program_options.so
    /opt/hypertable/current/lib/libboost_filesystem.so
    /opt/hypertable/current/lib/libsigar-amd64-linux.so
    /opt/hypertable/current/lib/libre2.so
    /opt/hypertable/current/lib/libcurses.so
    /opt/hypertable/current/lib/libtcmalloc_minimal.so
    /opt/hypertable/current/lib/libstdc++.so
    ed_world_writer)

target_link_libraries(ed_hypertable_reader_plugin
    /opt/hypertable/current/lib/libHyperThrift.a
    /opt/hypertable/current/lib/libHyperCommon.a
    /opt/hypertable/current/lib/libHypertable.a
    /opt/hypertable/current/lib/libthrift.so
    /opt/hypertable/current/lib/libevent.so
    /opt/hypertable/current/lib/libboost_system.so
    /opt/hypertable/current/lib/libboost_program_options.so
    /opt/hypertable/current/lib/libboost_filesystem.so
    /opt/hypertable/current/lib/libsigar-amd64-linux.so
    /opt/hypertable/current/lib/libre2.so
    /opt/hypertable/current/lib/libcurses.so
    /opt/hypertable/current/lib/libtcmalloc_minimal.so
    /opt/hypertable/current/lib/libstdc++.so
    ed_world_reader
    ed_world_writer
)

# ------------------------------------------------------------------------------------------------
#                                              TEST
# ------------------------------------------------------------------------------------------------

add_library(ed_bouncing_cubes test/bouncing_cubes.cpp)
target_link_libraries(ed_bouncing_cubes ${catkin_LIBRARIES})

