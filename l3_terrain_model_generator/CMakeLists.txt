cmake_minimum_required(VERSION 3.0.2)
project(l3_terrain_model_generator)

#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  grid_map_core
  vigir_pluginlib
  l3_terrain_model
)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED common octree)

# Compiler definitions and options
add_l3_compile_definitions_and_options()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS Boost PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Specify additional locations of header files
set(HEADERS
  include/${PROJECT_NAME}/plugins/aggregator/process_chain.h
  include/${PROJECT_NAME}/plugins/aggregator/sensors.h
  include/${PROJECT_NAME}/plugins/base/filter_plugin.h
  include/${PROJECT_NAME}/plugins/base/generator_plugin.h
  include/${PROJECT_NAME}/plugins/base/grid_map_generator_plugin.h
  include/${PROJECT_NAME}/plugins/base/point_cloud_filter_plugin.h
  include/${PROJECT_NAME}/plugins/base/point_cloud_sensor_plugin.h
  include/${PROJECT_NAME}/plugins/base/processor_plugin.h
  include/${PROJECT_NAME}/plugins/base/publisher_plugin.h
  include/${PROJECT_NAME}/plugins/base/sensor_plugin.h
  include/${PROJECT_NAME}/plugins/std/filter/pcl_mls_smooth_filter.h
  include/${PROJECT_NAME}/plugins/std/filter/pcl_pass_through_box_filter.h
  include/${PROJECT_NAME}/plugins/std/filter/pcl_pass_through_ellipse_filter.h
  include/${PROJECT_NAME}/plugins/std/filter/pcl_statistical_outlier_filter.h
  include/${PROJECT_NAME}/plugins/std/filter/pcl_voxel_grid_filter.h
  include/${PROJECT_NAME}/plugins/std/generator/copy_map_layer_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/edges_cloud_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/elevation_map_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/gradients_cloud_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/ground_level_map_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/normals_cloud_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/occupancy_map_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/octree_generator.h
  include/${PROJECT_NAME}/plugins/std/generator/surface_mesh_generator.h
  include/${PROJECT_NAME}/plugins/std/publisher/grid_map_publisher.h
  include/${PROJECT_NAME}/plugins/std/publisher/normals_publisher.h
  include/${PROJECT_NAME}/plugins/std/publisher/occupancy_map_publisher.h
  include/${PROJECT_NAME}/plugins/std/publisher/octree_publisher.h
  include/${PROJECT_NAME}/plugins/std/publisher/point_cloud_publisher.h
  include/${PROJECT_NAME}/plugins/std/publisher/surface_mesh_publisher.h
  include/${PROJECT_NAME}/plugins/std/publisher/terrain_model_publisher.h
  include/${PROJECT_NAME}/plugins/std/sensor/generic_lidar_sensor.h
  include/${PROJECT_NAME}/plugins/std/sensor/generic_rgbd_sensor.h
  include/${PROJECT_NAME}/plugins/std/sensor/grid_map_sensor.h
  include/${PROJECT_NAME}/plugins/std/sensor/image_map_sensor.h
  include/${PROJECT_NAME}/plugins/std/sensor/sensor_pose_subscriber.h
  include/${PROJECT_NAME}/plugins/std/periodic_process.h
  include/${PROJECT_NAME}/utils/pcl/incremental_octree_search.h
  include/${PROJECT_NAME}/utils/pcl/octree_voxel_grid.h
  include/${PROJECT_NAME}/utils/pcl/pcl_data_handle.h
  include/${PROJECT_NAME}/utils/pcl/pcl_utils.h
  include/${PROJECT_NAME}/utils/pcl/point_cloud_filter.h
  include/${PROJECT_NAME}/utils/data_handle.h
  include/${PROJECT_NAME}/utils/data_manager.h
  include/${PROJECT_NAME}/utils/helper.h
  include/${PROJECT_NAME}/utils/macros.h
  include/${PROJECT_NAME}/utils/math.h
  include/${PROJECT_NAME}/utils/utils.h
  include/${PROJECT_NAME}/point_cloud_portioner_node.h
  include/${PROJECT_NAME}/terrain_model_generator_node.h
  include/${PROJECT_NAME}/terrain_model_generator.h
  include/${PROJECT_NAME}/typedefs.h
)

set(SOURCES
  src/plugins/aggregator/process_chain.cpp
  src/plugins/aggregator/sensors.cpp
  src/plugins/base/filter_plugin.cpp
  src/plugins/base/generator_plugin.cpp
  src/plugins/base/grid_map_generator_plugin.cpp
  src/plugins/base/point_cloud_filter_plugin.cpp
  src/plugins/base/point_cloud_sensor_plugin.cpp
  src/plugins/base/processor_plugin.cpp
  src/plugins/base/publisher_plugin.cpp
  src/plugins/base/sensor_plugin.cpp
  src/plugins/std/filter/pcl_mls_smooth_filter.cpp
  src/plugins/std/filter/pcl_pass_through_box_filter.cpp
  src/plugins/std/filter/pcl_pass_through_ellipse_filter.cpp
  src/plugins/std/filter/pcl_statistical_outlier_filter.cpp
  src/plugins/std/filter/pcl_voxel_grid_filter.cpp
  src/plugins/std/generator/copy_map_layer_generator.cpp
  src/plugins/std/generator/edges_cloud_generator.cpp
  src/plugins/std/generator/elevation_map_generator.cpp
  src/plugins/std/generator/gradients_cloud_generator.cpp
  src/plugins/std/generator/ground_level_map_generator.cpp
  src/plugins/std/generator/normals_cloud_generator.cpp
  src/plugins/std/generator/occupancy_map_generator.cpp
  src/plugins/std/generator/octree_generator.cpp
  src/plugins/std/generator/surface_mesh_generator.cpp
  src/plugins/std/publisher/grid_map_publisher.cpp
  src/plugins/std/publisher/normals_publisher.cpp
  src/plugins/std/publisher/occupancy_map_publisher.cpp
  src/plugins/std/publisher/octree_publisher.cpp
  src/plugins/std/publisher/point_cloud_publisher.cpp
  src/plugins/std/publisher/surface_mesh_publisher.cpp
  src/plugins/std/publisher/terrain_model_publisher.cpp
  src/plugins/std/sensor/generic_lidar_sensor.cpp
  src/plugins/std/sensor/generic_rgbd_sensor.cpp
  src/plugins/std/sensor/grid_map_sensor.cpp
  src/plugins/std/sensor/image_map_sensor.cpp
  src/plugins/std/sensor/sensor_pose_subscriber.cpp
  src/plugins/std/periodic_process.cpp
  src/utils/pcl/incremental_octree_search.cpp
  src/utils/pcl/octree_voxel_grid.cpp
  src/utils/pcl/pcl_data_handle.cpp
  src/utils/pcl/pcl_utils.cpp
  src/utils/pcl/point_cloud_filter.cpp
  src/utils/data_handle.cpp
  src/utils/data_manager.cpp
  src/utils/helper.cpp
  src/utils/math.cpp
  src/utils/utils.cpp
  src/terrain_model_generator.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

## Declare a cpp executable
add_executable(point_cloud_portioner_node src/point_cloud_portioner_node.cpp)
add_executable(terrain_model_generator_node src/terrain_model_generator_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(point_cloud_portioner_node ${PROJECT_NAME})
target_link_libraries(terrain_model_generator_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS src/${PROJECT_NAME}/image_map_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} point_cloud_portioner_node terrain_model_generator_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

# Mark plugin description file for installation
install(FILES
  plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_l3_terrain_model_generator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
