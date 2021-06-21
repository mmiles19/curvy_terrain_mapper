cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(curvy_terrain_mapper)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    visualization_msgs
    #pcl_ros
    #message_generation
    tf_conversions
    tf
    dynamic_reconfigure

    grid_map_ros
    grid_map_core
    grid_map_pcl
    grid_map_msgs
)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choosing Release build type - the fastest option."
       FORCE )
endif()

generate_dynamic_reconfigure_options(
  cfg/CurvyTerrainMapper.cfg
)

#add_message_files(
#  DIRECTORY
#    msg
#  FILES
#    PointCloud2Array.msg
#  )

#generate_messages(
#  DEPENDENCIES
#    sensor_msgs
#)

catkin_package(
  CATKIN_DEPENDS roscpp 
  )

include_directories(
    ${catkin_INCLUDE_DIRS}
    #${colormap_SOURCE_DIR}/include
    )

include_directories (${PCL_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include")

link_directories (${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})

add_executable( curvy_terrain_mapper_node
  src/curvy_terrain_mapper_node.cpp
  src/preanalysis.cpp 
  src/regions.cpp 
  src/segmentPatch.cpp 
  src/regiongrowing.cpp 
  src/voxSAC.cpp 
  src/splitmerge.cpp 
  src/planeshape.cpp 
  src/Stairs.cpp 
  src/StairVector.cpp 
  src/recognition.cpp
  src/prediction.cpp
)
add_dependencies( curvy_terrain_mapper_node
  curvy_terrain_mapper_gencfg
  curvy_terrain_mapper_gencpp
)
target_link_libraries( curvy_terrain_mapper_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

add_executable( grid_map_node
  src/grid_map_node.cpp
)
# add_dependencies( curvy_terrain_mapper_node
#   curvy_terrain_mapper_gencfg
#   curvy_terrain_mapper_gencpp
# )
target_link_libraries( grid_map_node
  ${catkin_LIBRARIES}
)
