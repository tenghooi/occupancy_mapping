cmake_minimum_required(VERSION 2.8.3)

# Create Project - set the project name and version
project(occupancy_mapping)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

# Find Packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             nav_msgs
             visualization_msgs
             tf
             cv_bridge
             pcl_ros
             pcl_conversions
             )   

find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
)

# Include header files
include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(occupancy_mapping 
               include/parameters.h
               src/parameters.cpp
               include/raycasting.h
               src/raycasting.cpp
               include/occupancy_map.h
               src/occupancy_map.cpp
               include/mapping.h
               
               src/main.cpp
              )

target_link_libraries(occupancy_mapping 
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
                      ${OpenCV_LIBS}
                      Eigen3::Eigen
                      )
# target_link_libraries(occupancy_mapping ${PCL_LIBRARIES} Eigen3::Eigen)



