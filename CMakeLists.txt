cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Create Project - set the project name and version
project(occupancy_mapping VERSION 0.1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE "Release")

# Create bin folder
if(NOT EXISTS ${CMAKE_HOME_DIRECTORY}/bin)
  file(MAKE_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)
endif()

# Set destination of executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

# Find Packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             nav_msgs
             visualization_msgs
             tf
             cv_bridge
             )   
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

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
  ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(occupancy_mapping 
               src/main.cpp
               src/occupancy_map.cpp
              )
target_link_libraries(occupancy_mapping 
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})
# target_link_libraries(occupancy_mapping ${PCL_LIBRARIES} Eigen3::Eigen)



