cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Create Project - set the project name and version
project(occupancy_mapping VERSION 0.1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Create bin folder
if(NOT EXISTS ${CMAKE_HOME_DIRECTORY}/bin)
  file(MAKE_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)
endif()

# Set destination of executable
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/bin)

# Find Packages
find_package(PCL 1.8 REQUIRED)
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Include header files
include_directories(${PROJECT_SOURCE_DIR}/include)

# Include PCL library
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add executable
add_executable(occupancy_mapping src/main.cpp)
# target_link_libraries(occupancy_mapping ${PCL_LIBRARIES})
# target_link_libraries(occupancy_mapping ${PCL_LIBRARIES} Eigen3::Eigen)



