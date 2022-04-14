# OccupancyMapping
ROS package for 3D occupancy grid mapping

## Build & Run
### Pre-requisite
* PCL 1.7
* Eigen3
* OpenCV
* C++ 17
* ROS Melodic

### Compile
```bash
cd ~/catkin_ws/src
git clone https://github.com/tenghooi/occupancy_mapping.git
cd ..
catkin_make
```

### Running the package
```bash
roslaunch occupancy_mapping occupancy_mapping.launch
```
or with **rviz** for visualization

```bash
roslaunch occupancy_mapping test.launch
```
