#ifndef _MAPPING_H_
#define _MAPPING_H_

#include "raycasting.h"
#include "occupancy_map.h"

#include <iostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>

template<class DepthMsgType, class PoseMsgType>
class Mapping
{
private:
    Parameters parameters_;
    OccupancyMap* occupancy_map_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;

    bool new_msg_ = false;

    

};


#endif