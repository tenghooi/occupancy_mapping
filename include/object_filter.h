#ifndef _OBJECT_FILTER_H_
#define _OBJECT_FILTER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>

struct DynamicObject
{
    double upperbound_x;
    double upperbound_y;
    double upperbound_z;
    double lowerbound_x;
    double lowerbound_y;
    double lowerbound_z;
};





#endif //_OBJECT_FILTER_H_