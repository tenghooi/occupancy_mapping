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

class DynamicObject
{
private:
    Eigen::Vector4f max_vec_;
    Eigen::Vector4f min_vec_;

public:
    void SetObjBBox();
    void FilterObject(pcl::PCLPointCloud2& point_cloud);

    void MsgCallback();
};

void SetNodeParameters(const ros::NodeHandle& node);



#endif //_OBJECT_FILTER_H_