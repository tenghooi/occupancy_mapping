#ifndef _MAPPING_H_
#define _MAPPING_H_

#include "raycasting.h"
#include "occupancy_map.h"

#include <iostream>
#include <queue>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

template<class DepthMsgType, class PoseMsgType>
class Mapping
{
private:
    Parameters parameters_;
    OccupancyMap* occupancy_map_;
    pcl::PointCloud<pcl::PointXYZ> cloud_;

    bool new_msg_ = false;

    ros::Publisher occupancy_pub_;
    ros::Publisher text_pub_;
    ros::Subscriber transform_sub_;
    ros::Subscriber depth_sub_;

    ros::Timer update_mesh_timer_;

    Eigen::Vector3d sync_pos_;
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d raycast_origin_;
    Eigen::Quaterniond sync_q_;

    std::queue<std::tuple<ros::Time, Eigen::Vector3d, Eigen::Quaterniond>> transform_queue_;
    std::queue<DepthMsgType> depth_image_queue_;
    DepthMsgType sync_depth_;

    cv::Mat img_[2];
    Eigen::Matrix4d transform_;
    Eigen::Matrix4d last_transform_;

    uint image_count_ = 0;
    uint esdf_count_ = 0;
    uint total_ = 0;

    std::vector<int> set_free_;
    std::vector<int> set_occ_;

public:
    Mapping(ros::NodeHandle node);
    ~Mapping();

    void RayCastingProcess(int number_depth_points, int tt);

    void DepthConversion();
    void SynchronizationAndProcess();
    void DepthCallBack(const DepthMsgType& depth_image_msg);
    void PoseCallBack(const PoseMsgType& pose_msg);

    void Visualization(OccupancyMap* occupancy_map, bool global_vis, const std::string& text);
    
    void UpdateOccupancyEvent(const ros::TimerEvent & /*event*/);
};


#endif