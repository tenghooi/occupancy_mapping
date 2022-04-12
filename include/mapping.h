#ifndef _MAPPING_H_
#define _MAPPING_H_

#include "raycasting.h"
#include "occupancy_map.h"

#include <iostream>
#include <queue>
#include <tuple>
#include <type_traits>

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
#include <sensor_msgs/Imu.h>
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
    ros::Time begin = ros::Time::now();

    Eigen::Vector3d sync_pos_;
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d raycast_origin_;
    Eigen::Quaterniond sync_q_;

    std::queue<std::tuple<ros::Time, Eigen::Vector3d, Eigen::Quaterniond>> transform_queue_;
    std::queue<std::tuple<ros::Time, DepthMsgType>> depth_image_queue_;
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
    void DepthCallBack(const DepthMsgType& depth_image_msg); //(const DepthMsgType& depth_image_msg)
    void PoseCallBack(const PoseMsgType& pose_msg);

    void Visualization(OccupancyMap* occupancy_map, bool global_vis, const std::string& text);
    
    void UpdateOccupancyEvent(const ros::TimerEvent & /*event*/);
};



template<class DepthMsgType, class PoseMsgType>
Mapping<DepthMsgType, PoseMsgType>::Mapping(ros::NodeHandle node)
{
    SetNodeParameters(parameters_, node);

    occupancy_map_ = new OccupancyMap(parameters_.bottom_left, 
                                      parameters_.resolution,
                                      parameters_.map_size);

    occupancy_map_->SetParameters(parameters_.prob_hit,
                                  parameters_.prob_miss,  
                                  parameters_.prob_min,  
                                  parameters_.prob_max, 
                                  parameters_.prob_occupancy);
    
    set_free_.resize(occupancy_map_->grid_total_size_);
    set_occ_.resize(occupancy_map_->grid_total_size_);
    std::fill(set_free_.begin(), set_free_.end(), 0);
    std::fill(set_occ_.begin(), set_occ_.end(), 0);

    transform_sub_ = node.subscribe("odometry", 10, &Mapping::PoseCallBack, this);
    depth_sub_ = node.subscribe("depth", 10, &Mapping::DepthCallBack, this);

    occupancy_pub_ = node.advertise<sensor_msgs::PointCloud>("visualize_pointcloud", 1, true);
    text_pub_ = node.advertise<visualization_msgs::Marker>("text", 1, true);

    update_mesh_timer_ = node.createTimer(ros::Duration(parameters_.update_occupancy_every_n_sec),
                                          &Mapping::UpdateOccupancyEvent, this);
    
}

template<class DepthMsgType, class PoseMsgType>
Mapping<DepthMsgType, PoseMsgType>::~Mapping()
{
    delete occupancy_map_;
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::RayCastingProcess(int number_depth_points, int tt)
{
    Eigen::Vector3d half = {0.5, 0.5, 0.5};

    for(int indx = 0; indx < number_depth_points; indx++)
    {
        std::vector<Eigen::Vector3d> traversed_voxels;

        pcl::PointXYZ point = cloud_.points[indx];
        int count = 0;

        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
            continue;

        Eigen::Vector4d tmp_point = transform_ * 
                              Eigen::Vector4d(point.x, point.y, point.z, 1);
        Eigen::Vector3d transformed_point = Eigen::Vector3d(tmp_point[0], tmp_point[1], tmp_point[2]) / tmp_point[3];

        int tmp_indx;
        double length = (transformed_point - raycast_origin_).norm();
        
        if (length < parameters_.min_ray_length)
            continue;
        else if (length > parameters_.max_ray_length)
        {
            // Normalizes the vector and set to max_ray_length. Set the measured point voxel occupancy to 0.
            // transformed_point = (transformed_point - raycast_origin_) / length
            //                     * parameters_.max_ray_length + raycast_origin_; 
            // tmp_indx = occupancy_map_->SetOccupancy(transformed_point, 0);
            continue;
        }
        else
        {
            tmp_indx = occupancy_map_->SetOccupancy(transformed_point, 1);
        }

        if(tmp_indx != -10000)
        {
            if(set_occ_[tmp_indx] == tt)
                continue;
            else
                set_occ_[tmp_indx] = tt;
        }

        RayCasting3D(raycast_origin_/parameters_.resolution,
                     transformed_point/parameters_.resolution,
                     traversed_voxels);
        
        // Set occupancy 0 for all traversed voxels except the measured one.
        for (size_t i = traversed_voxels.size() - 2; i >= 0; i--)
        {
            Eigen::Vector3d current_voxel = (traversed_voxels[i] + half) * parameters_.resolution;

            length = (current_voxel - raycast_origin_).norm();
            if (length < parameters_.min_ray_length)
                    break;
            if (length > parameters_.max_ray_length)
                continue;

            int tmp_indx;
            tmp_indx = occupancy_map_->SetOccupancy(current_voxel, 0);

            if (tmp_indx != -10000)
            {
                if (set_free_[tmp_indx] == tt)
                {
                    if (++count >= 1)
                    {
                        count = 0;
                        break;
                    }
                }
                else
                {
                    set_free_[tmp_indx] = tt;
                    count = 0;
                }
            }
        }
    }
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::DepthConversion()
{
    ++image_count_;
    cv::Mat &current_image = img_[image_count_ & 1];
    cv::Mat &last_image = img_[!(image_count_ & 1)];

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(depth_image_queue_.front(), depth_image_queue_.front()->encoding);

    constexpr double k_depth_scaling_factor = 1000.0;
    if (depth_image_queue_.front()->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, k_depth_scaling_factor);

    }
    cv_ptr->image.copyTo(current_image);

    double depth;
    cloud_.clear();

    uint16_t* row_ptr;
    int cols = current_image.cols, rows = current_image.rows;
    if (!parameters_.use_depth_filter) 
    {
        for (int v = 0; v < rows; v++) 
        {
            row_ptr = current_image.ptr<uint16_t>(v);
            for (int u = 0; u < cols; u++) 
            {
                depth = (*row_ptr++)/k_depth_scaling_factor;
                if (depth > parameters_.filter_max_depth || depth < parameters_.filter_min_depth)
                        continue;
                pcl::PointXYZ point;
                point.x = (u - parameters_.center_x)*depth/parameters_.focal_length_x;
                point.y = (v - parameters_.center_y)*depth/parameters_.focal_length_y;
                point.z = depth;
                cloud_.push_back(point);
            }
        }
    } 
    
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::SynchronizationAndProcess()
{
    ros::Time depth_msg_time;
    double time_delay = 3e-3;

    while(!depth_image_queue_.empty())
    {
        bool new_pose = false;
        depth_msg_time = std::get<0>(depth_image_queue_.front());
        ROS_INFO("Depth msg time: %lf", depth_msg_time.toSec());

        while (transform_queue_.size() > 1 && 
               std::get<0>(transform_queue_.front()) <= depth_msg_time + ros::Duration(time_delay))
        {
            sync_pos_ = std::get<1>(transform_queue_.front());
            sync_q_ = std::get<2>(transform_queue_.front());
            transform_queue_.pop();

            new_pose = true;
        }

        if (transform_queue_.empty() ||
            std::get<0>(transform_queue_.front()) <= depth_msg_time + ros::Duration(time_delay))
        {
            break;
        }

        if (!new_pose)
        {
            depth_image_queue_.pop();
            continue;
        }

        new_msg_ = true;  

        if (parameters_.use_depth_filter) {last_transform_ = transform_;}
        // 4x4 transformation matrix
        transform_.block<3, 3>(0, 0) = sync_q_.toRotationMatrix();
        transform_.block<3, 1>(0, 3) = sync_pos_;
        transform_(3, 0) = 0;
        transform_(3, 1) = 0;
        transform_(3, 2) = 0;
        transform_(3, 3) = 1;
        //transform_ = transform_ * parameters_.T_D_B * parameters_.T_Body_Camera;
        transform_ = transform_ ;

        raycast_origin_ = Eigen::Vector3d(transform_(0, 3), transform_(1, 3), transform_(2, 3))/transform_(3, 3);

        if constexpr(std::is_same<DepthMsgType, sensor_msgs::Image::ConstPtr>::value) 
        {
            DepthConversion();
        } 
         else if constexpr(std::is_same<DepthMsgType, sensor_msgs::PointCloud2::ConstPtr>::value) 
        {
            sensor_msgs::PointCloud2::ConstPtr tmp = std::get<1>(depth_image_queue_.front());
            pcl::fromROSMsg(*tmp, cloud_);
        } 

        if (cloud_.points.size()==0) 
        {
            depth_image_queue_.pop();
            continue;
        }

        {
        int tt = ++total_;
        RayCastingProcess(cloud_.points.size(), tt);
        }
        
        depth_image_queue_.pop();

    }
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::DepthCallBack(const DepthMsgType& depth_image_msg) //(const DepthMsgType& depth_image_msg)
{
    //ROS_INFO("Initial time: %d", begin);
    std_msgs::Header header;

    ros::Duration duration = ros::Time::now() - begin;
    header.stamp.sec = duration.sec;
    header.stamp.nsec = duration.nsec;

    //ROS_INFO("Depth call back time: %lf", header.stamp.toSec());
    //ROS_INFO("Depth msg added");
    depth_image_queue_.push(std::make_tuple(header.stamp, depth_image_msg));
    SynchronizationAndProcess();
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::PoseCallBack(const PoseMsgType& pose_msg)
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond q;
    std_msgs::Header header;

    //ros::Duration dduration = ros::Time::now() - begin;
    double duration = (ros::Time::now() - begin).toSec();
    //header.stamp.sec = dduration.sec;
    //header.stamp.nsec = dduration.nsec;
    header.stamp = ros::Time().fromSec(duration);

    pos << pose_msg->pose.pose.position.x, 
            pose_msg->pose.pose.position.y,
            pose_msg->pose.pose.position.z;

    q = Eigen::Quaterniond (pose_msg->pose.pose.orientation.w,
                            pose_msg->pose.pose.orientation.x,
                            pose_msg->pose.pose.orientation.y,
                            pose_msg->pose.pose.orientation.z);
                            
    //pos << 0, 0, 0;
    //q = Eigen::Quaterniond (1, 0, 0, 0);
    //ROS_INFO("Pose call back: %lf", header.stamp.toSec());
    transform_queue_.push(std::make_tuple(header.stamp, pos, q));
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::Visualization(OccupancyMap* occupancy_map,
                                                       bool global_vis,
                                                       const std::string& text)
{
    if (occupancy_map_ != nullptr)
    {
        std::cout << "Visualization" << std::endl;

        if(global_vis) occupancy_map_->SetOriginalRange();

        sensor_msgs::PointCloud point_cloud;
        occupancy_map_->GetVisualizePointCloud(point_cloud, parameter_.map_frame_id);
        occupancy_pub_.publish(point_cloud);
    }

    if (!text.empty()) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.id = 3456;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::MODIFY;

        marker.pose.position.x = 8.0;
        marker.pose.position.y = 2.0;
        marker.pose.position.z = 3.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.text = text;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.6;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        text_pub_.publish(marker);
     }
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::UpdateOccupancyEvent(const ros::TimerEvent&)
{
    if (!new_msg_) return;

    new_msg_ = false;
    current_pos_ = sync_pos_;

    esdf_count_++;
    std::cout << "Running " << esdf_count_ << " updates." << std::endl;

    if(occupancy_map_->CheckUpdate())
    {
        if(parameters_.global_update)
        {
            occupancy_map_->SetOriginalRange();
        }

        occupancy_map_->UpdateOccupancy(parameters_.global_update);
    }

    if (parameters_.visualize_every_n_updates != 0 &&
        esdf_count_ % parameters_.visualize_every_n_updates == 0)
    {
        Visualization(occupancy_map_, parameters_.global_vis, "");
    }

}

#endif