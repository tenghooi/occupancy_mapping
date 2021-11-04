#include "mapping.h"

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

    transform_sub_ = node.subscribe("transform", 10, &PoseCallBack, this);
    depth_sub_ = node.subscribe("depth", 10, &DepthCallBack, this);

    occupancy_pub_ = node.advertise<sensor_msgs::PointCloud>("OccupancyMap/occupancy_pointcloud", 1, true);
    text_pub_ = node.advertise<visualization_msgs::Marker>("OccupancyMap/text", 1, true);

    update_mesh_timer_ = node.createTimer(ros::Duration(parameters_.update_occupancy_every_n_sec),
                                          &UpdateOccupancyEvent, this);
    
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
            transformed_point = (transformed_point - raycast_origin_) / length
                                * parameters_.max_ray_length + raycast_origin_; 
            tmp_indx = occupancy_map_->SetOccupancy(transformed_point, 0);
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

}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::SynchronizationAndProcess()
{
    ros::Time depth_msg_time;
    double time_delay = 3e-3;

    while(!depth_image_queue_.empty())
    {
        bool new_pose = false;
        depth_msg_time = depth_image_queue_.front()->header.stamp;

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
        transform_ = transform_ * parameters_.T_D_B * parameters_.T_Body_Camera;

        raycast_origin_ = Eigen::Vector3d(transform_(0, 3), transform_(1, 3), transform_(2, 3))/transform_(3, 3);

        if constexpr(std::is_same<DepthMsgType, sensor_msgs::Image::ConstPtr>::value) 
        {
            DepthConversion();
        } 
        /* else if constexpr(std::is_same<DepthMsgType, sensor_msgs::PointCloud2::ConstPtr>::value) 
        {
            sensor_msgs::PointCloud2::ConstPtr tmp = depth_queue_.front();
            pcl::fromROSMsg(*tmp, cloud_);
        } */

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
void Mapping<DepthMsgType, PoseMsgType>::DepthCallBack(const DepthMsgType& depth_image_msg)
{
    depth_image_queue_.push(depth_image_msg);
    SynchronizationAndProcess();
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::PoseCallBack(const PoseMsgType& pose_msg)
{
    Eigen::Vector3d pose;
    Eigen::Quaterniond q;

    pose << pose_msg->pose.position.x, 
            pose_msg->pose.position.y,
            pose_msg->pose.position.z;

    q << pose_msg->pose.orientation.w,
         pose_msg->pose.orientation.x,
         pose_msg->pose.orientation.y,
         pose_msg->pose.orientation.z;

    transform_queue_.push(std::make_tuple(pose_msg->header.stamp, pose, q));

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
        occupancy_map_->GetPointCloud(point_cloud);
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