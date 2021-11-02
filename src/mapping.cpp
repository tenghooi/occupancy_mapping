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

    transform_sub_ = node.subscribe("transform", 10, PoseCallBack, this);
    depth_sub_ = node.subscribe("depth", 10, DepthCallBack, this);

    occupancy_pub_ = node.advertise<sensor_msgs::PointCloud>("OccupancyMap/occupancy_pointcloud", 1, true);
    text_pub_ = node.advertise<visualization_msgs::Marker>("OccupancyMap/text", 1, true);

    update_mesh_timer_ = node.createTimer(ros::Duration(parameters_.update_occupancy_every_n_sec),
                                          &UpdateEsdfEvent, this);
    
}

template<class DepthMsgType, class PoseMsgType>
Mapping<DepthMsgType, PoseMsgType>::~Mapping()
{
    delete occupancy_map_;
}

template<class DepthMsgType, class PoseMsgType>
void Mapping<DepthMsgType, PoseMsgType>::RayCastingProcess(int i, int part, int tt)
{
    Eigen::Vector3d half = {0.5, 0.5, 0.5};
    for(int indx = part * i; indx < part * (i + 1); indx++)
    {
        std::vector<Eigen::Vector3d> output;

        if(indx > cloud_.points.size()) break;

        pcl::PointXYZ point = cloud_.points[indx];
        int count = 0;

        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
            continue;

        Eigen::Vector4d tmp = transform_ * 
                              Eigen::Vector4d(point.x, point.y, point.z, 1);
        Eigen::Vector3d transformed_point = {tmp[0], tmp[1], tmp[2]} / tmp[3];

        int tmp_indx;
        double length = (transformed_point - raycast_origin_).norm();
        
        if (length < parameters_.min_ray_length)
            continue;
        else if (length > parameters_.max_ray_length)
        {
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
                     parameters_.bottom_left/parameters_.resolution,
                     parameters_.upper_right/parameters_.resolution,
                     &output);
        
        for (size_t i = output.size() - 2; i >= 0; i--)
        {
            Eigen::Vector3d tmp = (output[i] + half) parameters_.resolution;

            length = (tmp - raycast_origin_).norm();
            if (length < parameters_.min_ray_length_)
                    break;
            if (length > parameters_.max_ray_length_)
                continue;
            int tmp_idx;
            tmp_idx = occupancy_map_->SetOccupancy(tmp, 0);

            if (tmp_idx != -10000)
            {
                if (set_free_[tmp_idx] == tt)
                {
                    if (++count >= 1)
                    {
                        count = 0;
                        break;
                    }
                }
                else
                {
                    set_free_[tmp_idx] = tt;
                    count = 0;
                }
            }
        }
    }
}