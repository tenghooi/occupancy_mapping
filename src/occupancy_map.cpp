#include "occupancy_map.h"
#include <ctime>

OccupancyMap::OccupancyMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size)
    : map_size_(map_size), origin_(origin), resolution_(resolution)
{
    grid_size_[0] = ceil(map_size_[0] / resolution_);
    grid_size_[1] = ceil(map_size_[1] / resolution_);
    grid_size_[2] = ceil(map_size_[2] / resolution_);

    grid_size_yz_ = grid_size_[1] * grid_size_[2];
    grid_total_size_ = grid_size_[0] * grid_size_yz_;

    min_range_ = origin_;
    max_range_ = origin_ + map_size_;

    infinity_ = 10000;
    undefined_ = -10000;

    min_vec_= Eigen::Vector3i::Zero();
    max_vec_ << grid_size_(0) - 1, grid_size_(1) - 1, grid_size_(2) - 1;
    last_min_vec_ = min_vec_;
    last_max_vec_ = max_vec_;

    std::cout << "Total grid size: " << grid_total_size_ << std::endl;
    occupancy_buffer_.resize(grid_total_size_);
    distance_buffer_.resize(grid_total_size_);
    num_hit_.resize(grid_total_size_);
    num_miss_.resize(grid_total_size_);

    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);
    std::fill(distance_buffer_.begin(), distance_buffer_.end(), undefined_);
    std::fill(num_hit_.begin(), num_hit_.end(), 0);
    std::fill(num_miss_.begin(), num_miss_.end(), 0);

}

double OccupancyMap::Logit(const double& prob) const
{
    return log(prob / (1 - prob));
}

bool OccupancyMap::Exist(const int& indx) 
{
    return occupancy_buffer_[indx] > occupancy_threshold_logit_;
}

void OccupancyMap::Vox2Pos(Eigen::Vector3i& voxel, Eigen::Vector3d& pos)
{
    pos[0] = (voxel[0] + 0.5) * resolution_ + origin_[0];
    pos[1] = (voxel[1] + 0.5) * resolution_ + origin_[1];
    pos[2] = (voxel[2] + 0.5) * resolution_ + origin_[2];
}

void OccupancyMap::Pos2Vox(Eigen::Vector3d& pos, Eigen::Vector3i& voxel)
{
    voxel[0] = floor((pos[0] - origin_[0]) / resolution_);
    voxel[1] = floor((pos[1] - origin_[1]) / resolution_);
    voxel[2] = floor((pos[2] - origin_[2]) / resolution_);
}

int OccupancyMap::Vox2Indx(Eigen::Vector3i& voxel)
{
    return voxel[0] * grid_size_yz_ + voxel[1] * grid_size_[2] + voxel[2];
}

Eigen::Vector3i OccupancyMap::Indx2Vox(int& indx)
{
    return Eigen::Vector3i(indx / grid_size_yz_, 
                           indx % grid_size_yz_ / grid_size_[2],
                           indx % grid_size_[2]);
}

void OccupancyMap::SetParameters(const double& prob_hit, const double& prob_miss, 
                                 const double& prob_min, const double& prob_max, 
                                 const double& prob_occupancy)
{
    logit_hit_ = Logit(prob_hit);
    logit_miss_ = Logit(prob_miss);
    max_logit_ = Logit(prob_max);
    min_logit_ = Logit(prob_min);
    occupancy_threshold_logit_ = Logit(prob_occupancy);
}

bool OccupancyMap::CheckUpdate()
{
    return !occupancy_queue_.empty();
}

void OccupancyMap::UpdateOccupancy(bool global_map)
{
    std::cout << "Occupancy update queue size: " << occupancy_queue_.size() << std::endl;

    while (!occupancy_queue_.empty())
    {
        QueueElement voxel = occupancy_queue_.front();
        occupancy_queue_.pop();
        int indx = Vox2Indx(voxel.point_);
        int occupied = Exist(indx);
        double log_odds_update = (num_hit_[indx] >= num_miss_[indx] - num_hit_[indx]) ?
                                 logit_hit_ : logit_miss_;
        
        num_hit_[indx] = num_miss_[indx] = 0;

        if ((log_odds_update >= 0 && occupancy_buffer_[indx] >= max_logit_) ||
            (log_odds_update <= 0 && occupancy_buffer_[indx] <= min_logit_))
        {
            continue;
        }

        occupancy_buffer_[indx] = std::min(
                                  std::max(occupancy_buffer_[indx] + log_odds_update, min_logit_),
                                  max_logit_);
        
    }
    
}

int OccupancyMap::SetOccupancy(Eigen::Vector3d pos, int occ)
{
    if (occ != 1 && occ != 0)
    {
        std::cerr << "Occ value error!" << std::endl;
        return undefined_;
    }

    Eigen::Vector3i voxel;
    Pos2Vox(pos, voxel);

    return SetOccupancy(voxel, occ);
}

int OccupancyMap::SetOccupancy(Eigen::Vector3i voxel, int occ)
{
    int indx = Vox2Indx(voxel);

    num_miss_[indx]++;
    num_hit_[indx] += occ;
    occupancy_queue_.push(QueueElement{voxel, 0.0});
    
    return indx;
}

int OccupancyMap::GetOccupancy(Eigen::Vector3i voxel)
{
    return Exist(Vox2Indx(voxel));
}

int OccupancyMap::GetOccupancy(Eigen::Vector3d pos)
{
    Eigen::Vector3i voxel;
    Pos2Vox(pos, voxel);

    return Exist(Vox2Indx(voxel));
}

void OccupancyMap::SetOriginalRange()
{
    min_vec_= Eigen::Vector3i::Zero();
    max_vec_ << grid_size_(0) - 1, grid_size_(1) - 1, grid_size_(2) - 1;
    last_min_vec_ = min_vec_;
    last_max_vec_ = max_vec_;
}

void OccupancyMap::GetPointCloud(sensor_msgs::PointCloud& point_cloud)
{
    point_cloud.header.frame_id = "camera_init";
    //point_cloud.header.frame_id = "uav1/t265_odom_frame";
    point_cloud.points.clear();

    for (int x = min_vec_[0]; x <= max_vec_[0]; ++x)
        for (int y = min_vec_[1]; y <= max_vec_[1]; ++y)
            for (int z = min_vec_[2]; z <= max_vec_[2]; ++z) 
            {
                Eigen::Vector3i tmp_voxel = {x, y, z};

                if (!Exist(Vox2Indx(tmp_voxel)))
                    continue;

                Eigen::Vector3d pos;
                Vox2Pos(tmp_voxel, pos);

                geometry_msgs::Point32 p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = pos(2);
                
                point_cloud.points.push_back(p);
            }
}

void OccupancyMap::GetVisualizePointCloud(sensor_msgs::PointCloud& point_cloud)
{
    point_cloud.header.frame_id = "camera_init";
    //point_cloud.header.frame_id = "uav1/t265_odom_frame";
    point_cloud.points.clear();
    

    for (int x = 19; x <= max_vec_[0]; ++x)
        for (int y = 19; y <= max_vec_[1]; ++y)
            for (int z = 19; z <= max_vec_[2]; ++z) 
            {
                Eigen::Vector3i tmp_voxel = {x, y, z};

                if (!Exist(Vox2Indx(tmp_voxel)))
                    continue;

                Eigen::Vector3d pos;
                Vox2Pos(tmp_voxel, pos);

                geometry_msgs::Point32 p;
                p.x = pos(0);
                p.y = pos(1);
                p.z = pos(2);
                
                point_cloud.points.push_back(p);
            }
}