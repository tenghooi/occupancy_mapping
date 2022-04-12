#ifndef OCCUPANCY_MAP_H_
#define OCCUPANCY_MAP_H_

#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <cmath>
#include <queue>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "parameters.h"

struct QueueElement
{
    Eigen::Vector3i point_;
    double distance_;
    bool operator<(const QueueElement& element) const 
    {
        return distance_ > element.distance_;
    }
};

class OccupancyMap
{
private:
    // member attributes for occupancy update
    double logit_hit_;
    double logit_miss_;
    double min_logit_;
    double max_logit_;
    double occupancy_threshold_logit_;
    // map sizes
    Eigen::Vector3d map_size_;
    Eigen::Vector3d min_range_;
    Eigen::Vector3d max_range_;
    Eigen::Vector3i grid_size_;
    int grid_size_yz_;

    std::vector<double> occupancy_buffer_; // 0 is free, 1 is occupied
    std::vector<double> distance_buffer_;
    std::vector<int> num_hit_;
    std::vector<int> num_miss_;

    std::queue<QueueElement> occupancy_queue_;

    // map properties
    Eigen::Vector3d origin_;
    std::string frame_id_;
    int infinity_;
    int undefined_;
    double resolution_;
    Eigen::Vector3i max_vec_, min_vec_, last_max_vec_, last_min_vec_;

    // boundary for visualizing/publishing map point cloud
    Eigen::Vector3i lower_bound, upper_bound;

public:
    int grid_total_size_;

    OccupancyMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size);
    ~OccupancyMap(){};

    // log odds function for occupancy update
    double Logit(const double& prob) const;
    bool Exist(const int& indx);

    // member methods for conversion between voxel, position and index
    void Vox2Pos(Eigen::Vector3i& voxel, Eigen::Vector3d& pos);
    void Pos2Vox(Eigen::Vector3d& pos, Eigen::Vector3i& voxel);
    int Vox2Indx(Eigen::Vector3i& voxel);
    Eigen::Vector3i Indx2Vox(int& indx);

    void SetParameters(const double& prob_hit, const double& prob_miss, 
                       const double& prob_min, const double& prob_max, const double& prob_occupancy);

    bool CheckUpdate();
    void UpdateOccupancy(bool global_map);

    // occupancy management
    int SetOccupancy(Eigen::Vector3d pos, int occ);
    int SetOccupancy(Eigen::Vector3i voxel, int occ);
    int GetOccupancy(Eigen::Vector3i voxel);
    int GetOccupancy(Eigen::Vector3d pos);

    void SetOriginalRange();

    void SetVisualizationMargin(const Eigen::Vector3d& vis_min_margin, 
                                const Eigen::Vector3d& vis_max_margin);
    void GetPointCloud(sensor_msgs::PointCloud& point_cloud, const std::string& map_frame_id);
    void GetVisualizePointCloud(sensor_msgs::PointCloud& point_cloud, 
                                const std::string& map_frame_id);

};




#endif