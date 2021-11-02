#ifndef _OCCUPANCY_MAP_H_
#define _OCCUPANCY_MAP_H_

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <queue>
#include <Eigen/Eigen>

#include <ros/ros.h>

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
    int infinity_;
    int undefined_;
    double resolution_;
    Eigen::Vector3i max_vec_, min_vec_, last_max_vec_, last_min_vec_;


public:
    int grid_total_size_;

    OccupancyMap(Eigen::Vector3d origin, double resolution, Eigen::Vector3d map_size);
    ~OccupancyMap();

    // log odds function for occupancy update
    double Logit(const double& prob) const;
    bool Exist(const int& indx);

    // member methods for conversion between voxel, position and index
    void Vox2Pos(Eigen::Vector3i& voxel, Eigen::Vector3d& pos);
    void Pos2Vox(Eigen::Vector3d& pos, Eigen::Vector3i& voxel);
    int Vox2Indx(Eigen::Vector3i& voxel);
    Eigen::Vector3i Indx2Vox(int& indx);

    void SetParameters(double prob_hit, double prob_miss, 
                       double prob_min, double prob_max, double prob_occupancy);

    bool CheckUpdate();
    void UpdateOccupancy(bool global_map);

    // occupancy management
    int SetOccupancy(Eigen::Vector3d pos, int occ);
    int SetOccupancy(Eigen::Vector3i voxel, int occ);
    int GetOccupancy(Eigen::Vector3i voxel);
    int GetOccupancy(Eigen::Vector3d pos);


};




#endif