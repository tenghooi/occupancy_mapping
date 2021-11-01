#ifndef _OCCUPANCY_MAP_H_
#define _OCCUPANCY_MAP_H_

#include <vector>
#include <cmath>
#include <queue>
#include <Eigen/Eigen>

// Parameters for occupancy map
struct Parameters
{   
    // resolution (in meters)
    double resolution;
    // boundary and size of map in vector form
    Eigen::Vector3d bottom_left;
    Eigen::Vector3d upper_right;
    Eigen::Vector3d map_size;
    // probabilty parameters for probabilistic occupancy map
    double prob_hit;
    double prob_miss;
    double prob_occupancy;
    double prob_min;
    double prob_max;
    // intrinsic parameters of camera
    double center_x;
    double center_y;
    double focal_length_x;
    double focal_length_y;
    // transformation from camera to body frame
    Eigen::Matrix4d T_Body_Camera;
    Eigen::Matrix4d T_D_B;
};


class OccupancyMap
{
private:
    Parameters parameters_;
    // member attributes for occupancy update
    double logit_hit_;
    double logit_miss_;
    double min_logit_;
    double max_logit_;
    double occupancy_threshold_logit_;

    Eigen::Vector3d map_size_;
    Eigen::Vector3d min_range_;
    Eigen::Vector3d max_range_;
    Eigen::Vector3i grid_size_;
    int grid_size_yz_;

    std::vector<double> occupancy_buffer_; // 0 is free, 1 is occupied
    std::vector<double> distance_buffer_;
    std::vector<int> num_hit_;
    std::vector<int> num_miss_;

    std::queue<int> occupancy_queue_;

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
    void Pos2Vox(Eigen::Vector3d& pos, Eigen::Vector3i& vox);
    int Vox2Indx(Eigen::Vector3i& voxel);
    Eigen::Vector3i Indx2Vox(int& indx);

    void SetParameters(double prob_hit, double prob_miss, \
                       double prob_min, double prob_max, double prob_occupancy);

    bool UpdateOccupancy(bool global_map);

    // occupancy management
    int SetOccupancy(Eigen::Vector3d pos, int occ);
    int SetOccupancy(Eigen::Vector3i voxel, int occ);
    int GetOccupancy(Eigen::Vector3i pos_id);
    int GetOccupancy(Eigen::Vector3d pos);


};




#endif