#ifndef _OCCUPANCY_MAP_PARAMETERS_H_
#define _OCCUPANCY_MAP_PARAMETERS_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <string>

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
    // parameters for ray casting
    double min_ray_length;
    double max_ray_length;
    // intrinsic parameters of camera
    double center_x;
    double center_y;
    double focal_length_x;
    double focal_length_y;
    // depth filter for depth images
    bool use_depth_filter;
    double filter_max_depth;
    double filter_min_depth;
    double filter_tolerance;
    int depth_filter_margin;
    // transformation from camera to body frame
    Eigen::Matrix4d T_Body_Camera;
    Eigen::Matrix4d T_D_B;
    // update frequency
    double update_occupancy_every_n_sec;
    // visualization frequency
    int visualize_every_n_updates;
    // global
    bool global_update;
    bool global_map;
    bool global_vis;

    // map margin for visualization
    Eigen::Vector3d vis_min_margin, vis_max_margin;

    // publishing sensor_msgs::PointCloud header frame id
    std::string map_frame_id;
};

void SetNodeParameters(Parameters& parameters, const ros::NodeHandle& node);

#endif