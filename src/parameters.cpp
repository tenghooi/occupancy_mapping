#include "parameters.h"

void SetNodeParameters(Parameters& parameters, const ros::NodeHandle& node)
{
    node.param<double>("resolution", parameters.resolution, 0.1);
    
    double bottom_left_x, bottom_left_y, bottom_left_z;
    double upper_right_x, upper_right_y, upper_right_z;
    node.param<double>("bottom_left_x", bottom_left_x, -20.0);
    node.param<double>("bottom_left_y", bottom_left_y, -20.0);
    node.param<double>("bottom_left_z", bottom_left_z, -5.0);
    node.param<double>("upper_right_x", upper_right_x, 20.0);
    node.param<double>("upper_right_y", upper_right_y, 20.0);
    node.param<double>("upper_right_z", upper_right_z, 10.0);

    parameters.bottom_left << bottom_left_x, bottom_left_y, bottom_left_z;
    parameters.upper_right << upper_right_x, upper_right_y, upper_right_z;
    parameters.map_size = parameters.upper_right - parameters.bottom_left;

    node.param<double>("prob_hit", parameters.prob_hit, 0.70);
    node.param<double>("prob_miss", parameters.prob_miss, 0.30);
    node.param<double>("prob_occupancy", parameters.prob_occupancy, 0.8);
    node.param<double>("prob_min", parameters.prob_min, 0.1);
    node.param<double>("prob_max", parameters.prob_max, 0.95);

    node.param<double>("min_ray_length", parameters.min_ray_length, 0.3);
    node.param<double>("max_ray_length", parameters.max_ray_length, 5.0);

    node.param<double>("center_x", parameters.center_x, 322.477357419);
    node.param<double>("center_y", parameters.center_y, 237.076346481);
    node.param<double>("focal_length_x", parameters.focal_length_x, 384.458089392);
    node.param<double>("focal_length_y", parameters.focal_length_y, 383.982755697);

    node.param<bool>("use_depth_filter", parameters.use_depth_filter, false);
    node.param<double>("filter_max_depth", parameters.filter_max_depth, 10.0);
    node.param<double>("filter_min_depth", parameters.filter_min_depth, 0.1);
    node.param<double>("filter_tolerance", parameters.filter_tolerance, 0.1);
    node.param<int>("depth_filter_margin", parameters.depth_filter_margin, 0);

    parameters.T_Body_Camera << 1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;
    /*
    parameters.T_D_B << 0.971048, -0.120915, 0.206023, 0.00114049,
                         0.15701, 0.973037, -0.168959, 0.0450936,
                        -0.180038, 0.196415, 0.96385, 0.0430765,
                         0.0, 0.0, 0.0, 1.0;
    */
    parameters.T_D_B << 0, 0, 1, 0,
                        -1, 0, 0, 0,
                        0, -1, 0, 0,
                        0, 0, 0, 1;

    node.param<double>("update_occupancy_every_n_sec", parameters.update_occupancy_every_n_sec, 0.1);

    node.param<int>("visualize_every_n_updates", parameters.visualize_every_n_updates, 1);

    node.param<bool>("global_update", parameters.global_update, true);
    node.param<bool>("global_map", parameters.global_map, true);
    node.param<bool>("global_vis", parameters.global_vis, true);

    node.param<double>("vis_min_margin_x", parameters.vis_min_margin_x, 0.0);
    node.param<double>("vis_min_margin_y", parameters.vis_min_margin_y, 0.0);
    node.param<double>("vis_min_margin_z", parameters.vis_min_margin_z, 0.0);
    node.param<double>("vis_max_margin_x", parameters.vis_max_margin_x, 0.0);
    node.param<double>("vis_max_margin_y", parameters.vis_max_margin_y, 0.0);
    node.param<double>("vis_max_margin_z", parameters.vis_max_margin_z, 0.0);

    node.param<std::string>("map_frame_id", parameters.map_frame_id, "camera_init");
   
}