#include "mapping.h"

int main(int argc, char **argv)
{        
    ros::init(argc, argv, "occupancy_map");
    ros::NodeHandle node("~");

    //Mapping<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> occupancy_map (node);
    Mapping<sensor_msgs::Image::ConstPtr, nav_msgs::Odometry::ConstPtr> occupancy_map (node);

    ros::spin();

    return 0;
}