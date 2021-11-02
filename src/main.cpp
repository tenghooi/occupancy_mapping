#include "mapping.h"

int main(int argc, char **argv)
{        
    ros::init(argc, argv, "occupancy_map");
    ros::NodeHandle node("~");

    Mapping<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> occupancy_map (node);

    ros::spin();

    return 0;
}