#include "object_filter.h"
                   
int main(int argc, char** argv)
{           
    ros::init(argc, argv, "object_filter");
    ros::NodeHandle node;

    DynamicObject ObjA;
    
    node.param<double>("objA_upper_x", ObjA.upperbound_x, 0.0);
    node.param<double>("objA_upper_y", ObjA.upperbound_y, 0.0);
    node.param<double>("objA_upper_y", ObjA.upperbound_z, 0.0);
    node.param<double>("objA_upper_y", ObjA.lowerbound_x, 0.0);
    node.param<double>("objA_upper_y", ObjA.lowerbound_y, 0.0);
    node.param<double>("objA_upper_y", ObjA.lowerbound_z, 0.0);


    return 0;
}