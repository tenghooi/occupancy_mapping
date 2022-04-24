#include "object_filter.h"

void SetNodeParameters(const ros::NodeHandle& node,
                       DynamicObject& objA,
                       DynamicObject& objB,
                       DynamicObject& objC)
{   
    double objA_min_x, objA_min_y, objA_min_z;
    double objA_max_x, objA_max_y, objA_max_z;
    node.param<double>("objA_min_x", objA_min_x, 0.0);
    node.param<double>("objA_min_x", objA_min_y, 0.0);
    node.param<double>("objA_min_x", objA_min_z, 0.0);
    node.param<double>("objA_min_x", objA_max_x, 0.0);
    node.param<double>("objA_min_x", objA_max_y, 0.0);
    node.param<double>("objA_min_x", objA_max_z, 0.0);
    objA.SetObjBBox(objA_min_x, objA_min_y, objA_min_z, objA_max_x, objA_max_y, objA_max_z);

    double objB_min_x, objB_min_y, objB_min_z;
    double objB_max_x, objB_max_y, objB_max_z;
    node.param<double>("objB_min_x", objB_min_x, 0.0);
    node.param<double>("objB_min_x", objB_min_y, 0.0);
    node.param<double>("objB_min_x", objB_min_z, 0.0);
    node.param<double>("objB_min_x", objB_max_x, 0.0);
    node.param<double>("objB_min_x", objB_max_y, 0.0);
    node.param<double>("objB_min_x", objB_max_z, 0.0);
    objB.SetObjBBox(objB_min_x, objB_min_y, objB_min_z, objB_max_x, objB_max_y, objB_max_z);

    double objC_min_x, objC_min_y, objC_min_z;
    double objC_max_x, objC_max_y, objC_max_z;
    node.param<double>("objC_min_x", objC_min_x, 0.0);
    node.param<double>("objC_min_x", objC_min_y, 0.0);
    node.param<double>("objC_min_x", objC_min_z, 0.0);
    node.param<double>("objC_min_x", objC_max_x, 0.0);
    node.param<double>("objC_min_x", objC_max_y, 0.0);
    node.param<double>("objC_min_x", objC_max_z, 0.0);
    objC.SetObjBBox(objC_min_x, objC_min_y, objC_min_z, objC_max_x, objC_max_y, objC_max_z);    

}

int main(int argc, char** argv)
{           
    ros::init(argc, argv, "object_filter");
    ros::NodeHandle node("~");
    
    ObjectsFiltering objects_filter(node);
    
    ros::spin();
    return 0;
}