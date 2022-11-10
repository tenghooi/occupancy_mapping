#include "object_filter.h"

void SetNodeParameters(const ros::NodeHandle& node,
                       Eigen::Vector4f& raw_cloud_max_vec,
                       Eigen::Vector4f& raw_cloud_min_vec,
                       DynamicObject& self,
                       DynamicObject& objA,
                       DynamicObject& objB,
                       DynamicObject& objC)
{   
    // To set box vectors for downsizing raw point cloud
    double cloud_min_x, cloud_min_y, cloud_min_z;
    double cloud_max_x, cloud_max_y, cloud_max_z;
    node.param<double>("cloud_min_x", cloud_min_x, 0.0);
    node.param<double>("cloud_min_y", cloud_min_y, 0.0);
    node.param<double>("cloud_min_z", cloud_min_z, 0.0);
    node.param<double>("cloud_max_x", cloud_max_x, 0.0);
    node.param<double>("cloud_max_y", cloud_max_y, 0.0);
    node.param<double>("cloud_max_z", cloud_max_z, 0.0);
    raw_cloud_max_vec << cloud_max_x, cloud_max_y, cloud_max_z, 1.0;
    raw_cloud_min_vec << cloud_min_x, cloud_min_y, cloud_min_z, 1.0;

    // To set box vectors for cropping out own self in point cloud
    double self_min_x, self_min_y, self_min_z;
    double self_max_x, self_max_y, self_max_z;
    node.param<double>("self_min_x", self_min_x, -0.5);
    node.param<double>("self_min_y", self_min_y, -0.5);
    node.param<double>("self_min_z", self_min_z, -0.5);
    node.param<double>("self_max_x", self_max_x, 0.5);
    node.param<double>("self_max_y", self_max_y, 0.5);
    node.param<double>("self_max_z", self_max_z, 0.5);
    self.SetMinMaxVec(self_min_x, self_min_y, self_min_z, self_max_x, self_max_y, self_max_z);

    double objA_min_x, objA_min_y, objA_min_z;
    double objA_max_x, objA_max_y, objA_max_z;
    node.param<double>("objA_min_x", objA_min_x, 0.0);
    node.param<double>("objA_min_y", objA_min_y, 0.0);
    node.param<double>("objA_min_z", objA_min_z, 0.0);
    node.param<double>("objA_max_x", objA_max_x, 0.0);
    node.param<double>("objA_max_y", objA_max_y, 0.0);
    node.param<double>("objA_max_z", objA_max_z, 0.0);
    objA.SetMinMaxVec(objA_min_x, objA_min_y, objA_min_z, objA_max_x, objA_max_y, objA_max_z);

    double objB_min_x, objB_min_y, objB_min_z;
    double objB_max_x, objB_max_y, objB_max_z;
    node.param<double>("objB_min_x", objB_min_x, 0.0);
    node.param<double>("objB_min_y", objB_min_y, 0.0);
    node.param<double>("objB_min_z", objB_min_z, 0.0);
    node.param<double>("objB_max_x", objB_max_x, 0.0);
    node.param<double>("objB_max_y", objB_max_y, 0.0);
    node.param<double>("objB_max_z", objB_max_z, 0.0);
    objB.SetMinMaxVec(objB_min_x, objB_min_y, objB_min_z, objB_max_x, objB_max_y, objB_max_z);

    double objC_min_x, objC_min_y, objC_min_z;
    double objC_max_x, objC_max_y, objC_max_z;
    node.param<double>("objC_min_x", objC_min_x, 0.0);
    node.param<double>("objC_min_y", objC_min_y, 0.0);
    node.param<double>("objC_min_z", objC_min_z, 0.0);
    node.param<double>("objC_max_x", objC_max_x, 0.0);
    node.param<double>("objC_max_y", objC_max_y, 0.0);
    node.param<double>("objC_max_z", objC_max_z, 0.0);
    objC.SetMinMaxVec(objC_min_x, objC_min_y, objC_min_z, objC_max_x, objC_max_y, objC_max_z);    

}

int main(int argc, char** argv)
{           
    ros::init(argc, argv, "object_filter");
    ros::NodeHandle node("~");
    
    ObjectsFiltering objects_filter(node);
    
    ros::spin();
    return 0;
}