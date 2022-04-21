#include "object_filter.h"

void DynamicObject::SetObjBBox()
{

}

void DynamicObject::FilterObject(pcl::PCLPointCloud2& point_cloud)
{

}

void DynamicObject::MsgCallback(const ObjectPoseType::ConstPtr& obj_pose_msg)
{

}

int main(int argc, char** argv)
{           
    ros::init(argc, argv, "object_filter");
    ros::NodeHandle node("~");

    ObjectsFiltering objects_filter(node);
    

    DynamicObject ObjA;
    DynamicObject ObjB;
    DynamicObject ObjC;

    ros::Subscriber objA_sub = node.subscribe<ObjectPoseType>("/objA_pose", 10, &DynamicObject::MsgCallback, &ObjA);
    ros::Subscriber objB_sub = node.subscribe<ObjectPoseType>("/objB_pose", 10, &DynamicObject::MsgCallback, &ObjB);
    ros::Subscriber objC_sub = node.subscribe<ObjectPoseType>("/objC_pose", 10, &DynamicObject::MsgCallback, &ObjC);
    /*
    node.param<double>("objA_upper_x", ObjA.upperbound_x, 0.0);
    node.param<double>("objA_upper_y", ObjA.upperbound_y, 0.0);
    node.param<double>("objA_upper_y", ObjA.upperbound_z, 0.0);
    node.param<double>("objA_upper_y", ObjA.lowerbound_x, 0.0);
    node.param<double>("objA_upper_y", ObjA.lowerbound_y, 0.0);
    node.param<double>("objA_upper_y", ObjA.lowerbound_z, 0.0);
*/
    ros::spin();
    return 0;
}