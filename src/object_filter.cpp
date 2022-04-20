#include "object_filter.h"

void DynamicObject::SetObjBBox()
{

}

void DynamicObject::FilterObject(pcl::PCLPointCloud2& point_cloud)
{

}

void PCLCallBack(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
    pcl::PCLPointCloud2 pcl_point_cloud;
    pcl_conversions::toPCL(*point_cloud_msg, pcl_point_cloud);
}

int main(int argc, char** argv)
{           
    ros::init(argc, argv, "object_filter");
    ros::NodeHandle node;

    ros::Subscriber point_cloud_sub = node.subscribe("point_cloud", 10, PCLCallBack);
    ros::Publisher filtered_point_cloud_pub = node.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 10);

    DynamicObject ObjA;
    DynamicObject ObjB;
    DynamicObject ObjC;

    ros::Subscriber objA_pose_sub = node.subscribe<geometry_msgs::PoseWithCovarianceStamped>
                                                  ("/objA_pose", 10, &DynamicObject::MsgCallback);
    
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