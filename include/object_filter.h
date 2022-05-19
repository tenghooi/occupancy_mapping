#ifndef _OBJECT_FILTER_H_
#define _OBJECT_FILTER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <memory>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/min_cut_segmentation.h>

typedef geometry_msgs::PoseWithCovarianceStamped ObjectPoseType;
typedef sensor_msgs::PointCloud2 PointCloudType;

/**************************
 DynamicObject class to 
 set object BBox and filtering
 **************************/
class DynamicObject
{
private:
    Eigen::Vector4f max_vec_;
    Eigen::Vector4f min_vec_;

public:
    DynamicObject();
    ~DynamicObject();

    Eigen::Vector4f GetMaxVec() const;
    Eigen::Vector4f GetMinVec() const;

    void SetMinMaxVec(double& min_x, double& min_y, double& min_z,
                   double& max_x, double& max_y, double& max_z);

    void FilterObject(pcl::PCLPointCloud2Ptr point_cloud, 
                      pcl::PCLPointCloud2& filtered_cloud,
                      const Eigen::Vector3f& sync_pose);
};

DynamicObject::DynamicObject(){};
DynamicObject::~DynamicObject(){};

Eigen::Vector4f DynamicObject::GetMaxVec() const
{
    return max_vec_;
}

Eigen::Vector4f DynamicObject::GetMinVec() const
{
    return min_vec_;
}

void DynamicObject::SetMinMaxVec(double& min_x, double& min_y, double& min_z,
                                 double& max_x, double& max_y, double& max_z)
{
    min_vec_ << min_x, min_y, min_z, 1.0;
    max_vec_ << max_x, max_y, max_z, 1.0;
}

void DynamicObject::FilterObject(pcl::PCLPointCloud2Ptr point_cloud, 
                                 pcl::PCLPointCloud2& filtered_cloud,
                                 const Eigen::Vector3f& sync_pose)
{
    pcl::CropBox<pcl::PCLPointCloud2> box_filter;
    box_filter.setInputCloud(point_cloud);
    box_filter.setMax(max_vec_);
    box_filter.setMin(min_vec_);
    box_filter.setTranslation(sync_pose);
    box_filter.setNegative(true);

    box_filter.filter(filtered_cloud);
    std::cout << "::" << filtered_cloud.width << std::endl;
}

/******************************
 Setting node parameters for
 node and DynamicObject objects
 *****************************/
void SetNodeParameters(const ros::NodeHandle& node,
                       Eigen::Vector4f& raw_cloud_max_vec,
                       Eigen::Vector4f& raw_cloud_min_vec,
                       DynamicObject& self,
                       DynamicObject& objA,
                       DynamicObject& objB,
                       DynamicObject& objC);

/**************************
 ObjectsFiltering class to 
 encapsulate and process
 objects and filtering.
 **************************/
class ObjectsFiltering
{
private:
    DynamicObject self_;
    DynamicObject objA_;
    DynamicObject objB_;
    DynamicObject objC_;

    Eigen::Vector4f raw_cloud_max_vec_;
    Eigen::Vector4f raw_cloud_min_vec_;

    std::queue<ObjectPoseType>objA_queue_;
    std::queue<ObjectPoseType>objB_queue_;
    std::queue<ObjectPoseType>objC_queue_;

    ros::Publisher filtered_cloud_pub_;

    ros::Subscriber point_cloud_sub_;
    ros::Subscriber objA_pose_sub_;
    ros::Subscriber objB_pose_sub_;
    ros::Subscriber objC_pose_sub_;

public:
    ObjectsFiltering(ros::NodeHandle node);
    ~ObjectsFiltering();

    void CropCloud(pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& output_cloud);
    void RemoveSelf(pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& output_cloud);
    void FilterObjects(pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& filtered_cloud);
    
    void CloudCallBack(const PointCloudType::ConstPtr& point_cloud_msg);
    void ObjAPoseCallBack(const ObjectPoseType::ConstPtr& objA_pose_msg);
    void ObjBPoseCallBack(const ObjectPoseType::ConstPtr& objB_pose_msg);
    void ObjCPoseCallBack(const ObjectPoseType::ConstPtr& objC_pose_msg);
};
                       
ObjectsFiltering::ObjectsFiltering(ros::NodeHandle node)
{
    SetNodeParameters(node, raw_cloud_max_vec_, raw_cloud_min_vec_,
                      self_, objA_, objB_, objC_);

    point_cloud_sub_ = node.subscribe("raw_point_cloud", 10, &ObjectsFiltering::CloudCallBack, this);
    objA_pose_sub_ = node.subscribe("objA_pose", 20, &ObjectsFiltering::ObjAPoseCallBack, this);
    objB_pose_sub_ = node.subscribe("objB_pose", 20, &ObjectsFiltering::ObjBPoseCallBack, this);
    objC_pose_sub_ = node.subscribe("objC_pose", 20, &ObjectsFiltering::ObjCPoseCallBack, this);

    filtered_cloud_pub_ = node.advertise<PointCloudType>("filtered_point_cloud", 10);
}

ObjectsFiltering::~ObjectsFiltering(){}

void ObjectsFiltering::CropCloud(pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& output_cloud)
{
    pcl::CropBox<pcl::PCLPointCloud2> box_filter;

    box_filter.setInputCloud(input_cloud);
    box_filter.setMax(raw_cloud_max_vec_);
    box_filter.setMin(raw_cloud_min_vec_);

    box_filter.filter(output_cloud);
    std::cout << "Cropped raw point cloud size: " << output_cloud.width << std::endl;
}

void ObjectsFiltering::RemoveSelf(pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& output_cloud)
{
    //TODO
}


void ObjectsFiltering::FilterObjects(pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& filtered_cloud)
{
    ros::Duration time_tolerance(1e-3); // 1ms
    //ros::Time point_cloud_time = pcl_conversions::fromPCL(point_cloud.header.stamp);
    ros::Time point_cloud_time = ros::Time::now();
    ROS_INFO("Point cloud msg time: %lf", point_cloud_time.toSec());

    Eigen::Vector3f sync_objA_pos;
    Eigen::Vector3f sync_objB_pos;
    Eigen::Vector3f sync_objC_pos;

    while(objA_queue_.size() > 1 &&
          objA_queue_.front().header.stamp <= point_cloud_time + time_tolerance)
    {
        sync_objA_pos << objA_queue_.front().pose.pose.position.x,
                     objA_queue_.front().pose.pose.position.y,
                     objA_queue_.front().pose.pose.position.z;

        objA_queue_.pop();
    }

    while(objB_queue_.size() > 1 &&
          objB_queue_.front().header.stamp <= point_cloud_time + time_tolerance)
    {
        sync_objB_pos << objB_queue_.front().pose.pose.position.x,
                     objB_queue_.front().pose.pose.position.y,
                     objB_queue_.front().pose.pose.position.z;

        objB_queue_.pop();
    }

    while(objC_queue_.size() > 1 &&
          objC_queue_.front().header.stamp <= point_cloud_time + time_tolerance)
    {
        sync_objC_pos << objC_queue_.front().pose.pose.position.x,
                     objC_queue_.front().pose.pose.position.y,
                     objC_queue_.front().pose.pose.position.z;

        objC_queue_.pop();
    }
    
    objA_.FilterObject(input_cloud, filtered_cloud, sync_objA_pos);
    // *point_cloud = filtered_cloud;
    // filtered_cloud.data.clear();
    // objB_.FilterObject(point_cloud, filtered_cloud, sync_objB_pos);
    
    
}

void ObjectsFiltering::CloudCallBack(const PointCloudType::ConstPtr& point_cloud_msg)
{
    pcl::PCLPointCloud2* pcl_point_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2Ptr pcl_point_cloud_ptr(pcl_point_cloud);
    pcl::PCLPointCloud2 filtered_pcl_point_cloud;

    pcl_conversions::toPCL(*point_cloud_msg, *pcl_point_cloud);
    FilterObjects(pcl_point_cloud_ptr, filtered_pcl_point_cloud);

    sensor_msgs::PointCloud2 filtered_ros_point_cloud;
    pcl_conversions::moveFromPCL(filtered_pcl_point_cloud, filtered_ros_point_cloud);

    filtered_cloud_pub_.publish(filtered_ros_point_cloud);
}

void ObjectsFiltering::ObjAPoseCallBack(const ObjectPoseType::ConstPtr& objA_pose_msg)
{
    ObjectPoseType objA_pose;
    objA_pose.header = objA_pose_msg->header;
    objA_pose.pose = objA_pose_msg->pose;

    objA_queue_.push(objA_pose);
}

void ObjectsFiltering::ObjBPoseCallBack(const ObjectPoseType::ConstPtr& objB_pose_msg)
{
    ObjectPoseType objB_pose;
    objB_pose.header = objB_pose_msg->header;
    objB_pose.pose = objB_pose_msg->pose;

    objA_queue_.push(objB_pose);
}

void ObjectsFiltering::ObjCPoseCallBack(const ObjectPoseType::ConstPtr& objC_pose_msg)
{
    ObjectPoseType objC_pose;
    objC_pose.header = objC_pose_msg->header;
    objC_pose.pose = objC_pose_msg->pose;

    objC_queue_.push(objC_pose);
}


#endif //_OBJECT_FILTER_H_