#include "truncated_map.h"

TruncatedMap::TruncatedMap(ros::NodeHandle nh)
{
    // Box filter parameters
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    nh.param<double>("min_x", min_x, -10.0);
    nh.param<double>("min_y", min_y, -10.0);
    nh.param<double>("min_z", min_z, -2.0);
    nh.param<double>("max_x", max_x, 10.0);
    nh.param<double>("max_y", max_y, 10.0);
    nh.param<double>("max_z", max_z, 3.0);

    min_point_vec_ << min_x, min_y, min_z, 1.0;
    max_point_vec_ << max_x, max_y, max_z, 1.0;

    cloud_sub_ = nh.subscribe("point_cloud", 10, &TruncatedMap::cloudCallback, this);
    odom_sub_ = nh.subscribe("odometry", 10, &TruncatedMap::odomCallback, this);

    filtered_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("truncated_map", 2);
}

TruncatedMap::~TruncatedMap(){}

#if SUBSCRIBE_TO_POINTCLOUD2
void TruncatedMap::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    pcl::PCLPointCloud2Ptr pcl_cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 filtered_cloud;

    pcl_conversions::toPCL(*cloud_msg, *pcl_cloud);

    pcl::CropBox<pcl::PCLPointCloud2> box_filter;
    box_filter.setInputCloud(pcl_cloud);
    box_filter.setMin(min_point_vec_);
    box_filter.setMax(max_point_vec_);

    box_filter.filter(filtered_cloud);

    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(filtered_cloud, output);
    output.header = cloud_msg->header;
    filtered_cloud_pub_.publish(output);
}
#else
void TruncatedMap::cloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
{   
    sensor_msgs::PointCloud2 ros_pointcloud2_msg;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, ros_pointcloud2_msg);

    pcl::PCLPointCloud2Ptr pcl_cloud_ptr(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 truncated_cloud;

    pcl_conversions::moveToPCL(ros_pointcloud2_msg, *pcl_cloud_ptr);
    CropOutVolume(pcl_cloud_ptr, truncated_cloud);

    sensor_msgs::PointCloud2 ros_truncated_cloud;
    pcl_conversions::moveFromPCL(truncated_cloud, ros_truncated_cloud);

    filtered_cloud_pub_.publish(ros_truncated_cloud);
}
#endif

void TruncatedMap::odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    nav_msgs::Odometry current_odom;
    current_odom = *odom_msg;

    odom_queue_.push(current_odom);
}

void TruncatedMap::SyncOdom(const pcl::PCLPointCloud2Ptr input_cloud, Eigen::Vector3f& sync_position)
{
    if (odom_queue_.empty()) return;

    ros::Duration time_tolerance(5e-3); // 2ms

    // Get the timestamp of the point cloud
    ros::Time cloud_time;
    cloud_time.sec = input_cloud->header.stamp / 1000000;
    cloud_time.nsec = input_cloud->header.stamp * 1000;

    // Find the closest odometry message in the queue
    while (!odom_queue_.empty())
    {
        nav_msgs::Odometry odom_msg = odom_queue_.front();
        ros::Time odom_time = odom_msg.header.stamp;
        if (odom_time <= cloud_time + time_tolerance && odom_time >= cloud_time - time_tolerance)
        {
            sync_position << odom_msg.pose.pose.position.x,
                             odom_msg.pose.pose.position.y,
                             odom_msg.pose.pose.position.z;
            odom_queue_.pop();
            return;
        }
        else
        {
            odom_queue_.pop();
            // TODO might not work well
            if (odom_time >= cloud_time + time_tolerance*10 && odom_time <= cloud_time - time_tolerance*10)
            {
                std::cout << "No matching odometry message found for the point cloud timestamp." << std::endl;
                return;
            }

        }
    }

    // If no matching odometry message is found, use the last known position
    // nav_msgs::Odometry last_odom = odom_queue_.back();
    // sync_position << last_odom.pose.pose.position.x,
    //                  last_odom.pose.pose.position.y,
    //                  last_odom.pose.pose.position.z;
}

void TruncatedMap::CropOutVolume(pcl::PCLPointCloud2Ptr input_cloud, pcl::PCLPointCloud2& output_cloud)
{
    pcl::CropBox<pcl::PCLPointCloud2> box_filter;
    box_filter.setInputCloud(input_cloud);
    box_filter.setMin(min_point_vec_);
    box_filter.setMax(max_point_vec_);

    Eigen::Vector3f sync_position;
    SyncOdom(input_cloud, sync_position);
    box_filter.setTranslation(sync_position);

    box_filter.filter(output_cloud);
    std::cout << "Truncated point cloud size: " << output_cloud.width << std::endl;
}

int main(int argc, char** argv)
{           
    ros::init(argc, argv, "truncated_map");
    ros::NodeHandle node("~");
    
    TruncatedMap truncated_map(node);
    
    ros::spin();
    return 0;
}