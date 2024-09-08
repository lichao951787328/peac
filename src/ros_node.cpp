#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <peac/PEAC_plane_detection.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
plane_detection pd;

void detect_callback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);
    pd.detect(pcl_cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_detection");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/depth_camera/depth_camera/points", 1, detect_callback);
    pd.initial("/home/lichao/catkin_pathplanning/src/peac/config/plane_fitter_pcd.ini");
    ros::spin();
    return 0;
}
