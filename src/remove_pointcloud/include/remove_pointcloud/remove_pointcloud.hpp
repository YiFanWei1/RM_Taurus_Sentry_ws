#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <pcl/io/pcd_io.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h> //toRosMsg fromRosMsg
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl_ros/transforms.hpp>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <std_srvs/srv/empty.hpp>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
namespace taurus
{
class Remove_pointcloud : public rclcpp::Node{
public:
Remove_pointcloud(const rclcpp::NodeOptions &options);
~Remove_pointcloud();
private:
bool rosInit();
bool paramInit();
bool pcdLoader(const std::string &filename);
pcl::PointCloud<pcl::PointXYZ>::Ptr loadPointcloudFromPcd(const std::string &filemname);

//发布者
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr removed_cloud_pub_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_frame_pointcloud_pub;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_point_cloud_pub_;

//订阅者
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr livox_lidar_sub_;
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr range_sensor_sub_;

// tf2相关
std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> transform_listener;
void livox_lidar_sub_cbk_(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

std::unique_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;
//data
pcl::PointCloud<pcl::PointXYZ>::Ptr livox_lidar_pointcloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
std::string filename = "/home/wei/github_code/PCD/downsampled_rm_pt.pcd";
sensor_msgs::msg::PointCloud2 pcd_map_;
rclcpp::Time livox_lidar_stamp_;
bool is_first_frame_ = true;
pcl::KdTreeFLANN<pcl::PointXYZ> kdTree;
void comparePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
double dis_diff_ = 0.3;
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::TimerBase::SharedPtr timer_static_;
void timer_callback();
void timer_static_callback();
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;
geometry_msgs::msg::TransformStamped static_map_to_odom_;
void range_sensor_sub_cbk_(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
    // std::unique_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;

};   
} // namespace tartus

