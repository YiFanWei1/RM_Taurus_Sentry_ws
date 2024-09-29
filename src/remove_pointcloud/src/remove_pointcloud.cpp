#include "remove_pointcloud/remove_pointcloud.hpp"
namespace taurus
{
    Remove_pointcloud::Remove_pointcloud(const rclcpp::NodeOptions &options) : Node("Remove_pointcloud",options)
    {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        livox_lidar_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

        transformed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        
        client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(3000.0 / 1.0));
        auto period_ms_static_ = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));
        
        static_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    // 创建定时器，每秒调用一次定时器回调
        timer_ = this->create_wall_timer(period_ms, std::bind(&Remove_pointcloud::timer_callback, this));
        static_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        timer_static_ = this->create_wall_timer(period_ms_static_, std::bind(&Remove_pointcloud::timer_static_callback, this));
        if (!(rosInit() && paramInit()))
        {
            RCLCPP_ERROR(this->get_logger(), "Init Error");
            rclcpp::shutdown();
        }
    }
    Remove_pointcloud::~Remove_pointcloud()
    {

    }
    void Remove_pointcloud::timer_static_callback()
    {
        // std::cout << "pub_map_to_odom_static_tf" <<std::endl;
        static_map_to_odom_.header.stamp = this->get_clock()->now();

        static_map_to_odom_.child_frame_id = "odom";
        static_map_to_odom_.header.frame_id = "map";
        // static_broadcaster_->sendTransform(static_map_to_odom_);
    }
    void Remove_pointcloud::range_sensor_sub_cbk_(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
        {
            // geometry_msgs::msg::TransformStamped static_map_to_odom;
        static_map_to_odom_ = tf_buffer_->lookupTransform("map","odom",tf2::TimePointZero);
        //static
        // geometry_msgs::msg::TransformStamped static_map_to_odom_;
        // static_map_to_odom_.transform.translation.x = msg->pose.position.x;
        // static_map_to_odom_.transform.translation.y = msg->pose.position.y;
        // static_map_to_odom_.transform.translation.z = msg->pose.position.z;
        // static_map_to_odom_.transform.rotation.w    = msg->pose.orientation.w;
        // static_map_to_odom_.transform.rotation.x    = msg->pose.orientation.x;
        // static_map_to_odom_.transform.rotation.y    = msg->pose.orientation.y;
        // static_map_to_odom_.transform.rotation.z    = msg->pose.orientation.z;

        // RCLCPP_INFO(this->get_logger(),"static_map_to_odom_.transform.translation.x:" ,static_map_to_odom_.transform.translation.x);
        std::cout << "static_map_to_odom_.transform.translation.x:" << static_map_to_odom_.transform.translation.x <<std::endl;
        }
    void Remove_pointcloud::timer_callback()
    {
        // 等待服务可用
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "中断请求");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "等待服务变为可用");
        }

        // 发送请求
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        auto future = client_->async_send_request(request);

        // 等待响应
        // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        // rclcpp::FutureReturnCode::SUCCESS)
        // {
        // RCLCPP_ERROR(this->get_logger(), "服务调用失败");
        // }
    try {
        // Check if the future is completed
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Service call successful");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
        } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
    bool Remove_pointcloud::rosInit(){
        using std::placeholders::_1;
        //发布者
        removed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("removed_cloud_",10);
        map_pub            = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_map",1); 
        map_frame_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_frame_pointcloud",1); 
        processed_point_cloud_pub_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_point_cloud_",rclcpp::SensorDataQoS()); 
        //订阅者
        range_sensor_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("range_sensor_pose",rclcpp::QoS(rclcpp::KeepLast(1)),std::bind(&Remove_pointcloud::range_sensor_sub_cbk_,  this, _1));

        livox_lidar_sub_   = this->create_subscription<sensor_msgs::msg::PointCloud2>("livox/lidar/pointcloud",rclcpp::QoS(rclcpp::KeepLast(1)),std::bind(&Remove_pointcloud::livox_lidar_sub_cbk_,  this, _1));
        return true;
    }
    bool Remove_pointcloud::paramInit(){
        this->declare_parameter<double>("dis_diff", 0.1);rclcpp::Node::get_parameter("dis_diff", dis_diff_);

        return true;
    }
    bool Remove_pointcloud::pcdLoader(const std::string &filename){

        return true;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr Remove_pointcloud::loadPointcloudFromPcd(const std::string &filename){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloudBlob;
        pcl::io::loadPCDFile(filename, cloudBlob);
        pcl::fromPCLPointCloud2(cloudBlob, *cloud);
        pcl::toROSMsg(*cloud,pcd_map_);
        return cloud;
    }
    int a = 10;
    void Remove_pointcloud::livox_lidar_sub_cbk_(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        livox_lidar_stamp_ = msg->header.stamp;
        geometry_msgs::msg::TransformStamped trans;
        trans.header.frame_id = "map";
        trans.child_frame_id = "odom";
        trans.header.stamp = msg->header.stamp;
        trans.transform.translation.x = 0;
        trans.transform.translation.y = 0;
        trans.transform.translation.z = 0;
        trans.transform.rotation.w = 1;
        trans.transform.rotation.x = 0;
        trans.transform.rotation.y = 0;
        trans.transform.rotation.z = 0;
        static_broadcaster_->sendTransform(trans);
        pcl::fromROSMsg(*msg, *livox_lidar_pointcloud);
        if(is_first_frame_){
            is_first_frame_ = false;
            map_cloud_ = loadPointcloudFromPcd(filename);
            pcd_map_.header.frame_id = "map";
            pcd_map_.header.stamp = msg->header.stamp;
            map_pub->publish(pcd_map_);
            kdTree.setInputCloud(map_cloud_);
        }
        //读取livox_frame到map的tf
        geometry_msgs::msg::TransformStamped tf_livox_to_map;
        try{
            tf_livox_to_map = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
        }catch(tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
            return; 
        }
        
        // 应用变换
        //将livox_lidar_pointcloud进行旋转，矩阵A*点云
        pcl_ros::transformPointCloud(*livox_lidar_pointcloud, *transformed_cloud, tf_livox_to_map);
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*transformed_cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = livox_lidar_stamp_; //仿真
        // output.header.stamp = this->now();
        map_frame_pointcloud_pub->publish(output);

        comparePointClouds(transformed_cloud);

        //遍历map坐标系下点云A(sensor_msgs::msg::PointCloud2 output)的每一个点（用x，y）
        //首先获得这个点的x和y，然后找到对应点云B（sensor_msgs::msg::PointCloud2 taurus::Remove_pointcloud::pcd_map_）xy对应的点，
        //如果有，把z轴的数据作差，没有就继续比较下一个点，
        //结束后发布处理的点
        // RCLCPP_INFO(this->get_logger(),"livox_lidar_pointcloud->points.size(): %ld",livox_lidar_pointcloud->points.size());
    }
    void Remove_pointcloud::comparePointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b)
    {
        // RCLCPP_INFO(this->get_logger(),"comparePointClouds");
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto& cloud_b : cloud_b->points)
        {
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdTree.nearestKSearch(cloud_b, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
            float z_diff = cloud_b.z-map_cloud_->points[pointIdxNKNSearch[0]].z;
            if (z_diff >= dis_diff_)
            {
                // RCLCPP_INFO(this->get_logger(),"z_diff: %f",z_diff);
                // RCLCPP_INFO(this->get_logger(),"map_cloud_->points[pointIdxNKNSearch[0]].z: %f",map_cloud_->points[pointIdxNKNSearch[0]].z);
                // RCLCPP_INFO(this->get_logger(),"cloud_b.z: %f\n",cloud_b.z);
                result_cloud->points.push_back(pcl::PointXYZ(cloud_b.x, cloud_b.y, cloud_b.z));
            }
            }
        }
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*result_cloud, output);
        output.header.frame_id = "map";
        output.header.stamp = livox_lidar_stamp_; //仿真
        // output.header.stamp = this->now();
        processed_point_cloud_pub_->publish(output);
    }
} // namespace taurus
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(taurus::Remove_pointcloud);