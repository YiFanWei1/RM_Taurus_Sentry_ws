#include "remove_pointcloud/remove_pointcloud.hpp"
namespace taurus
{
    rclcpp::Rate sleep_rate(600);
    RemovePointcloud::RemovePointcloud(const rclcpp::NodeOptions &options) : Node("RemovePointcloud", options)
    {
        RCLCPP_INFO(this->get_logger(), "RemovePointcloud::RemovePointcloud");
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        livox_lidar_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        previous_livox_lidar_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        transformed_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        client_ = this->create_client<nav2_msgs::srv::ClearEntireCostmap>("global_costmap/clear_entirely_global_costmap");
        if (!(RosInit() && ParamInit()))
        {
            RCLCPP_ERROR(this->get_logger(), "Init Error");
            rclcpp::shutdown();
        }
        auto callable_tf_check_ = [this]()
        { tfCheckThread(); };
        tf_check_thread_ = std::thread(callable_tf_check_);
    }
    RemovePointcloud::~RemovePointcloud() {}
    bool RemovePointcloud::RosInit()
    {
        using std::placeholders::_1;
        // pub
        removed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("removed_cloud_", 10);
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_map", 1);
        map_frame_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_frame_pointcloud", 1);
        processed_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_point_cloud_", 1);
        cough_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cough_point_cloud_", 1); // cough_point_cloud_
        // sub
        livox_lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar/pointcloud", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&RemovePointcloud::LivoxLidarSubCallback, this, _1));
        return true;
    }
    bool RemovePointcloud::ParamInit()
    {
        this->declare_parameter<double>("dis_diff", 0.2);
        rclcpp::Node::get_parameter("dis_diff", dis_diff_);
        this->declare_parameter<std::string>("filename", "/home/auto/code/PCD/tessst_13.pcd");
        rclcpp::Node::get_parameter("filename", filename_);
        RCLCPP_INFO(this->get_logger(), "filename_:%s", filename_.c_str());
        return true;
    }
    void RemovePointcloud::tfCheckThread()
    {
        while (rclcpp::ok())
        {
            try
            {
                // RCLCPP_ERROR(this->get_logger(), "Init aaaaaaa");
                tf_livox_to_map = tf_buffer_->lookupTransform("odom", "lidar_link", tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                // RCLCPP_ERROR(this->get_logger(), "Transform lookup failed: %s", ex.what());
                // return;
                continue;
            }
            sleep_rate.sleep();
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr RemovePointcloud::LoadPointcloudFromPcd(const std::string &filemname)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloudBlob_;
        pcl::io::loadPCDFile(filename_, cloudBlob_);
        pcl::fromPCLPointCloud2(cloudBlob_, *cloud_);
        pcl::toROSMsg(*cloud_, pcd_map_);
        return cloud_;
    }
    void RemovePointcloud::LivoxLidarSubCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        accumulated_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        processed_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        // 转换ROS消息到PCL点云
        // pcl::PointCloud<pcl::PointXYZ>::Ptr livox_lidar_pointcloud_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *livox_lidar_pointcloud_);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 立方体边界
        float minX = -0.3, maxX = 0.3;
        float minY = -0.3, maxY = 0.3;
        float minZ = -1.0, maxZ = 1.0;

        // 遍历点云
        for (const auto &point : *livox_lidar_pointcloud_)
        {
            if (!(point.x >= minX && point.x <= maxX &&
                  point.y >= minY && point.y <= maxY &&
                  point.z >= minZ && point.z <= maxZ))
            {
                // 如果点不在立方体内，添加到过滤后的点云中
                cloud_filtered->points.push_back(point);
            }
        }

        *livox_lidar_pointcloud_ = *cloud_filtered;

        pointcloud_deque.push_back(*livox_lidar_pointcloud_);
        while (pointcloud_deque.size() > ACC_POINT_NUM)
        {
            pointcloud_deque.pop_front(); // 移除最老的点云
        }
        if (pointcloud_deque.size() <= ACC_POINT_NUM)
        {
            // 可以在这里处理这三个点云

            // 使用迭代器遍历点云而不删除元素
            for (auto &cloud : pointcloud_deque)
            {
                *processed_cloud += cloud;
            }
            RCLCPP_INFO(this->get_logger(), "pointcloud_deque.size() <= 3");
        }
        if (is_first_frame_)
        {
            is_first_frame_ = false;
            map_cloud_ = LoadPointcloudFromPcd(filename_);
            pcd_map_.header.frame_id = "map";
            pcd_map_.header.stamp = this->now();
            kdTree_.setInputCloud(map_cloud_);
            pcd_map_.header.stamp = this->now();
            map_pub_->publish(pcd_map_);
        }
        pcd_map_.header.stamp = this->now();
        map_pub_->publish(pcd_map_);
        // *processed_cloud = *livox_lidar_pointcloud_;
        pcl_ros::transformPointCloud(*processed_cloud, *transformed_cloud_, tf_livox_to_map);
        // pcl_ros::transformPointCloud(*livox_lidar_pointcloud_, *transformed_cloud_, tf_livox_to_map);
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*transformed_cloud_, output);
        output.header.frame_id = "odom";
        output.header.stamp = this->now();
        // auto fun_start_time = std::chrono::high_resolution_clock::now();
        ComparePointclouds(transformed_cloud_);
        // auto fun_end_time = std::chrono::high_resolution_clock::now();  // 结束时间
        // auto fun_eduration = std::chrono::duration_cast<std::chrono::milliseconds>(fun_end_time - fun_start_time);
        // RCLCPP_INFO(this->get_logger(),"fun_duration_time.count():%ld",fun_eduration.count());

        // 测量并打印执行时间
        // auto end_time = std::chrono::high_resolution_clock::now();  // 结束时间
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // RCLCPP_INFO(this->get_logger(),"fun_duration.count():%ld",duration.count());
    }
    void RemovePointcloud::ComparePointclouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b)
    {
        // auto start_time = std::chrono::high_resolution_clock::now();  // 开始时间
        pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cough_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto &cloud_b : cloud_b->points)
        {
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdTree_.nearestKSearch(cloud_b, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                float z_diff = cloud_b.z - map_cloud_->points[pointIdxNKNSearch[0]].z;
                if (z_diff >= dis_diff_)
                {
                    // RCLCPP_INFO(this->get_logger(),"dis_diff_:%f",dis_diff_);
                    result_cloud->points.push_back(pcl::PointXYZ(cloud_b.x, cloud_b.y, cloud_b.z));
                    cough_cloud->points.push_back(pcl::PointXYZ(cloud_b.x, cloud_b.y, cloud_b.z));
                }
                else if (z_diff >= 0.05)
                {
                    // RCLCPP_INFO(this->get_logger(),"dis_diff_:%f",dis_diff_);
                    cough_cloud->points.push_back(pcl::PointXYZ(cloud_b.x, cloud_b.y, cloud_b.z));
                }
            }
        }
        // auto end_time = std::chrono::high_resolution_clock::now();  // 结束时间
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        // RCLCPP_INFO(this->get_logger(),"duration.count():%ld",duration.count());
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*result_cloud, output);
        output.header.frame_id = "odom";
        // output.header.stamp = livox_lidar_stamp_; 仿真
        output.header.stamp = this->now();
        processed_point_cloud_pub_->publish(output);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cough_cloud);
        // sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);

        // 设置半径滤波器
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(filtered_cloud);
        outrem.setRadiusSearch(0.25);       // 设置检查半径为1米
        outrem.setMinNeighborsInRadius(10); // 设置最小邻居数为0，这意味着所有在半径内的点都将被删除

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        outrem.filter(*cloud_filtered);

        sensor_msgs::msg::PointCloud2 cough_output;
        // pcl::toROSMsg(*filtered_cloud, cough_output);
        pcl::toROSMsg(*cloud_filtered, cough_output);
        cough_output.header.frame_id = "odom";
        // cough_output.header.stamp = livox_lidar_stamp_; // 仿真
        cough_output.header.stamp = this->now();
        cough_point_cloud_pub_->publish(cough_output);
        // sleep(5);
        // RCLCPP_INFO(this->get_logger(), "cough_cloud 点云的个数: %zu", cough_cloud->points.size());
        // RCLCPP_INFO(this->get_logger(), "filtered_cloud 点云的个数: %zu", filtered_cloud->points.size());
    }
} // namespace taurus

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(taurus::RemovePointcloud);