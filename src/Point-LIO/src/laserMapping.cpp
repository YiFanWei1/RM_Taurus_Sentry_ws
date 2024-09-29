#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
// #include <ros/ros.h>
#include"rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include "IMU_Processing.hpp"
// #include <nav_msgs/Odometry.h>
#include"nav_msgs/msg/odometry.hpp"
// #include <nav_msgs/Path.h>
#include"nav_msgs/msg/path.hpp"
// #include <visualization_msgs/Marker.h>
#include"visualization_msgs/msg/marker.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
// #include <sensor_msgs/msg/PointCloud2.h>
#include"sensor_msgs/sensor_msgs/msg/point_cloud2.hpp"
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include<tf2/transform_datatypes.h>
#include<tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
// #include <geometry_msgs/msg/Vector3.h>
#include"geometry_msgs/msg/vector3.hpp"
// #include <livox_ros_driver2/CustomMsg.h>
#include"livox_ros_driver2/msg/custom_msg.h"
#include "parameters.h"
#include "Estimator.h"
// #include <ros/console.h>
#include<iostream>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Geometry>
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
const float MOV_THRESHOLD = 1.5f;
mutex mtx_buffer;
condition_variable sig_buffer;
string root_dir = ROOT_DIR;
int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int frame_ct = 0;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;
shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false, flg_first_scan = true;
PointCloudXYZI::Ptr  ptr_con(new PointCloudXYZI());
// Time Log Variables
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;
bool   lidar_pushed = false, flg_reset = false, flg_exit = false;
vector<BoxPointType> cub_needrm;
deque<PointCloudXYZI::Ptr>  lidar_buffer;
deque<double>               time_buffer;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;
V3D euler_cur;
MeasureGroup Measures;
sensor_msgs::msg::Imu imu_last, imu_next;
sensor_msgs::msg::Imu::SharedPtr imu_last_ptr;
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;
geometry_msgs::msg::TransformStamped static_map_to_odom_;
double stamp_;
sensor_msgs::msg::PointCloud2 pcd_map_;
PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string &filename){
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    pcl::PCLPointCloud2 cloudBlob;
    pcl::io::loadPCDFile(filename, cloudBlob);
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    pcl::toROSMsg(*cloud,pcd_map_);
    return cloud;
}
double get_time_sec(const builtin_interfaces::msg::Time &time)
{
    return rclcpp::Time(time).seconds();
}
void range_sensor_sub_cbk_(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
    static_map_to_odom_.transform.translation.x = msg->pose.position.x;
    static_map_to_odom_.transform.translation.y = msg->pose.position.y;
    static_map_to_odom_.transform.translation.z = msg->pose.position.z;
    static_map_to_odom_.transform.rotation.w    = msg->pose.orientation.w;
    static_map_to_odom_.transform.rotation.x    = msg->pose.orientation.x;
    static_map_to_odom_.transform.rotation.y    = msg->pose.orientation.y;
    static_map_to_odom_.transform.rotation.z    = msg->pose.orientation.z;
    // std::cout << "static_map_to_odom_.transform.translation.x:" << static_map_to_odom_.transform.translation.x <<std::endl;
}
rclcpp::Time get_ros_time(double timestamp)
{
    int32_t sec = std::floor(timestamp);
    auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
    uint32_t nanosec = nanosec_d;
    return rclcpp::Time(sec, nanosec);
}
void SigHandle(int sig)
{
    flg_exit = true;
    // ROS_WARN("catch sig %d", sig);
    std::cout<<"catch sig"<<sig<<std::endl;
    sig_buffer.notify_all();
}
inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang;
    if (!use_imu_as_input)
    {
        rot_ang = SO3ToEuler(kf_output.x_.rot);
    }
    else
    {
        rot_ang = SO3ToEuler(kf_input.x_.rot);
    }
    
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    if (use_imu_as_input)
    {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2)); // Bias_a  
    }
    else
    {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2)); // Pos  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2)); // Vel  
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));    // Bias_g  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));    // Bias_a  
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1), kf_output.x_.gravity(2)); // Bias_a  
    }
    fprintf(fp, "\r\n");  
    fflush(fp);
}
void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu;
    if (extrinsic_est_en)
    {
        if (!use_imu_as_input)
        {
            p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
        }
        else
        {
            p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
        }
    }
    else
    {
        p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
    }
    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}
int points_cache_size = 0;
void points_cache_collect() // seems for debug
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
/*
这个函数主要负责管理和更新局部地图的边界。
当激光雷达的位置接近当前局部地图的边界时，它会根据激光雷达的当前位置来调整局部地图的大小和位置。
这是为了确保激光雷达始终位于局部地图的中心区域，从而有效地处理和更新地图数据
*/
void lasermap_fov_segment()
{
    cub_needrm.shrink_to_fit();//对 cub_needrm 这个容器进行内存收缩，以适应其大小，移除多余的容量。

    V3D pos_LiD;//定义一个三维向量 pos_LiD，用于存储激光雷达的位置。
    if (use_imu_as_input)//检查是否使用IMU作为输入。false
    {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot * Lidar_T_wrt_IMU;//计算激光雷达的位置，这是通过将IMU的位置和旋转矩阵应用于激光雷达相对于IMU的位置来实现的。
    }
    else//如果不是用IMU作为输入。
    {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot * Lidar_T_wrt_IMU;//用类似的方法计算激光雷达的位置，但这次使用的是不同的状态变量（可能是经过处理的或不同模式的状态）。
    }
    if (!Localmap_Initialized){//检查本地地图是否已初始化。false(第一帧)
        for (int i = 0; i < 3; i++){//对于x、y、z三个维度。
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;//设置局部地图的最小顶点。
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;//设置局部地图的最大顶点。
        }
        Localmap_Initialized = true;//设置局部地图已初始化的标志。
        return;
    }
    float dist_to_map_edge[3][2];//定义一个二维数组，用于存储到地图边缘的距离。
    bool need_move = false;//定义一个布尔变量，用于标记是否需要移动地图。
    for (int i = 0; i < 3; i++){//对于x、y、z三个维度。
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);//计算到地图最小顶点的距离。
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);//计算到地图最大顶点的距离。
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;//如果到任一边缘的距离小于移动阈值和探测范围的乘积，则设置需要移动地图的标志。
    }
    if (!need_move) return;//如果不需要移动地图，则结束函数。
    BoxPointType New_LocalMap_Points, tmp_boxpoints;//定义新的局部地图点和临时盒子点。
    New_LocalMap_Points = LocalMap_Points;//将当前的局部地图点赋值给新的局部地图点。
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));//计算移动距离。
    for (int i = 0; i < 3; i++){//对于x、y、z三个维度。
        tmp_boxpoints = LocalMap_Points;//将当前的局部地图点赋值给临时盒子点。
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){//如果到最小顶点的距离小于移动阈值和探测范围的乘积。
            New_LocalMap_Points.vertex_max[i] -= mov_dist;//调整新局部地图点的最大顶点。
            New_LocalMap_Points.vertex_min[i] -= mov_dist;//调整新局部地图点的最小顶点。
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;//设置临时盒子点的最小顶点。
            cub_needrm.emplace_back(tmp_boxpoints);//将临时盒子点添加到需要移除的盒子列表中。
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){//如果到最大顶点的距离小于移动阈值和探测范围的乘积。
            New_LocalMap_Points.vertex_max[i] += mov_dist;//调整新局部地图点的最大顶点
            New_LocalMap_Points.vertex_min[i] += mov_dist;//调整新局部地图点的最小顶点。
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;//设置临时盒子点的最大顶点。
            cub_needrm.emplace_back(tmp_boxpoints);//将临时盒子点添加到需要移除的盒子列表中。
        }
    }
    LocalMap_Points = New_LocalMap_Points;//更新当前的局部地图点。

    points_cache_collect();//调用 points_cache_collect 函数来收集点缓存。
    if(cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);//如果需要移除的盒子列表不为空，则调用 kdtree 的 Delete_Point_Boxes 方法来删除这些盒子。
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    // if (msg->header.stamp.toSec() < last_timestamp_lidar)
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        // ROS_ERROR("lidar loop back, clear buffer");
        std::cout<<"lidar loop back, clear buffer"<<std::endl;
        // lidar_buffer.shrink_to_fit();

        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    // last_timestamp_lidar = msg->header.stamp.toSec();
    last_timestamp_lidar = get_time_sec(msg->header.stamp);

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
    double time_div = rclcpp::Time(msg->header.stamp).seconds();
    p_pre->process(msg, ptr);
    if (cut_frame)
    {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {
            ptr_div->push_back(ptr->points[i]);
            // cout << "check time:" << ptr->points[i].curvature << endl;
            // if (ptr->points[i].curvature / double(1000) + msg->header.stamp.toSec() - time_div > cut_frame_time_interval)
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div > cut_frame_time_interval)
            {
                if(ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr  ptr_div_i(new PointCloudXYZI());
                *ptr_div_i = *ptr_div;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty())
        {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            lidar_buffer.push_back(ptr_con_i);
            double time_con_i = time_con;
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    { 
        lidar_buffer.emplace_back(ptr);
        // time_buffer.emplace_back(msg->header.stamp.toSec());
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg) 
{
 // 获取当前时间（std::chrono::high_resolution_clock）
    auto curr_time = std::chrono::high_resolution_clock::now();
    // 将msg_time（rclcpp::Time）转换为std::chrono::time_point
    auto msg_time_chrono = std::chrono::time_point<std::chrono::high_resolution_clock>(
        std::chrono::seconds(msg->header.stamp.sec) + 
        std::chrono::nanoseconds(msg->header.stamp.nanosec));
    // 计算时间差（毫秒）
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - msg_time_chrono);
    // 打印时间差
    // std::cout<<"livox_pcl_cbk Time difference:"<<duration.count()<<" ms"<<std::endl;

    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    // if (msg->header.stamp.toSec() < last_timestamp_lidar)
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        // ROS_ERROR("lidar loop back, clear buffer");
        std::cout<<"lidar loop back, clear buffer"<<endl;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }

    // last_timestamp_lidar = msg->header.stamp.toSec();    
    last_timestamp_lidar = get_time_sec(msg->header.stamp);
    
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    // double time_div = msg->header.stamp.toSec();
    double time_div = get_time_sec(msg->header.stamp);

    if (cut_frame)
    {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {
            ptr_div->push_back(ptr->points[i]);
            // if (ptr->points[i].curvature / double(1000) + msg->header.stamp.toSec() - time_div > cut_frame_time_interval)
            if (ptr->points[i].curvature / double(1000) + get_time_sec(msg->header.stamp) - time_div > cut_frame_time_interval)
            {
                if(ptr_div->size() < 1) continue;
                PointCloudXYZI::Ptr  ptr_div_i(new PointCloudXYZI());
                // cout << "ptr div num:" << ptr_div->size() << endl;
                *ptr_div_i = *ptr_div;
                // cout << "ptr div i num:" << ptr_div_i->size() << endl;
                lidar_buffer.push_back(ptr_div_i);
                time_buffer.push_back(time_div);
                time_div += ptr->points[i].curvature / double(1000);
                ptr_div->clear();
            }
        }
        if (!ptr_div->empty())
        {
            lidar_buffer.push_back(ptr_div);
            // ptr_div->clear();
            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; //msg->header.stamp.toSec();
        }
        if (frame_ct < con_frame_num)
        {
            for (int i = 0; i < ptr->size(); i++)
            {
                ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
                ptr_con->push_back(ptr->points[i]);
            }
            frame_ct ++;
        }
        else
        {
            PointCloudXYZI::Ptr  ptr_con_i(new PointCloudXYZI());
            *ptr_con_i = *ptr_con;
            double time_con_i = time_con;
            lidar_buffer.push_back(ptr_con_i);
            time_buffer.push_back(time_con_i);
            ptr_con->clear();
            frame_ct = 0;
        }
    }
    else
    {
        lidar_buffer.emplace_back(ptr);
        // time_buffer.emplace_back(msg->header.stamp.toSec());
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr &msg_in) 
{    
    rclcpp::Time time_;
    // std::cout << "imuCallback" <<std::endl;
    stamp_ = (float)msg_in->header.stamp.sec;
    time_ = msg_in->header.stamp;
    // std::cout << "stamp_: "<< stamp_ <<std::endl;
    publish_count ++;
    // sensor_msgs::msg::Imu::Ptr msg(new sensor_msgs::msg::Imu(*msg_in));
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    // msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_lag_imu_to_lidar);
     msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_lag_imu_to_lidar);
    // double timestamp = msg->header.stamp.toSec();
    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        // ROS_ERROR("imu loop back, clear deque");
        std::cout<<"imu loop back, clear deque"<<std::endl;
        // imu_deque.shrink_to_fit();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }
    
    imu_deque.emplace_back(msg);
    last_timestamp_imu = timestamp;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
    if (!imu_en)
    {
        if (!lidar_buffer.empty())
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            time_buffer.pop_front();
            lidar_buffer.pop_front();
            if(meas.lidar->points.size() < 1) 
            {
                cout << "lose lidar" << std::endl;
                return false;
            }
            double end_time = meas.lidar->points.back().curvature;
            for (auto pt: meas.lidar->points)
            {
                if (pt.curvature > end_time)
                {
                    end_time = pt.curvature;
                }
            }
            lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
            meas.lidar_last_time = lidar_end_time;
            return true;
        }
        return false;
    }

    if (lidar_buffer.empty() || imu_deque.empty())
    {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        if(meas.lidar->points.size() < 1) 
        {
            cout << "lose lidar" << endl;
            lidar_buffer.pop_front();
            time_buffer.pop_front();
            return false;
        }
        meas.lidar_beg_time = time_buffer.front();
        double end_time = meas.lidar->points.back().curvature;
        for (auto pt: meas.lidar->points)
        {
            if (pt.curvature > end_time)
            {
                end_time = pt.curvature;
            }
        }
        lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
        
        meas.lidar_last_time = lidar_end_time;
        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_)
    {
        // double imu_time = imu_deque.front()->header.stamp.toSec();
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
            // imu_time = imu_deque.front()->header.stamp.toSec(); 
            imu_time = get_time_sec(imu_deque.front()->header.stamp); 
            if(imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }
    else if(!init_map)
    {
        // double imu_time = imu_deque.front()->header.stamp.toSec();
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);

        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
            // imu_time = imu_deque.front()->header.stamp.toSec(); 
            imu_time = get_time_sec(imu_deque.front()->header.stamp); 
            if(imu_time > lidar_end_time) break;
            meas.imu.emplace_back(imu_deque.front());
            imu_last = imu_next;
            imu_last_ptr = imu_deque.front();
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
        }
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

int process_increments = 0;
/*
函数的目的是实现地图的增量更新。它通过添加新的点云数据到已有的地图中，同时考虑了降采样和近邻点的影响
*/
void map_incremental()
{
    PointVector PointToAdd;//存储需要添加到地图中的点。
    PointVector PointNoNeedDownsample;//存储不需要降采样的点。
    PointToAdd.reserve(feats_down_size);//预留 PointToAdd 的大小为降采样后的特征点数量。
    PointNoNeedDownsample.reserve(feats_down_size);//预留 PointNoNeedDownsample 的大小为降采样后的特征点数量。
    
        for(int i = 0; i < feats_down_size; i++)//遍历所有降采样后的特征点。
        {            
            if (!Nearest_Points[i].empty())//如果第 i 个点的最近点集不为空。
            {
                const PointVector &points_near = Nearest_Points[i];//获取第 i 个点的最近点集。
                bool need_add = true;//设置一个标志变量 need_add，默认为真，表示这个点需要被添加到地图中。
                PointType downsample_result, mid_point; //定义两个 PointType 类型的变量，用于存储降采样结果和中间点。
                mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;//计算中间点的 x 坐标。
                mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;//计算中间点的 y 坐标。
                mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;//计算中间点的 z 坐标。
                    /* If the nearest points is definitely outside the downsample box */
                    if (fabs(points_near[0].x - mid_point.x) > 0.866 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 0.866 * filter_size_map_min || fabs(points_near[0].z - mid_point.z) > 0.866 * filter_size_map_min)
                    //判断最近点是否在降采样框外。
                    {
                        PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);//将这个点添加到不需要降采样的点集中。
                        continue;
                    }
                    /* Check if there is a point already in the downsample box */
                    float dist  = calc_dist<float>(feats_down_world->points[i],mid_point);//计算点到中间点的距离。
                    for (int readd_i = 0; readd_i < points_near.size(); readd_i ++)//遍历最近点集。
                    {
                            /* Those points which are outside the downsample box should not be considered. */
                            if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) 
                            //判断最近点是否在降采样框内。
                            {
                                need_add = false;//将 need_add 设置为假。
                                break;
                            }
                    }
                    if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
                    //如果需要添加这个点，则将其添加到 PointToAdd 中。
        }
        else
        {
                // PointToAdd.emplace_back(feats_down_world->points[i]);
                PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);//将这个点添加到不需要降采样的点集中。
        }
        }
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);//将需要添加的点添加到 kdtree 中，并接收添加的点的数量。
    std::cout << "add_point_size: " << add_point_size << std::endl;
    int no_need_down_num_ = ikdtree.Add_Points(PointNoNeedDownsample, false);//将不需要降采样的点也添加到 kdtree 中。
    std::cout << "no_need_down_num_: " << no_need_down_num_ << std::endl;
}

// void publish_init_kdtree(const ros::Publisher & pubLaserCloudFullRes)
void publish_init_kdtree(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr   laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector ().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                
    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);

    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    if (scan_pub_en)//true
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        
        for (int i = 0; i < size; i++)
        {
            // if (i % 3 == 0)
            // {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y; // 
            // }
        }
        //滤除头顶正上方的点
        pcl::CropBox<pcl::PointXYZINormal> cropBox;
        cropBox.setInputCloud(laserCloudWorld);
        cropBox.setMin(Eigen::Vector4f(-0.3, -0.3, -2.0, 1.0));
        cropBox.setMax(Eigen::Vector4f(0.3, 0.3,2.0 , 1.0));
        cropBox.setNegative(true); // 设置为 true 以保留框外的点
        // 应用过滤器并直接获取过滤后的点云
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cylinderRemoved(new pcl::PointCloud<pcl::PointXYZINormal>());
        cropBox.filter(*cloud_cylinderRemoved);
        // 将滤波后的结果点云重新赋值给cloud_filtered
        *laserCloudWorld = *cloud_cylinderRemoved;

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
         if(invert)
         {
            // 绕x旋转180度
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            float theta = M_PI;
            transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
            PointCloudXYZI::Ptr laserCloudWorld_rotated(new pcl::PointCloud<pcl::PointXYZINormal>());
            pcl::transformPointCloud(*laserCloudWorld, *laserCloudWorld_rotated, transform);
            pcl::toROSMsg(*laserCloudWorld_rotated, laserCloudmsg);
         }else{
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
         }


        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
    
    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_down_world->points.size();
        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        if(invert)
         {
            // 绕x旋转180度
            Eigen::Affine3f transform = Eigen::Affine3f::Identity();
            float theta = M_PI;
            transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
            PointCloudXYZI::Ptr laserCloudWorld_rotated(new pcl::PointCloud<pcl::PointXYZINormal>());
            pcl::transformPointCloud(*laserCloudWorld, *laserCloudWorld_rotated, transform);
            *pcl_wait_save += *laserCloudWorld_rotated;
         }else{
            *pcl_wait_save += *laserCloudWorld;
         }
        

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            // pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudIMUBody(new pcl::PointCloud<pcl::PointXYZINormal>(size, 1));
    sensor_msgs::msg::PointCloud2 laserCloudmsg;

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    // 体素滤波
    double voxelsize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_filter;
    voxel_filter.setInputCloud(laserCloudIMUBody);
    voxel_filter.setLeafSize(voxelsize, voxelsize, voxelsize); // 设置体素大小，例如0.1m
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZINormal>());
    voxel_filter.filter(*cloud_filtered);

    //滤除头顶正上方的点
    pcl::CropBox<pcl::PointXYZINormal> cropBox;
    cropBox.setInputCloud(cloud_filtered);
    cropBox.setMin(Eigen::Vector4f(-0.3, -0.3, -2.0, 1.0));
    cropBox.setMax(Eigen::Vector4f(0.3, 0.3, 2.0, 1.0));
    cropBox.setNegative(true); // 设置为 true 以保留框外的点

    // 应用过滤器并直接获取过滤后的点云
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_cylinderRemoved(new pcl::PointCloud<pcl::PointXYZINormal>());
    cropBox.filter(*cloud_cylinderRemoved);
    // 将滤波后的结果点云重新赋值给cloud_filtered
    *cloud_filtered = *cloud_cylinderRemoved;

    if(invert)
    {
        // 绕x旋转180度
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        float theta = M_PI;
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_rotated(new pcl::PointCloud<pcl::PointXYZINormal>());
        pcl::transformPointCloud(*cloud_filtered, *cloud_rotated, transform);
        pcl::toROSMsg(*cloud_rotated, laserCloudmsg);
    }else{
        pcl::toROSMsg(*cloud_filtered, laserCloudmsg);
    }
   
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    // laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);//rclcpp::Time();
    laserCloudmsg.header.stamp = rclcpp::Time();
    laserCloudmsg.header.frame_id = "base_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)//发布Lasermap
{
    bool dense_pub_en = false;
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);//feats_down_body特征点
    int size = laserCloudFullRes->points.size();
    // std::cout  << "laserCloudFullRes->points.size(): " << laserCloudFullRes->points.size() << std::endl;
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    int point_num = laserCloudmsg.row_step / laserCloudmsg.point_step;
    // std::cout << "point_num: " << point_num << std::endl;
    double stamp_ = (float)laserCloudmsg.header.stamp.sec;
    pcd_map_.header.stamp = get_ros_time(lidar_end_time);
    pcd_map_.header.frame_id = "odom";
    pubLaserCloudMap->publish(pcd_map_);
}
bool is_first_kf_ = true;
template<typename T>
void set_posestamp(T & out)
{
    if (!use_imu_as_input)
    {   
        if(is_first_kf_)
        {
            kf_output.x_.pos(0) = 2.5;
            out.position.x = kf_output.x_.pos(0);
            out.position.y = kf_output.x_.pos(1);
            out.position.z = kf_output.x_.pos(2);
            Eigen::Quaterniond q(kf_output.x_.rot);
            out.orientation.x = q.coeffs()[0];
            out.orientation.y = q.coeffs()[1];
            out.orientation.z = q.coeffs()[2];
            out.orientation.w = q.coeffs()[3];
            is_first_kf_ = false;
        }
        else
        {
            out.position.x = kf_output.x_.pos(0);
            out.position.y = kf_output.x_.pos(1);
            out.position.z = kf_output.x_.pos(2);
            Eigen::Quaterniond q(kf_output.x_.rot);
            out.orientation.x = q.coeffs()[0];
            out.orientation.y = q.coeffs()[1];
            out.orientation.z = q.coeffs()[2];
            out.orientation.w = q.coeffs()[3];
        }
    }
    else
    {
        out.position.x = kf_input.x_.pos(0);
        out.position.y = kf_input.x_.pos(1);
        out.position.z = kf_input.x_.pos(2);
        Eigen::Quaterniond q(kf_input.x_.rot);
        out.orientation.x = q.coeffs()[0];
        out.orientation.y = q.coeffs()[1];
        out.orientation.z = q.coeffs()[2];
        out.orientation.w = q.coeffs()[3];
    }
}

// void publish_odometry(const ros::Publisher & pubOdomAftMapped)
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "base_link";
    if (publish_odometry_without_downsample)
    {
        // odomAftMapped.header.stamp = ros::Time().fromSec(time_current);
        odomAftMapped.header.stamp = get_ros_time(time_current);
    }
    else
    {
        // odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    if(invert)
    {
        odomAftMapped.pose.pose.position.y = -odomAftMapped.pose.pose.position.y;
        odomAftMapped.pose.pose.position.z = -odomAftMapped.pose.pose.position.z;
    }

    pubOdomAftMapped->publish(odomAftMapped);

    // static tf::TransformBroadcaster br;
    // tf::Transform                   transform;
    // tf::Quaternion                  q;
    // transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
    //                                 odomAftMapped.pose.pose.position.y, \
    //                                 odomAftMapped.pose.pose.position.z));
    // q.setW(odomAftMapped.pose.pose.orientation.w);
    // q.setX(odomAftMapped.pose.pose.orientation.x);
    // q.setY(odomAftMapped.pose.pose.orientation.y);
    // q.setZ(odomAftMapped.pose.pose.orientation.z);
    // transform.setRotation( q );
    // br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "odom", "base_link" ) );

        // 声明一个tf2的TransformBroadcaster

    // 创建一个tf2的Transform对象
    geometry_msgs::msg::TransformStamped transformStamped;

    // 填充TransformStamped消息
    transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    if(invert)
    {
        transformStamped.transform.rotation.z = -odomAftMapped.pose.pose.orientation.z;
    }else{
        transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    }

    // 设定header信息
    transformStamped.header.stamp = get_ros_time(lidar_end_time);
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";

    // 广播TransformStamped消息
    tf_br->sendTransform(transformStamped);

    // 创建一个tf2的Transform对象
    geometry_msgs::msg::TransformStamped transformStamped_;

    // 填充TransformStamped消息
    transformStamped_.transform.translation.x = 0;
    transformStamped_.transform.translation.y = 0;
    transformStamped_.transform.translation.z = 0;
    transformStamped_.transform.rotation.w = 1;
    transformStamped_.transform.rotation.x = 0;
    transformStamped_.transform.rotation.y = 0;
    if(invert)
    {
        transformStamped_.transform.rotation.z = 0;
    }else{
        transformStamped_.transform.rotation.z = 0;
    }

    // 设定header信息
    // rclcpp::Clock::SharedPtr clock_;
    transformStamped_.header.stamp = get_ros_time(lidar_end_time);
    transformStamped_.header.frame_id = "map";
    transformStamped_.child_frame_id = "odom";

    // 广播TransformStamped消息
    tf_br->sendTransform(transformStamped_);
    
    // 创建一个tf2的Transform对象
    geometry_msgs::msg::TransformStamped tf_nav_link_;

    // 填充TransformStamped消息
    tf_nav_link_.transform.translation.x = odomAftMapped.pose.pose.position.x;
    tf_nav_link_.transform.translation.y = odomAftMapped.pose.pose.position.y;
    tf_nav_link_.transform.translation.z = 0.2;
    tf_nav_link_.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    tf_nav_link_.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    tf_nav_link_.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    if(invert)
    {
        tf_nav_link_.transform.rotation.z = -odomAftMapped.pose.pose.orientation.z;
    }else{
        tf_nav_link_.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    }

    // 设定header信息
    // rclcpp::Clock::SharedPtr clock_;
    tf_nav_link_.header.stamp = get_ros_time(lidar_end_time);
    tf_nav_link_.header.frame_id = "odom";
    tf_nav_link_.child_frame_id = "nav_link";

    // 广播TransformStamped消息
    tf_br->sendTransform(tf_nav_link_);
}

void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose.pose);
    // msg_body_pose.header.stamp = ros::Time::now();
    // msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";
    static int jjj = 0;
    jjj++;
    // if (jjj % 2 == 0) // if path is too large, the rvis will crash
    {
        if(invert)
        {
            msg_body_pose.pose.position.y = -msg_body_pose.pose.position.y;
            msg_body_pose.pose.position.z = -msg_body_pose.pose.position.z;
        }
        path.poses.emplace_back(msg_body_pose);
        pubPath->publish(path);
    }
}        


class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("laser_mapping", options)
    {
        readParameters();
        path.header.stamp    = get_ros_time(lidar_end_time);
        path.header.frame_id ="odom";
        double FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        double HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);
        memset(point_selected_surf, true, sizeof(point_selected_surf));//将point_selected_surf数组的所有元素都设置为1，即将它们全部设置为true
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);//降采样
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);//降采样
        Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
        if (extrinsic_est_en)//false
        {
            if (!use_imu_as_input)
            {
                kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
                kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
            }
            else
            {
                kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
                kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
            }
        }
        p_imu->lidar_type = p_pre->lidar_type = lidar_type;
        p_imu->imu_en = imu_en;

        kf_input.init_dyn_share_modified(get_f_input, df_dx_input, h_model_input);//初始化动力学模型相关的参数
        kf_output.init_dyn_share_modified_2h(get_f_output, df_dx_output, h_model_output, h_model_IMU_output);//初始化动力学模型相关的参数
        Eigen::Matrix<double, 24, 24> P_init = MD(24,24)::Identity() * 0.01;
        P_init.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
        P_init.block<6, 6>(15, 15) = MD(6,6)::Identity() * 0.001;
        P_init.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
        kf_input.change_P(P_init);//更新卡尔曼滤波器的协方差矩阵
        Eigen::Matrix<double, 30, 30> P_init_output = MD(30,30)::Identity() * 0.01;
        P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
        P_init_output.block<6, 6>(6, 6) = MD(6,6)::Identity() * 0.0001;
        P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
        kf_input.change_P(P_init);//更新卡尔曼滤波器的协方差矩阵
        kf_output.change_P(P_init_output);//更新卡尔曼滤波器的协方差矩阵
       
        Q_input = process_noise_cov_input();//计算输入和输出的过程噪声协方差矩阵
        Q_output = process_noise_cov_output();

        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(),"w");
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
        fout_imu_pbp.open(DEBUG_FILE_DIR("imu_pbp.txt"),ios::out);
        if (fout_out && fout_imu_pbp)
            cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
        else
            cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;


        if (p_pre->lidar_type == AVIA)
        {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic, 200000, livox_pcl_cbk);
        }
        else
        {
            sub_pcl_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, 200000, standard_pcl_cbk);
        }
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 200000, imu_cbk);
        pubLaserCloudFull = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
        pubLaserCloudFull_body = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
        pubLaserCloudEffect = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
        pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 20);
        pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
        pubPath = this->create_publisher<nav_msgs::msg::Path>("/path", 100000);
        plane_pub = this->create_publisher<visualization_msgs::msg::Marker>("planner_normal",1000);
        range_sensor_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("range_sensor_pose",20,range_sensor_sub_cbk_);
        static_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));  // 1ms
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
        auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));       // 1s

        map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));//发布Lasermap

        // pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;

    }

    ~LaserMappingNode()
    {
        fout_out.close();
        fout_imu_pbp.close();
        fclose(fp);
    }

private:
    void readParameters()
    {
        p_pre.reset(new Preprocess());

        this->declare_parameter<bool>("prop_at_freq_of_imu", true);  //true
        this->declare_parameter<bool>("use_imu_as_input", false);  //false
        this->declare_parameter<bool>("check_satu", true); //true
        this->declare_parameter<int>("init_map_size", 10);  //10
        this->declare_parameter<bool>("space_down_sample", true);  //true
        this->declare_parameter<double>("mapping.satu_acc", 3.);  //3.0
        this->declare_parameter<double>("mapping.satu_gyro", 35.); //35
        this->declare_parameter<double>("mapping.acc_norm",1.); //1.0
        this->declare_parameter<float>("mapping.plane_thr", 0.1); //0.1
        this->declare_parameter<int>("point_filter_num", 1); //1
        this->declare_parameter<std::string>("common.lid_topic","/livox/lidar");  
        this->declare_parameter<std::string>("common.imu_topic","/livox/imu");
        this->declare_parameter<bool>("common.con_frame",false);  //false
        this->declare_parameter<int>("common.con_frame_num",1); //1
        this->declare_parameter<bool>("common.cut_frame",false); //false
        this->declare_parameter<double>("common.cut_frame_time_interval",0.1); //0.1
        this->declare_parameter<double>("common.time_lag_imu_to_lidar",0.);  //0.0
        this->declare_parameter<double>("filter_size_surf",0.3);  //0.3
        this->declare_parameter<double>("filter_size_map",0.2); //0.2
        this->declare_parameter<double>("cube_side_length",2000.); //2000
        this->declare_parameter<float>("mapping.det_range",100.f);  //100
        this->declare_parameter<double>("mapping.fov_degree",360.); //360
        this->declare_parameter<bool>("mapping.imu_en",true); //true
        this->declare_parameter<bool>("mapping.start_in_aggressive_motion",false);  //false
        this->declare_parameter<bool>("mapping.extrinsic_est_en",false); //false
        this->declare_parameter<double>("mapping.imu_time_inte",0.005);  //0.005
        this->declare_parameter<double>("mapping.lidar_meas_cov",0.001); //0.001
        this->declare_parameter<double>("mapping.acc_cov_input",0.1); //0.1
        this->declare_parameter<double>("mapping.vel_cov",20.); //
        this->declare_parameter<double>("mapping.gyr_cov_input",0.01); //0.01
        this->declare_parameter<double>("mapping.gyr_cov_output",1000.); //1000
        this->declare_parameter<double>("mapping.acc_cov_output",500.); //500
        this->declare_parameter<double>("mapping.b_gyr_cov",0.0001); //4.94982e-05 dyn
        this->declare_parameter<double>("mapping.b_acc_cov",0.0001); //2.30444e-05 dyn
        this->declare_parameter<double>("mapping.imu_meas_acc_cov",0.1); //0.0008957688 dyn
        this->declare_parameter<double>("mapping.imu_meas_omg_cov",0.1);  //0.0011397957 dyn
        this->declare_parameter<bool>("mapping.invert",true);
        this->declare_parameter<double>("preprocess.blind", 0.5); //0.5
        this->declare_parameter<int>("preprocess.lidar_type", 1); //1
        this->declare_parameter<int>("preprocess.scan_line", 4); //4
        this->declare_parameter<int>("preprocess.scan_rate", 10); //
        this->declare_parameter<int>("preprocess.timestamp_unit", 3);  //1
        this->declare_parameter<double>("mapping.match_s", 81.0); //81
        this->declare_parameter<bool>("mapping.gravity_align", true); //true
        this->declare_parameter<vector<double>>("mapping.gravity", vector<double>()); //0.0 0.0 -0.981
        this->declare_parameter<vector<double>>("mapping.gravity_init", vector<double>());  //0.0 0.0 -0.981
        this->declare_parameter<vector<double>>("mapping.extrinsic_T", vector<double>()); //[ 0.04165, 0.02326, -0.0284 ]
        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>()); 
        this->declare_parameter<bool>("odometry.publish_odometry_without_downsample", false);  //false
        this->declare_parameter<bool>("publish.path_en", true); //true
        this->declare_parameter<bool>("publish.scan_publish_en",true); //true
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en",false);  //false
        this->declare_parameter<bool>("runtime_pos_log_enable", 0); //false
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false); //false
        this->declare_parameter<int>("pcd_save.interval", -1); //-1

        this->get_parameter_or<bool>("prop_at_freq_of_imu", prop_at_freq_of_imu, true);
        this->get_parameter_or<bool>("use_imu_as_input", use_imu_as_input, false);
        this->get_parameter_or<bool>("check_satu", check_satu, true);  
        this->get_parameter_or<int>("init_map_size", init_map_size, 10);
        this->get_parameter_or<bool>("space_down_sample", space_down_sample, true); 
        this->get_parameter_or<double>("mapping.satu_acc",satu_acc,3.0); 
        this->get_parameter_or<double>("mapping.satu_gyro",satu_gyro,35.0);
        this->get_parameter_or<double>("mapping.acc_norm",acc_norm,1.0);
        this->get_parameter_or<float>("mapping.plane_thr", plane_thr, 0.1f);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num, 1);
        this->get_parameter_or<std::string>("common.lid_topic",lid_topic,"/livox/lidar");
        this->get_parameter_or<std::string>("common.imu_topic", imu_topic,"/livox/imu");
        this->get_parameter_or<bool>("common.con_frame",con_frame,false);
        this->get_parameter_or<int>("common.con_frame_num",con_frame_num,1);
        this->get_parameter_or<bool>("common.cut_frame",cut_frame,false);
        this->get_parameter_or<double>("common.cut_frame_time_interval",cut_frame_time_interval,0.1);
        this->get_parameter_or<double>("common.time_lag_imu_to_lidar",time_lag_imu_to_lidar,0.0);
        this->get_parameter_or<double>("filter_size_surf",filter_size_surf_min,0.3);
        this->get_parameter_or<double>("filter_size_map",filter_size_map_min,0.2);
        this->get_parameter_or<double>("cube_side_length",cube_len,2000.);
        this->get_parameter_or<float>("mapping.det_range",DET_RANGE,100.f);
        this->get_parameter_or<double>("mapping.fov_degree",fov_deg,360.);
        this->get_parameter_or<bool>("mapping.imu_en",imu_en,true);
        this->get_parameter_or<bool>("mapping.start_in_aggressive_motion",non_station_start,false);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en",extrinsic_est_en,false);
        this->get_parameter_or<double>("mapping.imu_time_inte",imu_time_inte,0.005);
        this->get_parameter_or<double>("mapping.lidar_meas_cov",laser_point_cov,0.001);
        this->get_parameter_or<double>("mapping.acc_cov_input",acc_cov_input,0.1);
        this->get_parameter_or<double>("mapping.vel_cov",vel_cov,20.);
        this->get_parameter_or<double>("mapping.gyr_cov_input",gyr_cov_input,0.01);
        this->get_parameter_or<double>("mapping.gyr_cov_output",gyr_cov_output,1000.);
        this->get_parameter_or<double>("mapping.acc_cov_output",acc_cov_output,500.);
        this->get_parameter_or<double>("mapping.b_gyr_cov",b_gyr_cov,0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov",b_acc_cov,0.0001);
        this->get_parameter_or<double>("mapping.imu_meas_acc_cov",imu_meas_acc_cov,0.1);
        this->get_parameter_or<double>("mapping.imu_meas_omg_cov",imu_meas_omg_cov,0.1);
        this->get_parameter_or<bool>("mapping.invert",invert,true);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 1.0);
        this->get_parameter_or<int>("preprocess.lidar_type", lidar_type, 1);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 4);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE, 10);
        this->get_parameter_or<int>("preprocess.timestamp_unit", p_pre->time_unit, 3);
        this->get_parameter_or<double>("mapping.match_s", match_s, 81.0);
        this->get_parameter_or<bool>("mapping.gravity_align", gravity_align, true);
        this->get_parameter_or<vector<double>>("mapping.gravity", gravity, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.gravity_init", gravity_init, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT, vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        this->get_parameter_or<bool>("odometry.publish_odometry_without_downsample", publish_odometry_without_downsample, false);
        this->get_parameter_or<bool>("publish.path_en",path_en, true);
        this->get_parameter_or<bool>("publish.scan_publish_en",scan_pub_en,true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en",scan_body_pub_en,false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log, false);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en, false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        
    }

    void timer_callback()
    {
        // std::cout<<"timer_callback begin"<<std::endl;
        if(sync_packages(Measures)) //检查是否成功同步了测量数据（包括激光雷达和IMU数据）
        {
            if (flg_first_scan)//检查是否是第一次扫描。
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                cout << "first lidar time" << first_lidar_time << endl;
            }

            if (flg_reset)//检查是否需要重置
            {
                // ROS_WARN("reset when rosbag play back");
                std::cout<<"reset when rosbag play back"<<std::endl;
                p_imu->Reset();//调用IMU对象的Reset方法，重置IMU状态
                flg_reset = false;//将重置标志设为假
                return;
            }
            double t0,t1,t2,t3,t4,t5,match_start, solve_start;//定义多个双精度浮点数变量，用于记录时间点。
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();//记录当前时间（操作前）
            
            p_imu->Process(Measures, feats_undistort);//处理IMU数据，可能涉及数据的去畸变。!!!!!
            if (p_imu->imu_need_init_)//检查IMU是否需要初始化。
            {
                return;
            }
            if(imu_en)//检查是否启用了IMU（惯性测量单元）。true
            {
                if (!p_imu->gravity_align_)//检查是否已经完成了重力对齐。
                {
                    while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp))//在激光雷达数据的时间戳大于下一个IMU数据包的时间戳时进行循环。
                    {
                        imu_last = imu_next;//将上一个IMU数据设置为当前的IMU数据
                        imu_next = *(imu_deque.front());//从IMU数据队列中取出下一个数据
                        imu_deque.pop_front();//从队列前端移除刚刚取出的IMU数据
                        // imu_deque.pop();
                    }
                    if (non_station_start)//检查是否是非静止状态开始。false
                    {
                        state_in.gravity << VEC_FROM_ARRAY(gravity_init);//从初始重力数组中获取重力向量，并赋值给内部状态。
                        state_out.gravity << VEC_FROM_ARRAY(gravity_init);//同上，但赋值给输出状态。
                        state_out.acc << VEC_FROM_ARRAY(gravity_init);//从初始重力数组中获取加速度向量，并赋值给输出状态
                        state_out.acc *= -1;//将输出状态的加速度向量取反。
                    }
                    else //如果不是非静止状态开始，则进入此分支
                    {
                        state_in.gravity =  -1 * p_imu->mean_acc * G_m_s2 / acc_norm; //根据IMU的平均加速度、重力常数和加速度范数计算内部状态的重力向量。
                        state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm; //同上，但赋值给输出状态。
                        state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;//计算输出状态的加速度向量。
                    }
                    if (gravity_align)//检查是否启用了重力对齐。false
                    {
                        Eigen::Matrix3d rot_init;//定义一个初始旋转矩阵。
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);//从数组中获取重力向量，并赋值给IMU对象。
                        p_imu->Set_init(state_in.gravity, rot_init);//设置IMU的初始状态。
                        state_in.gravity = p_imu->gravity_;//更新内部状态的重力向量。
                        state_out.gravity = p_imu->gravity_;//更新输出状态的重力向量。
                        state_in.rot = rot_init;//更新内部状态的旋转矩阵。
                        state_out.rot = rot_init;//更新输出状态的旋转矩阵。
                        state_out.acc = -rot_init.transpose() * state_out.gravity;//根据旋转矩阵和重力向量计算输出状态的加速度向量。
                    }
                    kf_input.change_x(state_in);//在卡尔曼滤波器输入中更改状态。
                    kf_output.change_x(state_out);//在卡尔曼滤波器输出中更改状态。
                    p_imu->gravity_align_ = true;//将IMU的重力对齐标志设为真。
                }
            }
            else //如果IMU未启用或重力对齐已完成，则执行这部分代码。
            {
                if (!p_imu->gravity_align_)//再次检查重力是否对齐。
                {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);//使用初始化的重力向量设置内部状态的重力。
                    if (gravity_align)//如果启用了重力对齐，则执行以下步骤。
                    {
                        Eigen::Matrix3d rot_init;//创建一个Eigen库中的3x3矩阵，用作初始旋转矩阵。
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);//将重力数组的值赋给IMU对象的重力属性。
                        p_imu->Set_init(state_in.gravity, rot_init);//使用内部状态的重力和初始旋转矩阵来设置IMU的初始状态。
                        state_out.gravity = p_imu->gravity_;//将IMU对象的重力赋值给输出状态的重力。
                        state_out.rot = rot_init;//将初始旋转矩阵赋值给输出状态的旋转。
                        state_out.acc = -rot_init.transpose() * state_out.gravity;//根据旋转矩阵和重力计算输出状态的加速度。
                    }
                    else//如果不进行重力对齐，执行以下步骤。
                    {
                        state_out.gravity << VEC_FROM_ARRAY(gravity_init);//使用初始化的重力向量设置输出状态的重力。
                        state_out.acc << VEC_FROM_ARRAY(gravity_init);//使用初始化的加速度向量设置输出状态的加速度。
                        state_out.acc *= -1;//将输出状态的加速度向量取反。
                    }
                    kf_output.change_x(state_out);//在卡尔曼滤波器的输出中更改状态。
                    p_imu->gravity_align_ = true;//设置IMU对象的重力对齐标志为真。
                }
            }
            /*** Segment the map in lidar FOV 下面的代码是用于在激光雷达视场内分割地图。 ***/
            lasermap_fov_segment();//这个函数主要负责管理和更新局部地图的边界。
            //当激光雷达的位置接近当前局部地图的边界时，它会根据激光雷达的当前位置来调整局部地图的大小和位置。这是为了确保激光雷达始终位于局部地图的中心区域，从而有效地处理和更新地图数据。
            /*** downsample the feature points in a scan ***/

            /*
            这段代码主要处理点云的降采样、时间排序、转换到不同坐标系，并进行状态估计的准备工作。它涉及到空间和时间的处理，以及坐标转换和状态估计的基础准备。
            */
            t1 = omp_get_wtime();//记录当前时间点，使用OpenMP库的 omp_get_wtime 函数。
            if(space_down_sample)//检查是否启用了空间降采样。true
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);//设置降采样滤波器的输入为去畸变后的特征点云。
                downSizeFilterSurf.filter(*feats_down_body);//使用滤波器对点云进行降采样。
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); //根据时间列表对降采样后的点云进行排序。
            }
            else//如果没有启用空间降采样。
            {
                feats_down_body = Measures.lidar;//直接使用激光雷达的原始测量数据作为降采样点云。
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); //根据时间列表对点云进行排序。
            }
            time_seq = time_compressing<int>(feats_down_body);//对降采样后的点云进行时间压缩处理。
            feats_down_size = feats_down_body->points.size();//记录降采样后点云的大小。
            
            /*** initialize the map kdtree ***/
            if(!init_map)//检查是否需要初始化地图的kdtree。
            {
                if(ikdtree.Root_Node == nullptr) //检查kdtree的根节点是否为空。
                // if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);//设置kdtree降采样参数。
                }
                    
                feats_down_world->resize(feats_down_size);//调整世界坐标下降采样点云的大小。
                for(int i = 0; i < feats_down_size; i++)//遍历降采样后的点云。
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));//将点云从本体坐标系转换到世界坐标系。
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) //遍历转换后的点云。
                {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);//将转换后的点云点添加到初始化特征点云中。
                }
                if(init_feats_world->size() < init_map_size) return;//如果初始化特征点云的大小小于设定的地图大小，则返回。10
                // ikdtree.Build(init_feats_world->points); //使用初始化特征点云构建kdtree。
                PointCloudXYZI::Ptr map_cloud_(new PointCloudXYZI());
                // std::string filename = "/home/wei/github_code/PCD/downsampled_rm_pt.pcd";
                std::string filename = "/home/wei/code/test/src/Point-LIO/PCD_SAVE/dense_map_2.pcd";
                //在这里把pcd地图载入
                map_cloud_ = loadPointcloudFromPcd(filename);
                ikdtree.Build(map_cloud_->points); 
                init_map = true;//设置地图初始化标志为真。
                publish_init_kdtree(pubLaserCloudMap); //(pubLaserCloudFullRes);发布初始化后的kdtree。
                std::cout << "publish_init_kdtree(pubLaserCloudMap);" << std::endl;
                return;
            }        
            /*** ICP and Kalman filter update ICP和卡尔曼滤波器更新。 ***/
            normvec->resize(feats_down_size);//调整法向量数组的大小。
            feats_down_world->resize(feats_down_size);//调整世界坐标系下点云的大小。

            Nearest_Points.resize(feats_down_size);//调整最近点数组的大小。

            t2 = omp_get_wtime();//记录当前时间点。
            
            /*** iterated state estimation 用于迭代状态估计。 ***/
            crossmat_list.reserve(feats_down_size);//预留交叉矩阵列表的空间。
            pbody_list.reserve(feats_down_size);//预留本体点列表的空间。
            for (size_t i = 0; i < feats_down_body->size(); i++)//遍历点云。
            {
                V3D point_this(feats_down_body->points[i].x,//创建当前点的三维向量。
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;//将该点添加到本体点列表。
                if (extrinsic_est_en)//如果启用了外部估计。false
                {
                    if (!use_imu_as_input)//如果不使用IMU作为输入。
                    {
                        point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;//将点转换到另一个坐标系。
                    }
                    else //如果使用IMU作为输入。
                    {
                        point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;//将点转换到另一个坐标系。
                    }
                }
                else//如果没有启用外部估计。
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;//将点转换到激光雷达坐标系。
                }
                M3D point_crossmat;//定义一个3x3矩阵。
                point_crossmat << SKEW_SYM_MATRX(point_this);//将点转换为斜对称矩阵。
                crossmat_list[i]=point_crossmat;//将斜对称矩阵添加到交叉矩阵列表。
            }
            


            if (!use_imu_as_input)//如果不使用IMU作为输入。false
            {     
                // bool imu_upda_cov = false;
                effct_feat_num = 0;//设置有效特征点数量为0。
                /**** point by point update ****/

                double pcl_beg_time = Measures.lidar_beg_time;//记录激光雷达数据开始的时间。
                idx = -1;//初始化索引变量 idx 为-1。
                for (k = 0; k < time_seq.size(); k++)//对时间序列进行遍历。
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];//获取当前处理的点云点。
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;//计算当前点的时间戳。
                    if (is_first_frame)//如果是第一帧。
                    {
                        if(imu_en)//如果启用了IMU。true
                        {
                            while (time_current > get_time_sec(imu_next.header.stamp))//如果当前时间大于下一个IMU数据的时间戳，进行循环。
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_deque.pop_front();
                                // imu_deque.pop();
                            }

                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;//获取平均角速度。
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;//获取平均加速度。
                        }
                        is_first_frame = false;//设置第一帧标志为假。
                        // imu_upda_cov = true;
                        time_update_last = time_current;//更新最后一次更新的时间。
                        time_predict_last_const = time_current;//更新最后一次预测的恒定时间。
                    }
                    if(imu_en)//如果启用了IMU。
                    {
                        // bool imu_comes = time_current > imu_next.header.stamp.toSec();
                         bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                        while (imu_comes) //当IMU数据到来时，进行循环。
                        {
                            // imu_upda_cov = true;
                            angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

                            /*** covariance update ***/
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            // double dt = imu_last.header.stamp.toSec() - time_predict_last_const;
                            double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                                kf_output.predict(dt, Q_output, input_in, true, false);                  ///进行卡尔曼滤波器的预测步骤。
                            // time_predict_last_const = imu_last.header.stamp.toSec(); // big problem
                            time_predict_last_const = get_time_sec(imu_last.header.stamp); // big problem  更新最后一次预测的恒定时间。
                            // imu_comes = time_current > imu_next.header.stamp.toSec();
                            imu_comes = time_current > get_time_sec(imu_next.header.stamp);  //计算协方差更新的时间差。
                            // if (!imu_comes)
                            {
                                // double dt_cov = imu_last.header.stamp.toSec() - time_update_last; 
                                double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last; 

                                if (dt_cov > 0.0)  //如果时间差大于0。
                                {
                                    // time_update_last = imu_last.header.stamp.toSec();
                                    time_update_last = get_time_sec(imu_last.header.stamp);
                                    double propag_imu_start = omp_get_wtime();

                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);//进行卡尔曼滤波器的预测步骤，用于更新协方差。

                                    propag_time += omp_get_wtime() - propag_imu_start;
                                    double solve_imu_start = omp_get_wtime();
                                    kf_output.update_iterated_dyn_share_IMU(); //使用IMU数据更新迭代动态共享的卡尔曼滤波器。
                                    solve_time += omp_get_wtime() - solve_imu_start;
                                }
                            }
                        }
                    }

                    double dt = time_current - time_predict_last_const;//计算时间差。
                    double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu) //如果不是以IMU的频率进行传播。
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);//进行卡尔曼滤波器的预测步骤。
                            time_update_last = time_current;   
                        }
                    }
                        kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1) //如果降采样后的特征点数量小于1。
                    {
                        // ROS_WARN("No point, skip this scan!\n");
                        std::cout<<"No point, skip this scan!\n"<<std::endl;
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified()) //如果迭代动态共享的卡尔曼滤波器更新失败。
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    solve_start = omp_get_wtime();
                        
                    if (publish_odometry_without_downsample)//false
                    {
                        /******* Publish odometry *******/
                        // publish_odometry(pubOdomAftMapped);
                        publish_odometry(pubOdomAftMapped, tf_broadcaster);//发布里程计数据。
                        if (runtime_pos_log)//false
                        {
                            state_out = kf_output.x_;
                            euler_cur = SO3ToEuler(state_out.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose() \
                            <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++)//对时间序列中的每个点进行遍历。
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];//获取本体坐标系中的点。
                        PointType &point_world_j = feats_down_world->points[idx+j+1];//获取世界坐标系中的点。
                        pointBodyToWorld(&point_body_j, &point_world_j);//将点从本体坐标系转换到世界坐标系。
                    }
                
                    solve_time += omp_get_wtime() - solve_start;
    
                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                }
            }
            else//如果使用IMU作为输入。
            {
                bool imu_prop_cov = false;//定义一个布尔变量，用于标记是否进行了IMU协方差的传播。
                effct_feat_num = 0;//设置有效特征点的数量为0。

                double pcl_beg_time = Measures.lidar_beg_time;//记录激光雷达数据开始的时间。
                idx = -1;//初始化索引变量 idx 为-1。
                for (k = 0; k < time_seq.size(); k++)//对时间序列进行遍历。
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];//获取当前处理的点云点。
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;//计算当前点的时间戳。
                    if (is_first_frame)//如果是第一帧。
                    {
                        while (time_current > get_time_sec(imu_next.header.stamp)) //如果当前时间大于下一个IMU数据的时间戳，进行循环。
                        {
                            imu_last = imu_next;//
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            // imu_deque.pop();
                        }
                        imu_prop_cov = true;//设置IMU协方差传播的标志为真。
                        // imu_upda_cov = true;

                        is_first_frame = false;//设置第一帧标志为假。
                        t_last = time_current;//记录最后一次的时间为当前时间。
                        time_update_last = time_current; 
                        // if(prop_at_freq_of_imu)
                        {
                            input_in.gyro<<imu_last.angular_velocity.x,
                                        imu_last.angular_velocity.y,
                                        imu_last.angular_velocity.z;
                                            
                            input_in.acc<<imu_last.linear_acceleration.x,
                                        imu_last.linear_acceleration.y,
                                        imu_last.linear_acceleration.z;
                            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                        }
                    }
                    
                    // while (time_current > imu_next.header.stamp.toSec()) // && !imu_deque.empty())
                    while (time_current > get_time_sec(imu_next.header.stamp)) // && !imu_deque.empty())//当IMU数据到来时，进行循环。
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z; 
                        input_in.acc    = input_in.acc * G_m_s2 / acc_norm; 
                        // double dt = imu_last.header.stamp.toSec() - t_last;
                        double dt = get_time_sec(imu_last.header.stamp) - t_last;

                        // if(!prop_at_freq_of_imu)
                        // {       
                        // double dt_cov = imu_last.header.stamp.toSec() - time_update_last;
                        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); //进行卡尔曼滤波器的预测步骤。
                            // time_update_last = imu_last.header.stamp.toSec(); //time_current;
                            time_update_last = get_time_sec(imu_last.header.stamp); //time_current;
                        }
                            kf_input.predict(dt, Q_input, input_in, true, false); 
                        // t_last = imu_last.header.stamp.toSec();
                        t_last = get_time_sec(imu_last.header.stamp);//更新最后一次的时间。
                        imu_prop_cov = true;//设置IMU协方差传播的标志为真。
                        // imu_upda_cov = true;
                    }      

                    double dt = time_current - t_last;//计算时间差。
                    t_last = time_current;//更新最后一次的时间为当前时间。
                    double propag_start = omp_get_wtime();//记录传播开始的时间。
                    
                    if(!prop_at_freq_of_imu)
                    {   
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {    
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); //进行卡尔曼滤波器的预测步骤。
                            time_update_last = time_current; 
                        }
                    }
                        kf_input.predict(dt, Q_input, input_in, true, false); 

                    propag_time += omp_get_wtime() - propag_start;//计算并累加传播所花费的时间。

                    double t_update_start = omp_get_wtime();//记录更新开始的时间。
                    
                    if (feats_down_size < 1)//如果降采样后的特征点数量小于1。
                    {
                        // ROS_WARN("No point, skip this scan!\n");
                        std::cout<<"No point, skip this scan!\n"<<std::endl;

                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified()) //如果迭代动态共享的卡尔曼滤波器更新失败。
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }

                    solve_start = omp_get_wtime();
                    if (publish_odometry_without_downsample)//false
                    {
                        /******* Publish odometry *******/

                        // publish_odometry(pubOdomAftMapped);
                        publish_odometry(pubOdomAftMapped, tf_broadcaster);//发布里程计数据。
                        if (runtime_pos_log)//false
                        {
                            state_in = kf_input.x_;
                            euler_cur = SO3ToEuler(state_in.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose() \
                            <<" "<<state_in.bg.transpose()<<" "<<state_in.ba.transpose()<<" "<<state_in.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }

                    for (int j = 0; j < time_seq[k]; j++)//对时间序列中的每个点进行遍历。
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];//获取本体坐标系中的点。
                        PointType &point_world_j = feats_down_world->points[idx+j+1];//获取世界坐标系中的点。
                        pointBodyToWorld(&point_body_j, &point_world_j); //将点从本体坐标系转换到世界坐标系。
                    }
                    solve_time += omp_get_wtime() - solve_start;//计算并累加解算所花费的时间。
                
                    update_time += omp_get_wtime() - t_update_start;//计算并累加更新所花费的时间。
                    idx = idx + time_seq[k];//更新索引变量 idx。
                }  
            }

            /******* Publish odometry downsample *******/
            if (!publish_odometry_without_downsample)//false 如果不发布未经下采样的里程计数据。
            {
                publish_odometry(pubOdomAftMapped, tf_broadcaster);//发布里程计数据。
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();//记录当前时间点。
            
            if(feats_down_size > 4)//如果降采样后的特征点数量大于4。
            {
                // map_incremental();//执行地图的增量更新。
            }

            t5 = omp_get_wtime();//记录当前时间点。
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);//如果启用了路径发布，则发布路径。
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);//如果启用了扫描发布或点云保存，则发布世界坐标系下的扫描帧。
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);//如果同时启用了扫描发布和本体坐标系扫描发布，则发布本体坐标系下的扫描帧。
            static_map_to_odom_.header.stamp = this->get_clock()->now();//设置静态地图到里程计的时间戳。

            static_map_to_odom_.child_frame_id = "odom";
            static_map_to_odom_.header.frame_id = "map";
            /*** Debug variables Logging ***/
            if (runtime_pos_log)//如果启用了运行时位置记录。 false
            {
                frame_num ++;//帧数增加。
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;//计算平均时间消耗
                {
                    aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + update_time/frame_num;//计算ICP的平均时间。
                }
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;//计算匹配的平均时间。
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + solve_time/frame_num;//计算解算的平均时间。
                aver_time_propag = aver_time_propag * (frame_num - 1)/frame_num + propag_time / frame_num;//计算传播的平均时间。
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu, aver_time_icp, aver_time_propag); //打印调试信息。
                if (!publish_odometry_without_downsample)//如果不发布未经下采样的里程计数据。
                {
                    if (!use_imu_as_input)//如果不使用IMU作为输入。
                    {
                        state_out = kf_output.x_;//获取卡尔曼滤波器的输出状态。
                        euler_cur = SO3ToEuler(state_out.rot);//将旋转转换为欧拉角。
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose() \
                        <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;//将状态信息写入文件。
                    }
                    else//如果使用IMU作为输入。
                    {
                        state_in = kf_input.x_;//获取卡尔曼滤波器的输入状态。
                        euler_cur = SO3ToEuler(state_in.rot);//将旋转转换为欧拉角。
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose() \
                        <<" "<<state_in.bg.transpose()<<" "<<state_in.ba.transpose()<<" "<<state_in.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;//将状态信息写入文件。
                    }
                }
                dump_lio_state_to_log(fp);//将LIO（激光雷达惯导融合）的状态记录到日志。
            }
        }
    }
    void map_publish_callback()
    {
        // std::cout << "map_publish_callback()" << std::endl;
        publish_map(pubLaserCloudMap);
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;

//------------------------------------------------------------------------------------------------------
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr range_sensor_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> static_broadcaster_;
    FILE *fp;
    ofstream fout_out, fout_imu_pbp;

    int frame_num = 0;
    double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;
    std::time_t startTime, endTime;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::Matrix<double, 24, 24> Q_input;
    Eigen::Matrix<double, 30, 30> Q_output;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    // signal(SIGINT, SigHandle);
    rclcpp::spin(std::make_shared<LaserMappingNode>());

    if (rclcpp::ok())
        rclcpp::shutdown();

    // readParameters(nh);
    cout<<"lidar_type: "<<lidar_type<<endl;
 
    //--------------------------save map-----------------------------------
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    // fout_out.close();
    // fout_imu_pbp.close();

    return 0;
}
