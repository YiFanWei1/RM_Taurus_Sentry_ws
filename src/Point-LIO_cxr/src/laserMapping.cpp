#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include"rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include"nav_msgs/msg/odometry.hpp"
#include"nav_msgs/msg/path.hpp"
#include"visualization_msgs/msg/marker.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include"sensor_msgs/msg/point_cloud2.hpp"
#include<tf2/transform_datatypes.h>
#include<tf2_ros/transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include"geometry_msgs/msg/vector3.hpp"
#include"livox_ros_driver2/msg/custom_msg.h"
#include "parameters.h"
#include "Estimator.h"
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
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;
bool   lidar_pushed = false, flg_reset = false, flg_exit = false;
vector<BoxPointType> cub_needrm;
deque<PointCloudXYZI::Ptr>  lidar_buffer;
deque<double>               time_buffer;
deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
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
double stamp_;
sensor_msgs::msg::PointCloud2 pcd_map_;
PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
pcl::PCLPointCloud2 cloudBlob;
int points_cache_size = 0;
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
int process_increments = 0;
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
bool is_first_frame__ = true;
bool is_first_kf_ = true;
int sleep_time = 0;
PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string &filename){
    pcl::io::loadPCDFile(filename, cloudBlob);
    pcl::fromPCLPointCloud2(cloudBlob, *cloud);
    pcl::toROSMsg(*cloud,pcd_map_);
    return cloud;
}
double get_time_sec(const builtin_interfaces::msg::Time &time)
{
    return rclcpp::Time(time).seconds();
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
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                  
    if (use_imu_as_input)
    {
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2)); 
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                       
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                       
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2)); 
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));
        fprintf(fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1), kf_input.x_.gravity(2));
    }
    else
    {
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                       
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));
        fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));
        fprintf(fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1), kf_output.x_.gravity(2));
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
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}
void lasermap_fov_segment()
{
    cub_needrm.shrink_to_fit();
    V3D pos_LiD;
    if (use_imu_as_input)
    {
        pos_LiD = kf_input.x_.pos + kf_input.x_.rot * Lidar_T_wrt_IMU;
    }
    else
    {
        pos_LiD = kf_output.x_.pos + kf_output.x_.rot * Lidar_T_wrt_IMU;
    }
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.emplace_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;
    points_cache_collect();
    if(cub_needrm.size() > 0) int kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
}
void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }
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
            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; 
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
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr &msg) 
{
    auto curr_time = std::chrono::high_resolution_clock::now();
    auto msg_time_chrono = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::seconds(msg->header.stamp.sec) + 
    std::chrono::nanoseconds(msg->header.stamp.nanosec));
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (get_time_sec(msg->header.stamp) < last_timestamp_lidar)
    {
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        return;
    }
    last_timestamp_lidar = get_time_sec(msg->header.stamp);
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    PointCloudXYZI::Ptr  ptr_div(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    double time_div = get_time_sec(msg->header.stamp);
    if (cut_frame)
    {
        sort(ptr->points.begin(), ptr->points.end(), time_list);

        for (int i = 0; i < ptr->size(); i++)
        {
            ptr_div->push_back(ptr->points[i]);
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
            time_buffer.push_back(time_div);
        }
    }
    else if (con_frame)
    {
        if (frame_ct == 0)
        {
            time_con = last_timestamp_lidar; 
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
        time_buffer.emplace_back(get_time_sec(msg->header.stamp));
    }
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}
void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr &msg_in) 
{    
    rclcpp::Time time_;
    stamp_ = (float)msg_in->header.stamp.sec;
    time_ = msg_in->header.stamp;
    publish_count ++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));
    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_lag_imu_to_lidar);
    double timestamp = get_time_sec(msg->header.stamp);
    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        std::cout<<"imu loop back, clear deque"<<std::endl;
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
        // cout << "lidar_buffer.empty() || imu_deque.empty()" << endl;
        return false;
    }
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
    if (p_imu->imu_need_init_)
    {
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
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
        double imu_time = get_time_sec(imu_deque.front()->header.stamp);
        meas.imu.shrink_to_fit();
        meas.imu.emplace_back(imu_last_ptr);
        while ((!imu_deque.empty()) && (imu_time < lidar_end_time))
        {
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
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    
        for(int i = 0; i < feats_down_size; i++)
        {            
            if (!Nearest_Points[i].empty())
            {
                const PointVector &points_near = Nearest_Points[i];
                bool need_add = true;
                PointType downsample_result, mid_point; 
                mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
                    if (fabs(points_near[0].x - mid_point.x) > 0.866 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 0.866 * filter_size_map_min || fabs(points_near[0].z - mid_point.z) > 0.866 * filter_size_map_min)
                    {
                        PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
                        continue;
                    }
                    float dist  = calc_dist<float>(feats_down_world->points[i],mid_point);
                    for (int readd_i = 0; readd_i < points_near.size(); readd_i ++)
                    {
                            if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) 
                            {
                                need_add = false;
                                break;
                            }
                    }
                    if (need_add) PointToAdd.emplace_back(feats_down_world->points[i]);
            }
            else
            {
                    PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
            }
        }
    int add_point_size = ikdtree.Add_Points(PointToAdd, true);
    // std::cout << "add_point_size: " << add_point_size << std::endl;
    int no_need_down_num_ = ikdtree.Add_Points(PointNoNeedDownsample, false);
    // std::cout << "no_need_down_num_: " << no_need_down_num_ << std::endl;
}
void publish_init_kdtree(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    int size_init_ikdtree = ikdtree.size();
    PointCloudXYZI::Ptr   laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    PointVector ().swap(ikdtree.PCL_Storage);
    ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
    laserCloudInit->points = ikdtree.PCL_Storage;
    pcl::toROSMsg(*laserCloudInit, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudFullRes->publish(laserCloudmsg);
}
void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
        int size = laserCloudFullRes->points.size();

        PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));
        
        for (int i = 0; i < size; i++)
        {
            laserCloudWorld->points[i].x = feats_down_world->points[i].x;
            laserCloudWorld->points[i].y = feats_down_world->points[i].y;
            laserCloudWorld->points[i].z = feats_down_world->points[i].z;
            laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFullRes->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }
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
        *pcl_wait_save += *laserCloudWorld;
        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
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
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "lidar_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap)//发布Lasermap
{
    bool dense_pub_en = false;
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);//feats_down_body特征点
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyToWorld(&laserCloudFullRes->points[i], \
                            &laserCloudWorld->points[i]);
    }
    if(is_first_frame__)
    {
        *pcl_wait_pub += *cloud;
        is_first_frame__ = false;
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
    // pubLaserCloudMap->publish(pcd_map_);
    pubLaserCloudMap->publish(laserCloudmsg);
}
template<typename T>
void set_posestamp(T & out)
{
    if (!use_imu_as_input)
    {   
        if(is_first_kf_)
        {
            if(task == "A")
            {
                kf_output.x_.pos(0) = init_x_;
                kf_output.x_.pos(1) = init_y_;
                kf_output.x_.pos(2) = init_z_;
                out.position.x = kf_output.x_.pos(0);
                out.position.y = kf_output.x_.pos(1);
                out.position.z = kf_output.x_.pos(2);
                // Eigen::Quaterniond q(kf_output.x_.rot);
                // out.orientation.x = q.coeffs()[0];
                // out.orientation.y = q.coeffs()[1];
                // out.orientation.z = q.coeffs()[2];
                // out.orientation.w = q.coeffs()[3];
                // is_first_kf_ = false;
                Eigen::Quaterniond q(cos(M_PI / 2), 0, 0, sin(M_PI / 2));
                // 将四元数转换为旋转矩阵
                Eigen::Matrix3d rot_matrix = q.normalized().toRotationMatrix();
                // 将旋转矩阵转换为 MTK::SO3<double> 类型
                MTK::SO3<double> so3_rot(rot_matrix);
                // 将转换后的旋转矩阵赋值给 kf_output.x_.rot
                kf_output.x_.rot = so3_rot;
                // Eigen::Quaterniond q(kf_output.x_.rot);
                // out.orientation.x = q.coeffs()[0];
                // out.orientation.y = q.coeffs()[1];
                // out.orientation.z = q.coeffs()[2];
                // out.orientation.w = q.coeffs()[3];
                Eigen::Quaterniond q_updated(so3_rot); // 使用更新后的 kf_output.x_.rot 构造四元数
                out.orientation.x = q_updated.coeffs()[0];
                out.orientation.y = q_updated.coeffs()[1];
                out.orientation.z = q_updated.coeffs()[2];
                out.orientation.w = q_updated.coeffs()[3];
                is_first_kf_ = false; 

            }else if(task == "B")
            {
                kf_output.x_.pos(0) = init_x_;
                kf_output.x_.pos(1) = init_y_;
                kf_output.x_.pos(2) = init_z_;
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
void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped, std::unique_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
    odomAftMapped.header.frame_id = "odom";
    odomAftMapped.child_frame_id = "lidar_link";
    if (publish_odometry_without_downsample)
    {
        odomAftMapped.header.stamp = get_ros_time(time_current);
    }
    else
    {
        odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    }
    set_posestamp(odomAftMapped.pose.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transformStamped.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transformStamped.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transformStamped.header.stamp = rclcpp::Time(odomAftMapped.header.stamp);
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "lidar_link";
    // std::cout << "a"<<std::endl;
    tf_br->sendTransform(transformStamped);
}
void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_body_pose.header.frame_id = "odom";
    static int jjj = 0;
    jjj++;
    {
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
        pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
        pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
        pubPath = this->create_publisher<nav_msgs::msg::Path>("/path", 100000);
        plane_pub = this->create_publisher<visualization_msgs::msg::Marker>("planner_normal",1000);
        static_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 100.0));  // 1ms
        timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms, std::bind(&LaserMappingNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Node init finished.");
        // auto map_period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0));       // 1s
        // map_pub_timer_ = rclcpp::create_timer(this, this->get_clock(), map_period_ms, std::bind(&LaserMappingNode::map_publish_callback, this));//发布Lasermap
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

        this->declare_parameter<std::string>("gcl.task","C");
        this->get_parameter_or<std::string>("gcl.task",task,"C");
        RCLCPP_INFO(this->get_logger(), "gcl.task:%s",task.c_str());

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
        this->declare_parameter<bool>("pcd_save.use_pcd_map_", true);
        this->declare_parameter<std::string>("pcd_save.prior_PCD_map_path", "");
        this->declare_parameter<double>("init_pos.init_x", 0.0);
        this->declare_parameter<double>("init_pos.init_y", 0.0);
        this->declare_parameter<double>("init_pos.init_z", 0.0);
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
        this->get_parameter_or<bool>("pcd_save.use_pcd_map_", use_pcd_map_, true);
        this->get_parameter_or<std::string>("pcd_save.prior_PCD_map_path", priorPCDMapPath,"");
        this->get_parameter_or<double>("init_pos.init_x", init_x_, 0.0);
        this->get_parameter_or<double>("init_pos.init_y", init_y_, 0.0);
        this->get_parameter_or<double>("init_pos.init_z", init_z_, 0.0);
    }
    void timer_callback()
    {
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                flg_first_scan = false;
                cout << "first lidar time" << first_lidar_time << endl;
            }
            if (flg_reset)
            {
                std::cout<<"reset when rosbag play back"<<std::endl;
                p_imu->Reset();
                flg_reset = false;
                return;
            }
            double t0,t1,t2,t3,t4,t5,match_start, solve_start;
            match_time = 0;
            solve_time = 0;
            propag_time = 0;
            update_time = 0;
            t0 = omp_get_wtime();
            p_imu->Process(Measures, feats_undistort);
            if (p_imu->imu_need_init_)
            {
                return;
            }
            if(imu_en)
            {
                if (!p_imu->gravity_align_)
                {
                    while (Measures.lidar_beg_time > get_time_sec(imu_next.header.stamp))
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                    }
                    if (non_station_start)
                    {
                        state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc *= -1;
                    }
                    else 
                    {
                        state_in.gravity =  -1 * p_imu->mean_acc * G_m_s2 / acc_norm; 
                        state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm; 
                        state_out.acc = p_imu->mean_acc * G_m_s2 / acc_norm;
                    }
                    if (gravity_align)
                    {
                        Eigen::Matrix3d rot_init;
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                        p_imu->Set_init(state_in.gravity, rot_init);
                        state_in.gravity = p_imu->gravity_;
                        state_out.gravity = p_imu->gravity_;
                        state_in.rot = rot_init;
                        state_out.rot = rot_init;
                        state_out.acc = -rot_init.transpose() * state_out.gravity;
                    }
                    kf_input.change_x(state_in);
                    kf_output.change_x(state_out);
                    p_imu->gravity_align_ = true;
                }
            }
            else 
            {
                if (!p_imu->gravity_align_)
                {
                    state_in.gravity << VEC_FROM_ARRAY(gravity_init);
                    if (gravity_align)
                    {
                        Eigen::Matrix3d rot_init;
                        p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
                        p_imu->Set_init(state_in.gravity, rot_init);
                        state_out.gravity = p_imu->gravity_;
                        state_out.rot = rot_init;
                        state_out.acc = -rot_init.transpose() * state_out.gravity;
                    }
                    else
                    {
                        state_out.gravity << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc << VEC_FROM_ARRAY(gravity_init);
                        state_out.acc *= -1;
                    }
                    kf_output.change_x(state_out);
                    p_imu->gravity_align_ = true;
                }
            }
            lasermap_fov_segment();
            t1 = omp_get_wtime();
            if(space_down_sample)
            {
                downSizeFilterSurf.setInputCloud(feats_undistort);
                downSizeFilterSurf.filter(*feats_down_body);
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
            }
            else
            {
                feats_down_body = Measures.lidar;
                sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            }
            time_seq = time_compressing<int>(feats_down_body);
            feats_down_size = feats_down_body->points.size();
            if(!init_map)
            {
                if(ikdtree.Root_Node == nullptr)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                }
                    
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                }
                for (size_t i = 0; i < feats_down_world->size(); i++) 
                {
                    init_feats_world->points.emplace_back(feats_down_world->points[i]);
                }
                if(init_feats_world->size() < init_map_size) return;
                if(use_pcd_map_)
                {
                    PointCloudXYZI::Ptr map_cloud_(new PointCloudXYZI());
                    map_cloud_ = loadPointcloudFromPcd(priorPCDMapPath);
                    ikdtree.Build(map_cloud_->points); 
                }else{
                    ikdtree.Build(init_feats_world->points);
                }
                init_map = true;
                publish_init_kdtree(pubLaserCloudMap);
                std::cout << "publish_init_kdtree(pubLaserCloudMap);" << std::endl;
                return;
            }
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            t2 = omp_get_wtime();
            crossmat_list.reserve(feats_down_size);
            pbody_list.reserve(feats_down_size);
            for (size_t i = 0; i < feats_down_body->size(); i++)
            {
                V3D point_this(feats_down_body->points[i].x,
                            feats_down_body->points[i].y,
                            feats_down_body->points[i].z);
                pbody_list[i]=point_this;
                if (extrinsic_est_en)
                {
                    if (!use_imu_as_input)
                    {
                        point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
                    }
                    else
                    {
                        point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
                    }
                }
                else
                {
                    point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
                }
                M3D point_crossmat;
                point_crossmat << SKEW_SYM_MATRX(point_this);
                crossmat_list[i]=point_crossmat;
            }
            if (!use_imu_as_input)
            {     
                effct_feat_num = 0;
                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame)
                    {
                        if(imu_en)
                        {
                            while (time_current > get_time_sec(imu_next.header.stamp))
                            {
                                imu_last = imu_next;
                                imu_next = *(imu_deque.front());
                                imu_deque.pop_front();
                            }

                            angvel_avr<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                            acc_avr   <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
                        }
                        is_first_frame = false;
                        time_update_last = time_current;
                        time_predict_last_const = time_current;
                    }
                    if(imu_en)
                    {
                        bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                        while (imu_comes)
                        {
                            angvel_avr<<imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
                            acc_avr   <<imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;
                            imu_last = imu_next;
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                            double dt = get_time_sec(imu_last.header.stamp) - time_predict_last_const;
                            kf_output.predict(dt, Q_output, input_in, true, false);
                            time_predict_last_const = get_time_sec(imu_last.header.stamp);
                            imu_comes = time_current > get_time_sec(imu_next.header.stamp);
                            {
                                double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last; 
                                if (dt_cov > 0.0)
                                {
                                    time_update_last = get_time_sec(imu_last.header.stamp);
                                    double propag_imu_start = omp_get_wtime();
                                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                                    propag_time += omp_get_wtime() - propag_imu_start;
                                    double solve_imu_start = omp_get_wtime();
                                    kf_output.update_iterated_dyn_share_IMU();
                                    solve_time += omp_get_wtime() - solve_imu_start;
                                }
                            }
                        }
                    }
                    double dt = time_current - time_predict_last_const;
                    double propag_state_start = omp_get_wtime();
                    if(!prop_at_freq_of_imu)
                    {
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_output.predict(dt_cov, Q_output, input_in, false, true);
                            time_update_last = time_current;   
                        }
                    }
                    kf_output.predict(dt, Q_output, input_in, true, false);
                    propag_time += omp_get_wtime() - propag_state_start;
                    time_predict_last_const = time_current;
                    double t_update_start = omp_get_wtime();

                    if (feats_down_size < 1)
                    {
                        std::cout<<"No point, skip this scan!\n"<<std::endl;
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_output.update_iterated_dyn_share_modified())
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    solve_start = omp_get_wtime();
                        
                    if (publish_odometry_without_downsample)
                    {
                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log)
                        {
                            state_out = kf_output.x_;
                            euler_cur = SO3ToEuler(state_out.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose() \
                            <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }
                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j);
                    }
                    solve_time += omp_get_wtime() - solve_start;
                    update_time += omp_get_wtime() - t_update_start;
                    idx += time_seq[k];
                }
            }
            else
            {
                bool imu_prop_cov = false;
                effct_feat_num = 0;

                double pcl_beg_time = Measures.lidar_beg_time;
                idx = -1;
                for (k = 0; k < time_seq.size(); k++)
                {
                    PointType &point_body  = feats_down_body->points[idx+time_seq[k]];
                    time_current = point_body.curvature / 1000.0 + pcl_beg_time;
                    if (is_first_frame)
                    {
                        while (time_current > get_time_sec(imu_next.header.stamp)) 
                        {
                            imu_last = imu_next;//
                            imu_next = *(imu_deque.front());
                            imu_deque.pop_front();
                        }
                        imu_prop_cov = true;
                        is_first_frame = false;
                        t_last = time_current;
                        time_update_last = time_current; 
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
                    while (time_current > get_time_sec(imu_next.header.stamp))
                    {
                        imu_last = imu_next;
                        imu_next = *(imu_deque.front());
                        imu_deque.pop_front();
                        input_in.gyro<<imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
                        input_in.acc <<imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z; 
                        input_in.acc    = input_in.acc * G_m_s2 / acc_norm; 
                        double dt = get_time_sec(imu_last.header.stamp) - t_last;
                        double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
                        if (dt_cov > 0.0)
                        {
                            kf_input.predict(dt_cov, Q_input, input_in, false, true); 
                            time_update_last = get_time_sec(imu_last.header.stamp);
                        }
                            kf_input.predict(dt, Q_input, input_in, true, false); 
                        t_last = get_time_sec(imu_last.header.stamp);
                        imu_prop_cov = true;
                    }      
                    double dt = time_current - t_last;
                    t_last = time_current;
                    double propag_start = omp_get_wtime(); 
                    if(!prop_at_freq_of_imu)
                    {   
                        double dt_cov = time_current - time_update_last;
                        if (dt_cov > 0.0)
                        {    
                            kf_input.predict(dt_cov, Q_input, input_in, false, true);
                            time_update_last = time_current; 
                        }
                    }
                    kf_input.predict(dt, Q_input, input_in, true, false); 
                    propag_time += omp_get_wtime() - propag_start;
                    double t_update_start = omp_get_wtime();
                    if (feats_down_size < 1)
                    {
                        std::cout<<"No point, skip this scan!\n"<<std::endl;
                        idx += time_seq[k];
                        continue;
                    }
                    if (!kf_input.update_iterated_dyn_share_modified())
                    {
                        idx = idx+time_seq[k];
                        continue;
                    }
                    solve_start = omp_get_wtime();
                    if (publish_odometry_without_downsample)
                    {
                        publish_odometry(pubOdomAftMapped, tf_broadcaster);
                        if (runtime_pos_log)
                        {
                            state_in = kf_input.x_;
                            euler_cur = SO3ToEuler(state_in.rot);
                            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose() \
                            <<" "<<state_in.bg.transpose()<<" "<<state_in.ba.transpose()<<" "<<state_in.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                        }
                    }
                    for (int j = 0; j < time_seq[k]; j++)
                    {
                        PointType &point_body_j  = feats_down_body->points[idx+j+1];
                        PointType &point_world_j = feats_down_world->points[idx+j+1];
                        pointBodyToWorld(&point_body_j, &point_world_j); 
                    }
                    solve_time += omp_get_wtime() - solve_start;
                    update_time += omp_get_wtime() - t_update_start;
                    idx = idx + time_seq[k];
                }  
            }

            if (!publish_odometry_without_downsample)
            {
                publish_odometry(pubOdomAftMapped, tf_broadcaster);
            }
            t3 = omp_get_wtime();
            if(use_pcd_map_)
            {
                if(feats_down_size > 4)
                {
                    sleep_time = sleep_time + 1;
                    if(sleep_time > 200)
                    {
                        // std::cout << "start" <<std::endl;
                        map_incremental();
                    }
                }

            }
            else
            {
                if(feats_down_size > 4)
                {
                    map_incremental();
                }
            }
            t5 = omp_get_wtime();
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            if (runtime_pos_log)
            {
                frame_num ++;
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                {
                    aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + update_time/frame_num;
                }
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + solve_time/frame_num;
                aver_time_propag = aver_time_propag * (frame_num - 1)/frame_num + propag_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = aver_time_consu;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu, aver_time_icp, aver_time_propag); //打印调试信息。
                if (!publish_odometry_without_downsample)
                {
                    if (!use_imu_as_input)
                    {
                        state_out = kf_output.x_;
                        euler_cur = SO3ToEuler(state_out.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_out.pos.transpose() << " " << state_out.vel.transpose() \
                        <<" "<<state_out.omg.transpose()<<" "<<state_out.acc.transpose()<<" "<<state_out.gravity.transpose()<<" "<<state_out.bg.transpose()<<" "<<state_out.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;//将状态信息写入文件。
                    }
                    else
                    {
                        state_in = kf_input.x_;
                        euler_cur = SO3ToEuler(state_in.rot);
                        fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_in.pos.transpose() << " " << state_in.vel.transpose() \
                        <<" "<<state_in.bg.transpose()<<" "<<state_in.ba.transpose()<<" "<<state_in.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;//将状态信息写入文件。
                    }
                }
                dump_lio_state_to_log(fp);
            }
        }
    }
    void map_publish_callback()
    {
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
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
    rclcpp::spin(std::make_shared<LaserMappingNode>());
    if (rclcpp::ok())
        rclcpp::shutdown();
    cout<<"lidar_type: "<<lidar_type<<endl;
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }
    return 0;
}
