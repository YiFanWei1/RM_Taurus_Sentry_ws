#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
// #include <ros/ros.h>
#include"rclcpp/rclcpp.hpp"
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
// #include <nav_msgs/Odometry.h>
#include"nav_msgs/msg/odometry.hpp"
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <tf/transform_broadcaster.h>
#include<tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
// #include <eigen_conversions/eigen_msg.h>

#include <pcl_conversions/pcl_conversions.h>
// #include <sensor_msgs/Imu.h>
#include"sensor_msgs/msg/imu.hpp"
// #include <sensor_msgs/PointCloud2.h>
#include"sensor_msgs/msg/point_cloud2.hpp"
// #include <geometry_msgs/Vector3.h>
#include"geometry_msgs/msg/vector3.hpp"
#include<iostream>
/// *************Preconfiguration

#define MAX_INI_COUNT (100)

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();
  
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::SharedPtr &lastimu);
  void Process(const MeasureGroup &meas, PointCloudXYZI::Ptr pcl_un_);
  void Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot);

  ofstream fout_imu;
  // double first_lidar_time;
  int    lidar_type;
  bool   imu_en;
  V3D mean_acc, gravity_;
  bool   imu_need_init_ = true;
  bool   b_first_frame_ = true;
  bool   gravity_align_ = false;

 private:
  void IMU_init(const MeasureGroup &meas, int &N);
  V3D mean_gyr;
  int    init_iter_num = 1;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), gravity_align_(false)
{
  imu_en = true;
  init_iter_num = 1;
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  // ROS_WARN("Reset ImuProcess");
  std::cout<<"Reset ImuProcess"<<endl;
  mean_acc      = V3D(0, 0, 0.0);
  mean_gyr      = V3D(0, 0, 0);
  imu_need_init_    = true;
  init_iter_num     = 1;
}

void ImuProcess::IMU_init(const MeasureGroup &meas, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  // ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  std::cout << "ImuProcess::IMU_init: IMU Initializing: " << static_cast<double>(N) / MAX_INI_COUNT * 100 << " %" << std::endl;


  V3D cur_acc, cur_gyr;
  
  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
    const auto &imu_acc = meas.imu.front()->linear_acceleration;
    const auto &gyr_acc = meas.imu.front()->angular_velocity;
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;
    mean_gyr      += (cur_gyr - mean_gyr) / N;

    N ++;
  }
}

void ImuProcess::Process(const MeasureGroup &meas, PointCloudXYZI::Ptr cur_pcl_un_)
{  
  if (imu_en)
  {
    if(meas.imu.empty())  return;
    // ROS_ASSERT(meas.lidar != nullptr);
    // std::cerr << "Assertion failed: meas.lidar is not nullptr" << std::endl;
    assert(meas.lidar != nullptr);//然后使用assert()函数判断meas.lidar是否为空指针，如果为空指针，则会触发断言错误。

    if (imu_need_init_)
    {
      /// The very first lidar frame
      IMU_init(meas, init_iter_num);

      imu_need_init_ = true;

      if (init_iter_num > MAX_INI_COUNT)
      {
        // ROS_INFO("IMU Initializing: %.1f %%", 100.0);
        std::cout << "ImuProcess::Process: IMU Initializing: " << 100.0 << " %" << std::endl;
        imu_need_init_ = false;
        *cur_pcl_un_ = *(meas.lidar);
      }
      return;
    }
    // if (!gravity_align_) gravity_align_ = true;
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
  else
  {
    if (imu_need_init_)
    {
      imu_need_init_ = false;
    }
    *cur_pcl_un_ = *(meas.lidar);
    return;
  }
}

void ImuProcess::Set_init(Eigen::Vector3d &tmp_gravity, Eigen::Matrix3d &rot)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  // V3D tmp_gravity = - mean_acc / mean_acc.norm() * G_m_s2; // state_gravity;
  M3D hat_grav;
  hat_grav << 0.0, gravity_(2), -gravity_(1),
              -gravity_(2), 0.0, gravity_(0),
              gravity_(1), -gravity_(0), 0.0;
  double align_norm = (hat_grav * tmp_gravity).norm() / tmp_gravity.norm() / gravity_.norm();
  double align_cos = gravity_.transpose() * tmp_gravity;
  align_cos = align_cos / gravity_.norm() / tmp_gravity.norm();
  if (align_norm < 1e-6)
  {
    if (align_cos > 1e-6)
    {
      rot = Eye3d;
    }
    else
    {
      rot = -Eye3d;
    }
  }
  else
  {
    V3D align_angle = hat_grav * tmp_gravity / (hat_grav * tmp_gravity).norm() * acos(align_cos); 
    rot = Exp(align_angle(0), align_angle(1), align_angle(2));
  }
}