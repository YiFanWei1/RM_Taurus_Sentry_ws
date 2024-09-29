#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h" 
// #include "tf/transform_datatypes.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/twist.hpp"
// #include "robot_msgs/srv/decision.hpp"
#include "robot_msgs/srv/decision.hpp"
// #include "robot_msgs/srv/Decision.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
class pid_follow_ : public rclcpp::Node {
public:
    pid_follow_(const rclcpp::NodeOptions & options) : Node("pid_follow_", options) {
        // 订阅全局路径
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", 10, std::bind(&pid_follow_::pathCallback, this, std::placeholders::_1));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        auto period = std::chrono::milliseconds(50);
        timer_ = this->create_wall_timer(period,std::bind(&pid_follow_::timerCallback,this));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
        decision_server_ = 
        this->create_service<robot_msgs::srv::Decision>(
            "decision_server",
            std::bind(&pid_follow_::serverCallback, this,
            std::placeholders::_1,std::placeholders::_2));
    }

private:
    void serverCallback(
    const std::shared_ptr<robot_msgs::srv::Decision::Request> request,
    std::shared_ptr<robot_msgs::srv::Decision::Response> response) 
    {
        RCLCPP_INFO(this->get_logger(),"serverCallback");
        // RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a,
        //             request->b);
        // response->sum = request->a + request->b;
    }
    void timerCallback(){
        // RCLCPP_INFO(this->get_logger(),"timerCallback()");
        try{
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("map","base_link",tf2::TimePointZero);
            robot_pose_.position.x = transform.transform.translation.x;
            robot_pose_.position.y = transform.transform.translation.y;
            tf2::Quaternion q(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            );
            tf2::Matrix3x3 m(q);
            m.getRPY(robot_roll_,robot_pitch_,robot_yaw_);
            // RCLCPP_INFO(this->get_logger(),"Current yaw: %f, Current x: %f, Current y: %f",robot_yaw_,robot_pose_.position.x,robot_pose_.position.y);
        }catch(tf2::TransformException  &ex){
            // RCLCPP_ERROR(this->get_logger(),"Failure %s",ex.what());
        }
        if(rec_plan_)
        {
            //发布
            cmd_pub_->publish(cmd);
            rec_plan_ = false;

            // RCLCPP_INFO(this->get_logger(), "cmd_vel_x:  %f", cmd.linear.x);
            // RCLCPP_INFO(this->get_logger(), "cmd_vel_y:  %f", cmd.linear.y);
        }
        else
        {
            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.angular.z = 0;
            cmd_pub_->publish(cmd);

            // RCLCPP_INFO(this->get_logger(), "cmd_vel_x:  %f", cmd.linear.x);
            // RCLCPP_INFO(this->get_logger(), "cmd_vel_y:  %f", cmd.linear.y);
        }
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (msg->poses.empty()) {
            // RCLCPP_INFO(this->get_logger(), "Received an empty path.");
            return;
        }
        // 获取路径上的下一个点
        const auto& target_pose = msg->poses[5].pose.position;
        // RCLCPP_INFO(this->get_logger(), "target_pose.x: %f", (msg->poses[msg->poses.size()-1].pose.position.x));
        // RCLCPP_INFO(this->get_logger(), "target_pose.y: %f", (msg->poses[msg->poses.size()-1].pose.position.y));
        // RCLCPP_INFO(this->get_logger(), "robot_pose_.position.x: %f", robot_pose_.position.x);
        // RCLCPP_INFO(this->get_logger(), "robot_pose_.position.y: %f", robot_pose_.position.y);
        dist_diff_ = hypot(robot_pose_.position.x-(msg->poses[msg->poses.size()-1].pose.position.x),robot_pose_.position.y-msg->poses[msg->poses.size()-1].pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "dist_diff_: %f", dist_diff_);

        if(dist_diff_<-100 ||dist_diff_>100 || dist_diff_ < 0.35){//错误判断和目标点忍耐判断
            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.angular.z = 0;
            rec_plan_ = true;
        }
        
        // 计算偏航角差
        yaw_diff = calculateYawDiff(target_pose, robot_yaw_, robot_pose_);
        vx_global = 1 * cos(yaw_diff) * 1;
        vy_global = 1 * sin(yaw_diff) * 1;
        if(dist_diff_ > 0.65){
            cmd.linear.x = 2*vx_global;
            cmd.linear.y = 2*vy_global;
            rec_plan_ = true;
        }
        else if(dist_diff_<0.65&&dist_diff_>0.35)
        {
            cmd.linear.x = 0.2*vx_global;
            cmd.linear.y = 0.2*vy_global;
            cmd.angular.z = 0;
            rec_plan_ = true;
        }

    }


    double calculateYawDiff(const geometry_msgs::msg::Point& target_pose, double robot_yaw, const geometry_msgs::msg::Pose& robot_pose) {
        double target_yaw = atan2(target_pose.y - robot_pose.position.y, target_pose.x - robot_pose.position.x);
        double yaw_diff = target_yaw - robot_yaw;
        // 规范化角度差到 [-π, π] 范围
        while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
        while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI; 
        // RCLCPP_INFO(this->get_logger(), "target_yaw %f", target_yaw);
        // RCLCPP_INFO(this->get_logger(), "robot_yaw %f", robot_yaw);
        // RCLCPP_INFO(this->get_logger(), "robot_pose.position.x %f", robot_pose.position.x);
        // RCLCPP_INFO(this->get_logger(), "robot_pose.position.y %f", robot_pose.position.y);
        // RCLCPP_INFO(this->get_logger(), "target_yaw %f", target_yaw);
        // RCLCPP_INFO(this->get_logger(), "dist_diff_ %f", dist_diff_);

        return yaw_diff;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    geometry_msgs::msg::Pose robot_pose_;
    double robot_yaw_;
    double robot_roll_;
    double robot_pitch_;
    double yaw_diff ;
    double dist_diff_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener;
    double vx_global ;
    double vy_global ;
    geometry_msgs::msg::Twist cmd;
    bool rec_plan_ = false;
    rclcpp::Service<robot_msgs::srv::Decision>::SharedPtr decision_server_;
};
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pid_follow_);