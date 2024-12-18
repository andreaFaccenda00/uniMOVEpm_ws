#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <fstream>


class SimpleOdometry : public rclcpp::Node
{
public:
    SimpleOdometry(const std::string& name);
    ~SimpleOdometry();


private:
    void velCallback(const geometry_msgs::msg::TwistStamped &msg);

    void jointCallback(const sensor_msgs::msg::JointState &msg);


    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    // Odometry
    double wheelbase_;
    double x_;
    double y_;
    double theta_;
    rclcpp::Time prev_time_;
    double linear_;
    double angular_;
    nav_msgs::msg::Odometry odom_msg_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_stamped_;
    std::ofstream log_file_;
};

#endif // SIMPLE_ODOMETRY_HPP