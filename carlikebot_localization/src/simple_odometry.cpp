#include "carlikebot_localization/simple_odometry.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <fstream>


using std::placeholders::_1;

SimpleOdometry::SimpleOdometry(const std::string& name)
    : Node(name)
    , wheelbase_(0.26)
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , linear_(0.0)
    , angular_(0.0)
{
    // Apri il file in modalità di aggiunta
    log_file_.open("odometry_log.txt", std::ios::out | std::ios::app);
    if (!log_file_.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Impossibile aprire il file per la scrittura.");
    }

    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "bicycle_steering_controller/reference", 10,
        std::bind(&SimpleOdometry::velCallback, this, _1));

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&SimpleOdometry::jointCallback, this, _1));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_link";
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "base_link";

    prev_time_ = get_clock()->now();
}

SimpleOdometry::~SimpleOdometry()
{
    if (log_file_.is_open())
    {
        log_file_.close();
    }
}

void SimpleOdometry::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    linear_ = msg.twist.linear.x;
    angular_ = msg.twist.angular.z;
}

void SimpleOdometry::jointCallback(const sensor_msgs::msg::JointState &state)
{
    rclcpp::Time msg_time = state.header.stamp;
    rclcpp::Duration dt_ = msg_time - prev_time_;

    if (linear_ == 0)
    {
        prev_time_ = state.header.stamp;
        return;
    }
    prev_time_ = state.header.stamp;

    double steering_angle = std::atan(angular_ * wheelbase_ / linear_);
    theta_ += (linear_ / wheelbase_) * std::tan(steering_angle) * dt_.seconds();
    x_ += linear_ * std::cos(theta_) * dt_.seconds();
    y_ += linear_ * std::sin(theta_) * dt_.seconds();

    // Scrivi i dati nel file
    if (log_file_.is_open())
    {
        log_file_ << "linear: " << linear_
                  << ", steering_angle: " << steering_angle
                  << ", dt: " << dt_.seconds() << "\n";
    }

    // Compose and publish the odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg_.header.stamp = get_clock()->now();
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.getX();
    odom_msg_.pose.pose.orientation.y = q.getY();
    odom_msg_.pose.pose.orientation.z = q.getZ();
    odom_msg_.pose.pose.orientation.w = q.getW();
    odom_msg_.twist.twist.linear.x = linear_;
    odom_msg_.twist.twist.angular.z = angular_;
    odom_pub_->publish(odom_msg_);

    // TF
    transform_stamped_.transform.translation.x = x_;
    transform_stamped_.transform.translation.y = y_;
    transform_stamped_.transform.rotation.x = q.getX();
    transform_stamped_.transform.rotation.y = q.getY();
    transform_stamped_.transform.rotation.z = q.getZ();
    transform_stamped_.transform.rotation.w = q.getW();
    transform_stamped_.header.stamp = get_clock()->now();
    transform_broadcaster_->sendTransform(transform_stamped_);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleOdometry>("simple_odometry");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
