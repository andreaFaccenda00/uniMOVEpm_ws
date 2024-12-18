#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler

class SimpleOdometry(Node):

    def __init__(self):
        super().__init__("simple_odometry")

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        self.wheelbase_ = 0.26
        self.dt_= 0.1
        self.vel_sub_ = self.create_subscription(TwistStamped, "bicycle_steering_controller/reference", self.velCallback, 10)   
        self.odom_pub_ = self.create_publisher(Odometry, "/odom", 10)

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_link"

    def velCallback(self, msg):
        
        linear = msg.twist.linear.x 
        angular = msg.twist.angular.z 
        if linear == 0:
            return 0.0 
        steering_angle = math.atan(angular * self.wheelbase_ / linear)
    
        # Update position
        self.x_ += linear * math.cos(self.theta_) * self.dt_
        self.y_ += linear * math.sin(self.theta_) * self.dt_
        self.theta_+= (linear / self.wheelbase_) * math.tan(steering_angle) * self.dt_

        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular
        self.odom_pub_.publish(self.odom_msg_)

        # Publish TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    simple_odometry = SimpleOdometry()
    rclpy.spin(simple_odometry)
    
    simple_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



