#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PointStamped, TransformStamped

import dwm1001
import serial


class ActiveTagNode(Node):
    def __init__(self) -> None:
        super().__init__("dwm_active", allow_undeclared_parameters=True)

        self._declare_parameters()

        serial_port_param = self.get_parameter("serial_port").value
        self.get_logger().info(f"Provided serial port: '{serial_port_param}'")

        serial_handle = self._open_serial_port(serial_port_param)
        self.dwm_handle = dwm1001.ActiveTag(serial_handle)

        self.dwm_handle.start_position_reporting()

        tag_topic = f"~/output/{self.get_parameter('tag_id').value}"
        self.point_publisher = self.create_publisher(PointStamped, tag_topic, 1)
        self.timer = self.create_timer(1 / 25, self.timer_callback)

    def _open_serial_port(self, serial_port: str) -> serial.Serial:
        if not serial_port:
            self._shutdown_fatal("No serial port specified.")

        try:
            serial_handle = serial.Serial(serial_port, baudrate=115_200)
        except serial.SerialException:
            self._shutdown_fatal(f"Could not open serial port '{serial_port}'.")

        self.get_logger().info(f"Opened serial port: '{serial_port}'.")

        return serial_handle

    def _shutdown_fatal(self, message: str) -> None:
        self.get_logger().fatal(message + " Shutting down.")
        exit()

    def _declare_parameters(self):
        
        serial_port_descriptor = ParameterDescriptor(
            description="Device file or COM port associated with DWM1001",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        
        tag_id_descriptor = ParameterDescriptor(
            description="The ID for the particular DWM1001 device.",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )

        self.declare_parameter("tag_id", "", tag_id_descriptor)
        self.declare_parameter("serial_port", "", serial_port_descriptor)


    def timer_callback(self):
        try:
            tag_position = self.dwm_handle.position
        except dwm1001.ParsingError:
            self.get_logger().warn("Could not parse position report. Skipping it.")
            return

        time_stamp = self.get_clock().now().to_msg()
        msg = PointStamped()

        msg.header.stamp = time_stamp
        msg.header.frame_id = "dwm1001"
        msg.point.x = tag_position.x_m
        msg.point.y = tag_position.y_m
        msg.point.z = tag_position.z_m

        self.point_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    active_tag = ActiveTagNode()
    rclpy.spin(active_tag)

    active_tag.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()