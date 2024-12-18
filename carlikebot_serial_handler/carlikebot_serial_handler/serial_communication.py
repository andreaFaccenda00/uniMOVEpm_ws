#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from carlikebot_custom_msgs.msg import Serial
import serial
from sensor_msgs.msg import Imu
import time

def verify_values(value_list):
    return None not in value_list and all(v.strip() for v in value_list)

class SerialHandler(Node):

    def __init__(self):
        super().__init__('serial_pub')

        self.subscriber_serial = self.create_subscription(
            Serial,
            'serial_message',
            self.listener_callback,
            10
        )
        speed = 115200
        self.ser = serial.Serial("/dev/ttyAMA2", speed, timeout=60)
        self.stop_flag = False
        self.reset_flag = False
        self.imu_output = self.create_publisher(Imu, '/imu', 1)
        self.timer_period = 1.0 / 5
        self.timer = self.create_timer(self.timer_period, self.publish_imu_msg)
        self.get_logger().info('Serial ready')


    def publish_imu_msg(self):
        while self.ser.in_waiting > 0:
            raw_data = self.ser.readline()
            try:
                line = raw_data.decode('utf-8').strip()
                data = line.split(',')
                if len(data) == 10 and verify_values(data):
                    imuMsg = Imu()
                    imuMsg.header.stamp = self.get_clock().now().to_msg()
                    imuMsg.header.frame_id = 'imu_link'
                    imuMsg.orientation.w = float(data[9]) # encoder
                    imuMsg.orientation.x = float(data[6]) # magnetometer
                    imuMsg.orientation.y = float(data[7]) # magnetometer
                    imuMsg.orientation.z = float(data[8]) # magnetometer
                    imuMsg.angular_velocity.x = float(data[3]) # calibration
                    imuMsg.angular_velocity.y = float(data[4])
                    imuMsg.angular_velocity.z = float(data[5])
                    imuMsg.linear_acceleration.x = float(data[0])
                    imuMsg.linear_acceleration.y = float(data[1])
                    imuMsg.linear_acceleration.z = float(data[2])
                    self.imu_output.publish(imuMsg)
                else:
                    print("Data not valid")
            except UnicodeDecodeError:
                continue
            except ValueError:
                continue

    def _del_(self):
        self.ser.close()

    def listener_callback(self, msg):
        try:
            if self.stop_flag:
                self.get_logger().info(f'Stop engine')
                data = [0, 0.0, 100.0]
                serial_msg = self.string_formatter(data)
                self.ser.write(bytes(serial_msg, 'ascii'))
            elif self.reset_flag:
                self.get_logger().info(f'Reset pid')
                data = [3, 0.0, 100.0]
                serial_msg = self.string_formatter(data)
                self.ser.write(bytes(serial_msg, 'ascii'))
            else:
                data = [msg.flag, msg.vel, msg.radius]
                serial_msg = self.string_formatter(data)
                self.ser.write(bytes(serial_msg, 'ascii'))
            #   self.get_logger().info(f'Publishing {serial_msg}')
        except Exception as e:
            self.get_logger().error(f'Writing on serial error {e}')

    def string_formatter(self, data):
        converted_list = [str("%.3f" % element) for element in data]
        csv_string = ",".join(converted_list)
        return csv_string+'\n'
    
def main(args=None):
    rclpy.init(args=args)

    serial_handler = SerialHandler()
    rclpy.spin(serial_handler)

    serial_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()