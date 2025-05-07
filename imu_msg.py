#!/usr/bin/env python3.10
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import serial
import re
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class ArduinoImuPublisher(Node):
    def __init__(self):
        super().__init__('arduino_imu_publisher')
        
        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Arduino'nun bağlı olduğu seri port
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.publish_imu_data)
        
        self.get_logger().info('Arduino IMU Publisher Node with TF Started')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Euler açılarını quaternion'a dönüştürür.
        roll, pitch, yaw: X, Y, Z açıları (radyan cinsinden)
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return qx, qy, qz, qw

    def publish_imu_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                match = re.match(r"X : ([\d\.\-]+)\s+Y : ([\d\.\-]+)\s+Z : ([\d\.\-]+)", line)
                
                if match:
                    angle_x = float(match.group(1))
                    angle_y = float(match.group(2))
                    angle_z = float(match.group(3))
                    
                    # Açıları radian cinsine çevir
                    roll = math.radians(angle_x)
                    pitch = math.radians(angle_y)
                    yaw = math.radians(angle_z)
                    
                    # Euler açılarını quaternion'a dönüştür
                    qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)

                    # IMU Mesajı
                    imu_msg = Imu()
                    imu_msg.header = Header()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'
                    
                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw  

                    imu_msg.linear_acceleration.x = 0.0
                    imu_msg.linear_acceleration.y = 0.0
                    imu_msg.linear_acceleration.z = 0.0
                    
                    imu_msg.angular_velocity.x = 0.0
                    imu_msg.angular_velocity.y = 0.0
                    imu_msg.angular_velocity.z = 0.0
                    
                    self.publisher_.publish(imu_msg)

                    # **TF Yayını (IMU Link için map referansına bağlanması)**
                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'base_link'  # Harita referansı (map) kullanılıyor
                    t.child_frame_id = 'imu_link'  # IMU linki
                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = qx
                    t.transform.rotation.y = qy
                    t.transform.rotation.z = qz
                    t.transform.rotation.w = qw

                    self.tf_broadcaster.sendTransform(t)

                    self.get_logger().info(f'Published IMU Data: X={angle_x}, Y={angle_y}, Z={angle_z}')
            except Exception as e:
                self.get_logger().error(f"Error parsing serial data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

