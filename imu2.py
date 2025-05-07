#!/usr/bin/env python3.10
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import serial
import math


class IMUNode(Node):
    def __init__(self):
        super().__init__('arduino_imu_node')

        # Seri port ayarları
        self.serial_port = '/dev/ttyACM1'  # Arduino'nun bağlı olduğu port
        self.baud_rate = 115200  # Baud hızını 115200'e artırdık
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Serial port {self.serial_port} opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port {self.serial_port}: {e}")
            return

        # Odometry mesajı yayıncısı
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)

        # Zamanlayıcı (10 Hz frekansta yayın yapar)
        self.timer = self.create_timer(0.1, self.publish_odom_data)

    def publish_odom_data(self):
        """Seri porttan veri okur ve /odom topic'ine yayın yapar."""
        if self.ser.in_waiting > 0:
            try:
                # Seri porttan bir satır veri oku
                line = self.ser.readline().decode('utf-8').strip()

                # Veriyi virgüllerle ayır
                data = list(map(float, line.split(", ")))

                if len(data) == 9:
                    # Odometry mesajını oluştur
                    odom_msg = Odometry()
                    odom_msg.header = Header()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_footprint'

                    # Orientation (quaternion)
                    quaternion = self.euler_to_quaternion(data[0], data[1], data[2])  # Roll, Pitch, Yaw
                    odom_msg.pose.pose.orientation.x = quaternion[0]
                    odom_msg.pose.pose.orientation.y = quaternion[1]
                    odom_msg.pose.pose.orientation.z = quaternion[2]
                    odom_msg.pose.pose.orientation.w = quaternion[3]

                    # Position (x, y, z) - Bu verileri sensörden ya da hesaplamalardan alabilirsiniz
                    odom_msg.pose.pose.position.x = 0.0  # Pozisyon verisini sensörlerden ya da hesaplamalardan alabilirsiniz
                    odom_msg.pose.pose.position.y = 0.0
                    odom_msg.pose.pose.position.z = 0.0

                    # Angular Velocity (Gyro verisi)
                    odom_msg.twist.twist.angular.x = data[3]  # Gyro X
                    odom_msg.twist.twist.angular.y = data[4]  # Gyro Y
                    odom_msg.twist.twist.angular.z = data[5]  # Gyro Z

                    # Linear Velocity (Accel verisi)
                    odom_msg.twist.twist.linear.x = data[6]  # Accel X
                    odom_msg.twist.twist.linear.y = data[7]  # Accel Y
                    odom_msg.twist.twist.linear.z = data[8]  # Accel Z

                    # Covariance (Varsayılan belirsizlik değerleri)
                    odom_msg.pose.covariance = [0.0] * 36
                    odom_msg.twist.covariance = [0.0] * 36

                    # Mesajı yayınla
                    self.publisher_.publish(odom_msg)
                    self.get_logger().info(f"Published Odometry data: {odom_msg}")

            except Exception as e:
                self.get_logger().error(f"Error processing serial data: {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Euler açılarını quaternion'a dönüştürür."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    imu_node = IMUNode()

    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info("Shutting down IMU node.")
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
