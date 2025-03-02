import rclpy
import math
import sys
import termios
import tty
import select
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from msg_srv.srv import Controller

# ค่าความเร็วสูงสุด
LIN_VEL_DEFAULT = 0.22
ANG_VEL_DEFAULT = 2.84

class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_controller')
    
        # Subscriber รับข้อมูลจาก LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/a3lidar',   # ชื่อ topic ของ LiDAR
            self.controller_callback,
            10
        )
        self.srv = self.create_service(Controller, 'stop_robot_service', self.stop_robot_callback)


        # Publisher ส่งคำสั่งควบคุมหุ่นยนต์
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # กำหนดค่าเริ่มต้นของความเร็ว
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.target_lin_vel = 0.0
        self.target_ang_vel = 0.0

        # กำหนดสถานะของสิ่งกีดขวางเป็น attribute ของคลาส
        self.Left_block = True
        self.Right_block = True
        self.Front_block = True
        self.Back_block = True

        self.get_logger().info('✅ Lidar Teleop Node Started!')

    def update_velocity(self, lin, ang):
        """อัปเดตความเร็วและส่งคำสั่งไปยังหุ่นยนต์"""
        self.target_lin_vel = lin
        self.target_ang_vel = ang
        twist = Twist()
        twist.linear.x = self.target_lin_vel
        twist.angular.z = self.target_ang_vel
        self.pub.publish(twist)

    def controller_callback(self, msg):
        obstacle_detected = False
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):  
                continue

            if distance < 0.3:  
                obstacle_detected = True
                angle_radian = msg.angle_min + i * msg.angle_increment
                angle_degree = math.degrees(angle_radian) % 360
                distance_rounded = round(distance, 2)
                self.lin_vel = min(0.0374 / distance, LIN_VEL_DEFAULT)
                self.ang_vel = min(0.4828 / distance, ANG_VEL_DEFAULT)

                self.get_logger().info(f'🚀 Speed: Linear {self.lin_vel:.2f}, Angular {self.ang_vel:.2f}')

                if 230 <= angle_degree < 290 and self.Left_block:
                    self.get_logger().info(f"CCW: {distance_rounded} m, Angle: {angle_degree:.2f}°")
                    self.update_velocity(0.0, self.ang_vel)  
                elif 50 <= angle_degree < 100 and self.Right_block:
                    self.get_logger().info(f"CW: {distance_rounded} m, Angle: {angle_degree:.2f}°")
                    self.update_velocity(0.0, -self.ang_vel)  
                elif 150 <= angle_degree < 210 and self.Front_block:
                    self.get_logger().info(f"Forward: {distance_rounded} m, Angle: {angle_degree:.2f}°")
                    self.update_velocity(self.lin_vel, 0.0)  
                elif (320 <= angle_degree < 360 or 0 <= angle_degree < 15) and self.Back_block:
                    self.get_logger().info(f"Back: {distance_rounded} m")
                    self.update_velocity(-self.lin_vel, 0.0)  
                break

        if not obstacle_detected:
            self.Left_block = True
            self.Right_block = True
            self.Front_block = True
            self.Back_block = True

            self.get_logger().info("🔴 No obstacles detected. Stopping the robot.")
            self.update_velocity(0.0, 0.0)  

    def stop_robot_callback(self, request, response):
        direction = request.direction
        stop = request.distannce
        self.get_logger().info(f"Received stop command with direction: {direction}")  

        if direction == "Left":
            self.update_velocity(0.0, 0.0)
            self.Left_block = False
        elif direction == "Right":
            self.update_velocity(0.0, 0.0)
            self.Right_block = False
        elif direction == "Front":
            self.update_velocity(0.0, 0.0)
            self.Front_block = False
        elif direction == "Back":
            self.update_velocity(0.0, 0.0)
            self.Back_block = False
        else:
            response.success = False
            response.message = f"❌ Invalid direction: {direction}"
            return response

        self.get_logger().info(f"🛑 Stop command received: {direction}")
        response.success = True
        response.message = f"✅ Robot stopped due to {direction} obstacle!"
        self.get_logger().info(f"Response message: {response.message}")  # แสดงข้อความตอบกลับ
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LidarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("❌ Node Stopped by User")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
