import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from msg_srv.srv import Controller  # Import the custom service (controller.srv)
from msg_srv.msg import Robot  # Import the custom message (robot.msg)
import math
from rclpy.qos import qos_profile_sensor_data

class LidarRobot(Node):
    def __init__(self):
        super().__init__('lidar_robot')

        # Subscribe to the Lidar topic with appropriate QoS
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data  # Setting QoS for sensor data
        )

        # Create the service client for the controller service
        self.client = self.create_client(Controller, 'stop_robot_service')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("⏳ Waiting for service 'stop_robot_service'...")

        self.get_logger().info('✅ Lidar Robot Node Started!')

    def listener_callback(self, msg):
        # Process the Lidar data
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue

            angle_radian = msg.angle_min + i * msg.angle_increment
            angle_degree = math.degrees(angle_radian) % 360

            if distance < 0.20:  # If an obstacle is detected
                direction = ''
                if 40 <= angle_degree < 80:
                    self.get_logger().info(f"Left at {angle_degree:.2f}°")
                    direction = 'Left'
                elif 120 <= angle_degree < 160:
                    self.get_logger().info(f"🚨 Back at {angle_degree:.2f}°")
                    direction = 'Back'
                elif 220 <= angle_degree < 260:
                    self.get_logger().info(f"🚨 Right at {angle_degree:.2f}°")
                    direction = 'Right'
                elif 350 <= angle_degree < 360 or 0 <= angle_degree < 10:
                    self.get_logger().info(f"🚨 Front at {angle_degree:.2f}°")
                    direction = 'Front'

                # Send the stop request with direction, distance, and angle
                self.send_stop_request(direction, distance, angle_degree)

    def send_stop_request(self, direction, distance, angle):
        # Create the request object for the service call
        request = Controller.Request()
        request.direction = direction
        request.distance = distance
        request.angle = angle

        # Call the service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info(f"Successfully stopped robot at {response.message}.")
        else:
            self.get_logger().error(f"Failed to stop robot: {response.message}")

def main(args=None):
    rclpy.init(args=args)
    lidar_service = LidarRobot()
    rclpy.spin(lidar_service)

    lidar_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
