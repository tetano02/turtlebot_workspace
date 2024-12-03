import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import time

class OdomPositionSubscriber(Node):
    def __init__(self):
        super().__init__('odom_position_subscriber')

        # Initialize the variable for the previous position
        self.previous_position = None

        # Create the publisher for distance
        self.distance_publisher = self.create_publisher(Float64, '/distance_travelled', 10)

        # Create subscription to the odom topic that returns position
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Variables for throttling log output
        self.publish_count = 0
        self.last_log_time = time.time()
        # Distanza
        self.distance = 0

    def odom_callback(self, msg):
        # Extract the current position from the message
        current_position = msg.pose.pose.position

        # Increment the count of publishes in this interval
        self.publish_count += 1

        # Calculate distance traveled
        if self.previous_position is not None:
            self.distance += self.calculate_distance(self.previous_position, current_position)
        else:
            self.distance = 0.0

        # Throttle log output to every 1 second - it's annoying seeing it too often
        current_time = time.time()
        if current_time - self.last_log_time >= 0.95:
            # Publish the distance
            self.distance_publisher.publish(Float64(data=self.distance))
            # Log the position, distance, and number of publishes in the past second
            self.get_logger().info(
                f'Distance travelled: {self.distance:.2f} meters | '
                f'Publishes in last second: {self.publish_count}'
            )
            # Reset counter and timestamp
            self.publish_count = 0
            self.last_log_time = current_time
            # Reset distanza
            self.distance = 0.0

        # Store the current position as previous position for the next calculation
        self.previous_position = current_position

    def calculate_distance(self, pos1, pos2):
        # Calculate the Euclidean distance between two 3D points
        return math.sqrt(
            (pos2.x - pos1.x) ** 2 + 
            (pos2.y - pos1.y) ** 2
        )

def main(args=None):
    rclpy.init(args=args)
    node = OdomPositionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
