import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class OdomPositionSubscriber(Node):
    def __init__(self):
        super().__init__('odom_position_subscriber')

        # Inizializza la variabile per la posizione precedente
        self.previous_position = None

        # Crea il publisher per la distanza
        self.distance_publisher = self.create_publisher(Float64, '/distance_travelled', 10)

        # Crea la sottoscrizione al topic odom che restituisce la posizione
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        # Estrai la posizione corrente dal messaggio
        current_position = msg.pose.pose.position
        # Estrai l'orientamento (quaternione)
        orientation = msg.pose.pose.orientation
        # Converti il quaternione in angolo theta (rotazione attorno all'asse Z)
        theta_z = 2 * math.atan2(orientation.x, orientation.w) # rad

        self.get_logger().info(f'Position - x: {current_position.x}, y: {current_position.y}, theta: {theta_z}')

        # Se non è la prima lettura, calcola la distanza altrimenti setta a zero
        if self.previous_position is not None:
            distance = self.calculate_distance(self.previous_position, current_position)
        else:
            distance = 0.0

        self.get_logger().info(f'Distance travelled: {distance:.2f} meters')

        # Pubblica la distanza
        self.distance_publisher.publish(Float64(data=distance))
        
        # Memorizza la posizione corrente come posizione precedente per il prossimo calcolo
        self.previous_position = current_position

    def calculate_distance(self, pos1, pos2):
        # Calcola la distanza euclidea tra due punti 3D
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
