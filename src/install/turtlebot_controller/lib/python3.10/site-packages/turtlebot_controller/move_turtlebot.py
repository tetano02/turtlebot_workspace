#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleBotController(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')
        # Publisher per il topic /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Crea un timer per pubblicare i messaggi ogni 0.5 secondi
        timer_period = 0.5  # secondi
        self.timer = self.create_timer(timer_period, self.move_robot)

    def move_robot(self):
        # Crea un messaggio Twist per muovere il robot
        msg = Twist()
        # Imposta la velocità lineare e angolare
        mov_avanti = 0.2
        #mov_lato = 0.5
        mov_ruota_z = 0.0
        msg.linear.x = mov_avanti  # Avanti a 0.2 m/s
        msg.angular.z = mov_ruota_z # Nessuna rotazione
        #msg.linear.y = mov_lato
        # Pubblica il messaggio
        self.publisher_.publish(msg)
        self.get_logger().info(f'Movimento: Avanti a {mov_avanti} m/s.')

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
