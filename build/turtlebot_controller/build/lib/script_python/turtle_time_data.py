import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # Tipo di messaggio per ricevere il tempo di percorrenza

class TravelTimeSubscriber(Node):
    def __init__(self):
        super().__init__('travel_time_subscriber')
        
        # Crea un subscriber per il topic 'travel_time'
        self.subscription = self.create_subscription(
            Float32,          # Tipo di messaggio
            '/travel_time',    # Nome del topic
            self.time_callback,  # Funzione di callback
            10                # QoS per la gestione della coda di messaggi
        )
        
        self.subscription  # Mantiene il riferimento al subscriber per evitare che venga cancellato

    def time_callback(self, msg):
        # Stampa il tempo di percorrenza ricevuto
        self.get_logger().info(f'Tempo di percorrenza ricevuto: {msg.data:.2f} secondi')

def main():
    rclpy.init()
    
    # Istanzia il nodo subscriber
    time_subscriber = TravelTimeSubscriber()
    
    # Mantiene il nodo in esecuzione per ricevere i messaggi
    rclpy.spin(time_subscriber)
    
    # Chiude l'istanza di ROS 2
    rclpy.shutdown()

if __name__ == '__main__':
    main()
