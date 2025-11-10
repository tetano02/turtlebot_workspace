#1️⃣ Chiudere la simulazione precedente
#Se hai ancora nodi aperti dai terminali precedenti:
#Premi Ctrl + C nel terminale dove hai eseguito lo script.

#Chiudi tutti i terminali separati dei nodi (quelli aperti dallo script).

#2️⃣ Assicurarsi che il workspace sia pronto
# Vai al workspace
cd ~/turtlebot_workspace
# (Opzionale) Build del workspace se non l’hai già fatto
colcon build
# Sorgente dell’ambiente
source install/setup.bash

#3️⃣ Installare le librerie per il riconoscimento vocale
# Installa SpeechRecognition e PocketSphinx
pip install --user SpeechRecognition pocketsphinx

#4️⃣ Creare il nodo vocale
# Vai nella cartella turtlebot_controller:
cd ~/turtlebot_workspace/src/turtlebot_controller/script_python
# Crea il file voice_interface.py:
nano voice_interface.py
# Incolla il codice seguente nel file:
GNU nano 6.2                                                  voice_interface.py                                                            
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
class VoiceInterface(Node):
    def __init__(self):
        super().__init__('voice_interface')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.get_logger().info("Nodo vocale pronto, in ascolto...")
        self.timer = self.create_timer(1.0, self.listen_loop)
    def listen_loop(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            try:
                audio = self.recognizer.listen(source, timeout=3)
                text = self.recognizer.recognize_sphinx(audio)
                self.get_logger().info(f"Riconosciuto: {text}")
                if "vieni qui" in text.lower():
                    msg = String()
                    msg.data = "vieni_qui"
                    self.publisher_.publish(msg)
                    self.get_logger().info("✅ Comando 'Vieni qui' inviato!")
            except sr.WaitTimeoutError:
                pass
            except sr.UnknownValueError:
                pass
            except sr.RequestError as e:
                self.get_logger().error(f"Errore riconoscimento vocale: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = VoiceInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
# Ctrl+o per salvare
# Ctrl+x per uscire

#5: Modifica del nodo AB
#Apri un terminale e digita:
cd ~/turtlebot_workspace/src/turtlebot_controller/script_python
#Qui dovresti trovare tutti i file Python dei tuoi nodi, incluso move_turtle_AB.py.
#Verifica con:
ls
#Apri il file con un editor da terminale. Puoi usare nano (semplice da terminale):
nano move_turtle_AB.py
 GNU nano 6.2                                                   move_turtle_AB.py                                                            
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
import time

# File esterni
from script_python.turtle_estimate_position import publish_initial_pose
from script_python.patient_button_control import button

class GoalNavigation(Node): 
    def __init__(self):
        super().__init__('goal_navigation')

        # Action client per navigazione
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Publisher per stato di avanzamento
        self.move_status_publisher = self.create_publisher(Bool, '/move_status', 10)

        # Subscriber per comandi vocali
        self.voice_subscriber = self.create_subscription(
            String,
            '/voice_command',
            self.voice_callback,
            10
        )
        # Flag destinazione
        self.at_destination = False
    # Home position
        self.home_x = -2.5
        self.home_y = -2.5
        self.home_theta = 0.0

        self.get_logger().info("Nodo di navigazione pronto.")

    # Callback comandi vocali
    def voice_callback(self, msg):
        if msg.data == "vieni_qui":
            self.get_logger().info("🚀 Comando vocale ricevuto: muovo il robot verso il paziente")
            # Goal predefinito verso il paziente
            self.send_goal(-1.0, 3.5, 0.0)

    # Funzione per inviare goal
    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta  # semplice orientamento

        self.get_logger().info(f'Invio goal: x={x}, y={y}, theta={theta}')
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
# Callback risposta del goal
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rifiutato')
            return

        # Pubblica inizio movimento
        self.get_logger().info('Goal accettato')
        msg = Bool()
        msg.data = True
        self.move_status_publisher.publish(msg)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Callback risultato
    def get_result_callback(self, future):
        result = future.result().result

        if not self.at_destination:
            # Arrivato alla destinazione
            self.get_logger().info('Arrivato alla destinazione!')
            msg = Bool()
            msg.data = False
            self.move_status_publisher.publish(msg)

            self.at_destination = True
            button(self.at_destination)  # Simula interazione con il paziente
            # Torna a casa
            self.send_goal(self.home_x, self.home_y, self.home_theta)
else:
            # Arrivato a casa
            self.get_logger().info('Arrivato a casa!')
            msg = Bool()
            msg.data = False
            self.move_status_publisher.publish(msg)
            rclpy.shutdown()


def main():
    rclpy.init()
    publish_initial_pose()
    time.sleep(2)

    navigator = GoalNavigation()
    rclpy.spin(navigator)


if __name__ == '__main__':
    main()
# Ctrl+o per salvare
# Ctrl+x per uscire


#6 Ricostruisci il workspace. Dalla root del workspace:
cd ~/turtlebot_workspace
colcon build
source install/setup.bash
# Adesso il nodo move_turtle_AB riceverà i comandi dal topic vocale /voice_command.
