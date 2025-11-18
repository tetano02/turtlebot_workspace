import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# --- Blocco per sopprimere l'output di errore ---
import os
import sys
import contextlib

@contextlib.contextmanager
def suppress_stderr():
    """Un blocco 'with' per sopprimere stderr (avvisi ALSA/Jack)."""
    # Salva il descrittore di file stderr originale
    original_stderr_fd = os.dup(sys.stderr.fileno())
    # Apri /dev/null
    devnull_fd = os.open(os.devnull, os.O_WRONLY)
    
    # Reindirizza stderr a /dev/null
    os.dup2(devnull_fd, sys.stderr.fileno())
    
    try:
        yield
    finally:
        # Ripristina stderr originale
        os.dup2(original_stderr_fd, sys.stderr.fileno())
        # Chiudi i descrittori di file
        os.close(devnull_fd)
        os.close(original_stderr_fd)
# --- Fine blocco soppressione ---


# Importa la libreria rumorosa *dentro* il blocco di soppressione
with suppress_stderr():
    import speech_recognition as sr

class VoiceInterface(Node):
    def __init__(self):
        super().__init__('voice_interface')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.recognizer = sr.Recognizer()
        
        # Inizializza il microfono *dentro* il blocco di soppressione
        with suppress_stderr():
            # Forziamo l'uso del microfono PulseAudio (Index 9)
            self.microphone = sr.Microphone(device_index=9)
        
        # Ora i log di ROS funzioneranno normalmente
        self.get_logger().info("Nodo vocale pronto, in ascolto...")
        self.timer = self.create_timer(1.0, self.listen_loop)

    def listen_loop(self):
        with self.microphone as source:
            try:
                # Aumentato a 1.0s per una migliore precisione
                self.recognizer.adjust_for_ambient_noise(source, duration=1.0)
                
                # Sopprime gli errori anche durante l'ascolto
                with suppress_stderr():
                    audio = self.recognizer.listen(source, timeout=3)
                
                text = self.recognizer.recognize_google(audio, language="it-IT")
                
                self.get_logger().info(f"Riconosciuto: {text}")
                
                if "vieni qui" in text.lower():
                    msg = String()
                    msg.data = "vieni_qui"
                    self.publisher_.publish(msg)
                    self.get_logger().info("✅ Comando 'Vieni qui' inviato!")
            
            except sr.WaitTimeoutError:
                pass # Non dire nulla se c'è silenzio
            except sr.UnknownValueError:
                pass # Non dire nulla se non capisce
            except sr.RequestError as e:
                self.get_logger().error(f"Errore servizio Google: {e}")
            except Exception as e:
                self.get_logger().error(f"Errore microfono: {e}")

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
