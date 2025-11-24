TurtleBot3 Workspace con Comandi Vocali

Questo è un fork del workspace TurtleBot3 arricchito con un sistema di controllo vocale e script di installazione automatica per ROS2 Humble.

🚀 Installazione e Avvio Rapido

Non serve configurare manualmente il workspace. Ho creato uno script che installa le dipendenze, compila il codice e avvia la simulazione automaticamente.

1. Clona la repository

git clone [https://github.com/TUO_USERNAME/turtlebot_workspace.git](https://github.com/TUO_USERNAME/turtlebot_workspace.git)
cd turtlebot_workspace


2. Avvia tutto

Esegui semplicemente lo script universale:

bash avvio/Avvio.sh


Lo script farà tutto da solo:

✅ Controllerà e installerà le dipendenze (rosdep).

🏗️ Compilerà il workspace (colcon build).

🐢 Avvierà Gazebo e Rviz.

🎙️ Avvierà i nodi di controllo (incluso il comando vocale).

🎙️ Comandi Vocali Disponibili

Una volta avviato il nodo vocale (terminale "Comando Vocale"), puoi dire:

"Vieni qui": Il robot verrà verso di te (o eseguirà la logica programmata).

(Aggiungi qui altri comandi se ne hai)

🛠️ Requisiti

Ubuntu 22.04

ROS2 Humble
dipendenze:
pip install google-generativeai python-dotenv
