# TurtleBot Navigation System

Questo progetto implementa un sistema di navigazione autonoma utilizzando ROS2 e Nav2. Il robot TurtleBot è configurato per muoversi in un ambiente simulato (e potenzialmente reale) seguendo obiettivi specifici e salvando i dati di navigazione per l'analisi.
## Simulazione
[Guarda il video demo](robot_simulation.mp4)

---
## Indice
1. [Introduzione](#introduzione)
2. [Prerequisiti](#prerequisiti)
3. [Configurazione di ROS2](#configurazione-di-ros2)
4. [Installazione dei pacchetti necessari](#installazione-dei-pacchetti-necessari)
5. [Importazione del progetto](#importazione-del-progetto)
6. [Avvio della simulazione](#avvio-della-simulazione)
7. [Struttura dei nodi](#struttura-dei-nodi)
8. [Configurazione della mappa](#configurazione-della-mappa)
9. [Fasi della simulazione](#fasi-della-simulazione)
10. [Sviluppi futuri](#sviluppi-futuri)

---
## Visualizzazione comandi da eseguire
![alt text](https://github.com/tetano02/turtlebot_workspace/blob/main/comandi.png?raw=true)
---
## Introduzione
Il progetto combina robotica e supporto alla mobilità per assistere pazienti in un contesto riabilitativo. Include la configurazione di ROS2, Nav2, RViz, e Gazebo per la simulazione del TurtleBot.

---

## Prerequisiti
- Ubuntu 22.04 con ROS2 Humble installato.
- `WSL2` (se su Windows).
- Pacchetti Python necessari installati (vedi [requirements.txt](#)).

---

## Configurazione di ROS2
### Installazione di Ubuntu
1. Abilitare WSL:
   ```bash
   wsl --install
   ```
2. Installare Ubuntu Jellyfish:
   ```bash
   wsl --install -d Ubuntu-22.04
   ```
3. Configurare l'ambiente:
   ```bash
   wsl -d Ubuntu-22.04
   ```

### Installazione di ROS2
1. Configurare UTF-8:
   ```bash
   locale  # check for UTF-8
   
   sudo apt update && sudo apt install locales
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   
   locale  # verify settings
   ```
2. Assicurati che Ubuntu Universal Repository sia attivata
   ```bash
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   ```
3. Aggiungi la chiave GPG di Ros2
   ```bash
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```
4. Aggiungi la repository alla lista sorgente:
   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```
5. Prima di avviare l'installazione, check di tutti i pacchetti:
   ```bash
   sudo apt update
   sudo apt upgrade
   ```
6. Installazione di ROS2 e di pacchetti aggiuntivi:
   ```bash
   sudo apt install ros-humble-desktop
   sudo apt install ros-dev-tools
   ```
7. Esegui questo comando per configurare l’ambiente ROS 2 nella tua shell
corrente
   ```bash
   source /opt/ros/humble/setup.bash
   ```
---

## Installazione dei pacchetti necessari Nav2
1. Installare i pacchetti Nav2:
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```
2. Installare i pacchetti TurtleBot per Gazebo:
   ```bash
   sudo apt install ros-humble-turtlebot3-gazebo
   ```
Per consentire il corretto avviamento, avviare il progetto di base per sicurezza
```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=waffle
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
   source /usr/share/gazebo/setup.bash
   ros2 launch nav2_bringup tb3_simulation_launch.py
   ```
In caso non funzionasse, riavviare la piattaforma su cui si sta lavorando.

---
## Importazione del progetto
1. Clonare la repository GitHub:
   ```bash
   git clone https://github.com/tetano02/turtlebot_workspace.git
   ```
2. Esportare i file nelle directory di Nav2:
   ```bash
   sudo mkdir /opt/ros/humble/share/turtlebot3_gazebo/models/planimetria_santanna
   sudo cp -r ~/turtlebot_workspace/src/turtlebot_controller/models/planimetria_santanna/* /opt/ros/humble/share/turtlebot3_gazebo/models/planimetria_santanna
   ```
3. Sostituire il file delle texture Gazebo:
   ```bash
   sudo rm /usr/share/gazebo-11/media/materials/scripts/gazebo.material
   sudo cp ~/turtlebot_workspace/src/turtlebot_controller/gazebo.material /usr/share/gazebo-11/media/materials/scripts/
   ```
4. Per il corretto funzionamento del progetto, eseguire anche i seguenti comandi
   ```bash
   pip install pandas
   pip upgrade matplotlib
   ```
   Se l'ultimo comando non dovesse funzionare, usare il seguente comando.
   ```bash
   pip install --upgrade matplotlib
   ```
---

## Avvio della simulazione
1. Preparare l'ambiente:
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=waffle
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
   ```
   Se non dovesse funzionare, aggiungere anche questo comando
   ```bash
   source /usr/share/gazebo/setup.bash
   ```
3. Buildare il workspace:
   ```bash
   cd turtlebot_workspace
   colcon build
   source install/setup.bash
   ```
4. Avviare la simulazione:
   ```bash
   ros2 launch turtlebot_controller tb3_santanna_launch.py
   ```
---

## Struttura dei nodi
### Nodo principale: `move_turtle_AB`
- Gestisce la navigazione autonoma da un punto A a un punto B.
- Stima la posizione iniziale e invia obiettivi tramite l'Action Client.

### Nodi di supporto
- `turtle_distance_publisher`: Calcola e pubblica la distanza percorsa.
- `turtle_csv_handler`: Salva i dati di navigazione in formato CSV.

---
## Avvio dei nodi
Per avviare i nodi seguire i seguenti comandi
1. Apri una nuova scheda da terminale.
2. Eseguire il build del workspace:
   ```bash
   cd turtlebot_workspace
   colcon build
   source install/setup.bash
   ```
3. Avvia il nodo che pubblica la distanza:
   Nodo che pubblica la distanza:
   ```bash
   ros2 run turtlebot_controller turtle_distance_publisher
   ```
4. Esegui di nuovo i punti 1 e 2
5. Avvia il nodo che crea i csv con i dati del robot:
   Nodo che crea i csv:
   ```bash
   ros2 run turtlebot_controller turtle_csv_handler
   ```
6. Esegui di nuovo i punti 1 e 2
7. Nodo che permette al robot di muoversi:
   ```bash
   ros2 run turtlebot_controller move_turtle_AB
   ```
---
## Visualizzazione dei dati
Dopo aver eseguito il lancio della simulazione e i nodi, per visualizzare i dati della simulazione è necessario eseguire i seguenti comandi.
```bash
   cd ~/turtlebot_workspace/src/turtlebot_controller/script_python
   python3 elaborate_data.py
```
---
## Configurazione della mappa
1. La planimetria è stata convertita in formato `.pgm` e i parametri settati nel file `.yaml`.
2. Gazebo è stato configurato con i file di riferimento e aggiungendo i modelli scaricati.
---
## Fasi della simulazione
1. **Movimento verso la destinazione**: Il robot si muove verso il letto, salvando i dati.
2. **Ritorno alla base**: Premendo un bottone simulato, il robot torna alla posizione iniziale.
---
## Sviluppi futuri
- Implementazione su hardware reale con:
  - Configurazione del deambulatore.
  - Test in ambienti ospedalieri reali.
- Miglioramenti all'interfaccia utente e all'affidabilità del sistema di navigazione.
---

## Autori e contatti 
Per informazioni o segnalazioni di problemi, contattare il team di sviluppo: 
- [Stefano Agnelli](https://github.com/tetano02)
- [Michele Giovanelli]()
- [Wen Wen Sun](https://github.com/sunwenwen2002)
