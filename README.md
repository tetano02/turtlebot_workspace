# TurtleBot Navigation System

Questo progetto implementa un sistema di navigazione autonoma utilizzando ROS2 e Nav2. Il robot TurtleBot è configurato per muoversi in un ambiente simulato (e potenzialmente reale) seguendo obiettivi specifici e salvando i dati di navigazione per l'analisi.

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
![alt text](https://github.com/tetano02/turtlebot_workspace/blob/main/Comandi in ORDINE.png?raw=true)
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
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   ```
2. Aggiungere i repository e installare ROS2:
   ```bash
   sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list
   sudo apt update
   sudo apt install ros-humble-desktop ros-dev-tools
   ```

---

## Installazione dei pacchetti necessari
1. Installare i pacchetti Nav2:
   ```bash
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```
2. Installare i pacchetti TurtleBot per Gazebo:
   ```bash
   sudo apt install ros-humble-turtlebot3-gazebo
   ```
Per consentire il corretto avviamento, avviare il progetto di base
```bash
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
---

## Avvio della simulazione
1. Preparare l'ambiente:
   ```bash
   source /opt/ros/humble/setup.bash
   export TURTLEBOT3_MODEL=waffle
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
   ```
2. Buildare il workspace:
   ```bash
   cd turtlebot_workspace
   colcon build
   source install/setup.bash
   ```
3. Avviare la simulazione:
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

## Configurazione della mappa
1. Convertire la planimetria in formato `.pgm` e `.yaml`.
2. Configurare Gazebo con i file di riferimento e aggiungere i modelli scaricati.

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

## Contatti
Per informazioni o segnalazioni di problemi, contattare il team di sviluppo: 
- [Stefano Agnelli](https://github.com/tetano02)
- [Michele Giovanelli]()
- [Wen Wen Sun]()
