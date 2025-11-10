#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ===========================================
# Script per preparare ambiente, buildare workspace
# e avviare la simulazione TurtleBot3 su ROS2 Humble
# ===========================================

set -e  # Interrompe l’esecuzione in caso di errore

echo "==========================================="
echo "🔧 Preparazione dell'ambiente ROS2 Humble..."
echo "==========================================="

# Source ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble caricato."
else
    echo "❌ Errore: /opt/ros/humble/setup.bash non trovato!"
    exit 1
fi

# Esportazione variabili TurtleBot3
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
echo "✅ Variabili d'ambiente impostate."

# Source Gazebo (se necessario)
if [ -f /usr/share/gazebo/setup.bash ]; then
    source /usr/share/gazebo/setup.bash
    echo "✅ Gazebo caricato."
else
    echo "⚠️  /usr/share/gazebo/setup.bash non trovato, si procede comunque."
fi

echo
echo "==========================================="
echo "🏗️  Build del workspace TurtleBot..."
echo "==========================================="

# Controllo esistenza workspace
WORKSPACE_PATH="/home/leo/Desktop/turtlebot_workspace_vocale"
export WORKSPACE_PATH

if [ -d "$WORKSPACE_PATH" ]; then
    cd "$WORKSPACE_PATH"
else
    echo "❌ Errore: directory '$WORKSPACE_PATH' non trovata!"
    exit 1
fi

colcon build --symlink-install

# Source dell’ambiente appena buildato
source install/setup.bash
echo "✅ Workspace buildato e ambiente caricato."

echo
echo "==========================================="
echo "🚀 Avvio della simulazione TurtleBot..."
echo "==========================================="

# Avvio della simulazione
ros2 launch turtlebot_controller tb3_santanna_launch.py > logs_simulazione.txt 2>&1 &
sleep 10  # attende qualche secondo per permettere al simulatore di avviarsi
# Torna nella directory dello script
# Torna nella directory originale dello script
cd "$SCRIPT_DIR"

# Lancia lo script dei nodi
if [ -f "$SCRIPT_DIR/start_nodes.sh" ]; then
    echo "🧠 Avvio dello script dei nodi..."
    bash "$SCRIPT_DIR/start_nodes.sh"
else
    echo "⚠️  Script start_nodes.sh non trovato nella directory: $SCRIPT_DIR"
fi

