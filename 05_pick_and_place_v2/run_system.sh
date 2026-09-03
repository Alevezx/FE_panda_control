#!/bin/bash

# --- CONFIGURAZIONE ---
# IP del Robot Franka (modifica se necessario)
ROBOT_IP="172.16.0.2"
# Percorso specifico di libfranka sul PC del laboratorio
FRANKA_DIR="/home/panda/Desktop/ALESSANDRO_VECCHIO/libfranka/build"

# Percorsi relativi
PYTHON_SCRIPT="skeleton_yolo_and_transmission.py"
BUILD_DIR="build"
CPP_EXECUTABLE="controllo_ammettenza_pick_and_place"

# --- FUNZIONE DI CLEANUP ---
# Questa funzione viene chiamata quando premi CTRL+C o quando il C++ termina
cleanup() {
    echo ""
    echo "--- Chiusura del sistema ---"
    if [ -n "$PID_PYTHON" ]; then
        echo "Terminazione script Python (PID $PID_PYTHON)..."
        kill $PID_PYTHON
    fi
    exit
}

# Intercetta il segnale di interruzione (CTRL+C) per chiudere anche Python
trap cleanup SIGINT SIGTERM

# --- 0. COMPILAZIONE C++ ---
echo "[0/3] Verifica e Compilazione C++..."
mkdir -p $BUILD_DIR
cd $BUILD_DIR
# Configurazione CMake robusta: Release (veloce) + Path esplicito libfranka
cmake .. -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=$FRANKA_DIR > /dev/null
make -j4
if [ $? -ne 0 ]; then
    echo "ERRORE CRITICO: Compilazione fallita. Il sistema non verrà avviato."
    exit 1
fi
cd ..

# --- 1. AVVIO VISIONE (PYTHON) ---
echo "[1/3] Avvio modulo Visione (YOLO + RealSense)..."
python3 $PYTHON_SCRIPT &
PID_PYTHON=$!

# Attesa per dare tempo a YOLO di caricarsi e alla RealSense di stabilizzarsi
echo "Attesa 8 secondi per inizializzazione..."
sleep 8

# --- 2. AVVIO CONTROLLO (C++) ---
echo "[2/3] Avvio Controllo Ammettenza..."
# Esegue il programma C++. Lo script bash rimarrà qui finché il programma C++ non finisce.
./$BUILD_DIR/$CPP_EXECUTABLE $ROBOT_IP

# Quando il C++ finisce (o crasha), eseguiamo il cleanup
echo "Programma C++ terminato."
cleanup