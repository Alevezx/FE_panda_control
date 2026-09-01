# Progetto: Controllo di Ammettenza per Pick and Place v2 con Sicurezza Capsule

Questa è la seconda versione (v2) del progetto di controllo di ammettenza per operazioni di pick and place con il robot Franka Panda. Rispetto alla versione precedente (v1 in `../04_pick_and_place_v1/`), introduce miglioramenti nel calcolo dinamico delle traiettorie di recovery.

## Descrizione Generale

Il sistema esegue sequenze predefinite di movimento tra waypoints in modalità sequenziale continua, integrando sicurezza basata su capsule geometriche e rilevamento pose umane via YOLO/RealSense. La comunicazione avviene tramite ZMQ per garantire fluidità e sicurezza.

- **Waypoints e Sequenze**: Stessi 7 waypoints (A-G) e sequenze configurabili (es. F→E→C→D→C→E ripetuto).
- **Sicurezza Capsule**: Modellazione robot e ostacoli come capsule, calcolo distanze minime e forze repulsive con profilo quadratico per fluidità.
- **Visione e Comunicazione**: Script Python per rilevamento pose YOLOv8, filtering One Euro, trasmissione ZMQ.
- **Logging e Analisi**: Salvataggio stati robot, forze, matrici dinamiche. Generazione video di output per analisi visiva (sicurezza e soft repulsion).

## Differenze dalla Versione v1 (04_pick_and_place_v1)

La v2 introduce le seguenti modifiche principali rispetto alla v1:

- **Recovery Dinamico**: Durata della fase di recovery calcolata dinamicamente basata sulla distanza da recuperare e velocità massima (250 mm/s), invece di essere fissa a 3.0 secondi. Questo ottimizza i tempi di ripresa dopo uno stop, riducendo attese inutili.
- **Ottimizzazioni Codice**: Miglioramenti minori nella logica di calcolo traiettorie e gestione stati, con focus su stabilità e reattività.


## File e Programmi Principali

### File C++
- **controllo_ammettenza_pick_and_place.cpp**: Programma principale per controllo ammettenza in sequenze pick and place. Gestisce waypoints, traiettorie quintiche, forze repulsive capsule, modello dinamico Franka, comunicazione ZMQ, logging. Include modifiche per stop immediato e recovery dinamico.
- **header_capsuleStatiche.h** e **header_capsuleStatiche.cpp**: Header per strutture capsule, parametri sicurezza, funzioni calcolo distanze, generazione traiettorie, salvataggio log CSV.
- **examples_common.h** e **examples_common.cpp**: Utilities Franka per controllo robot.
- **skeleton_zmq.h**: Gestione ZMQ per ricezione dati capsule da Python.

### File Python
- **skeleton_yolo_and_transmission.py**: Script visione principale per YOLOv8, RealSense, filtering, ZMQ.
- **skeleton_yolo_and_transmission_originale.py**: Versione originale backup.

### File di Configurazione e Dati
- **marker_pos.txt**: Posizione marker calibrazione.
- **rotation_matrix.txt**: Matrice trasformazione camera-robot.
- **yolov8x-pose.pt**: Modello YOLOv8.
- **output_video_capsule_sicurezza.avi**: Video test sicurezza (stop immediato).
- **output_video_capsule_soft.avi** e **output_video_capsule_soft_2.avi**: Video test repulsione soft.

### Script e Build
- **CMakeLists.txt**: Configurazione CMake per eseguibile `controllo_ammettenza_pick_and_place`.
- **Istruzioni_compilazione.txt**: Guida compilazione.
- **run_system.sh**: Script automatizzato avvio completo.
- **build/**: Cartella build.

## Come Utilizzare

1. **Compilazione**: Usa `run_system.sh` o segui `Istruzioni_compilazione.txt`.
2. **Esecuzione**: Avvia `run_system.sh` o manualmente `python3 skeleton_yolo_and_transmission.py &` poi `./build/controllo_ammettenza_pick_and_place <IP_ROBOT>`.
3. **Configurazione**: Modifica `pattern` e `num_repetitions` per sequenze. Parametri sicurezza in codice.
4. **Calibrazione**: Aggiorna file calibrazione.

# Configurazione del file bash
1. **Rendi eseguibile lo script bash**
    - chmod +x run_system.sh
2. **Compila il progetto in c++, se non è già stato fatto**
    - mkdir -p build && cd build
    - cmake .. -DCMAKE_BUILD_TYPE=Release -DFranka_DIR=/home/lab/donaldo_ws/home/lab/donaldo_ws/libfranka/build
    - make -j4
    - cd ..
3. **Lancia tutto**
    - ./run_system.sh

## Dipendenze
- **C++**: Franka SDK, Eigen, ZMQ, CMake.
- **Python**: OpenCV, NumPy, PyRealSense2, Ultralytics, ZMQ.

## Note
- **Sequenze**: Configurabili via `pattern` (es. {5,4,2,3,2,4} = F→E→C→D→C→E).
- **Sicurezza**: Stop immediato (r_inner=0), repulsione quadratica, recovery dinamico.
- **Logging**: CSV per success/error, video per debug.
- **Ottimizzazioni**: Focus su reattività e stabilità rispetto a v1.