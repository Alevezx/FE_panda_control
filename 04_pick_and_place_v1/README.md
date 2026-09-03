# Progetto: Controllo di Ammettenza per Pick and Place con Sicurezza Capsule

Questo progetto implementa un sistema di controllo di ammettenza per operazioni di pick and place con il robot Franka Emika Panda. Il robot esegue sequenze predefinite di movimento tra waypoints (punti di riferimento 3D), integrando sicurezza basata su capsule geometriche statiche e rilevamento pose umane in tempo reale via YOLO e RealSense. La comunicazione avviene tramite ZMQ.

## Descrizione Generale

Il sistema è progettato per eseguire task di pick and place in un ambiente condiviso con umani, utilizzando controllo di ammettenza per adattabilità alle forze esterne. Include:

- **Waypoints e Sequenze**: 7 waypoints definiti (A: home/start, B-F: posizioni operative, G: punto intermedio per evitare singolarità). Sequenze configurabili (es. F→E→C→D→C→E ripetuto) per simulare operazioni di presa e rilascio.
- **Modalità di Esecuzione**: Il programma esegue sequenze di movimento tra waypoints in modalità sequenziale continua (senza pause operative tra segmenti), con gestione automatica di stati di sicurezza (stop, pause, recovery) in caso di avvicinamento a ostacoli umani rilevati dalla visione.
- **Sicurezza Capsule**: Modellazione robot e ostacoli umani come capsule cilindriche. Calcolo distanze minime e forze repulsive per evitare collisioni, con stop immediato se necessario.
- **Visione e Comunicazione**: Script Python per rilevamento pose umane con YOLOv8 e RealSense, filtering One Euro, trasmissione ZMQ al C++.
- **Logging e Analisi**: Salvataggio dettagliato di stati robot, forze, matrici dinamiche per analisi post-esecuzione. Generazione video di output per visualizzazione.


## File e Programmi Principali

### File C++
- **controllo_ammettenza_pick_and_place.cpp**: Programma principale che implementa il controllo di ammettenza per sequenze pick and place. Gestisce waypoints, pianificazione traiettorie quintiche tra punti, calcolo forze repulsive capsule, integrazione modello dinamico Franka, comunicazione ZMQ, e logging.
- **header_capsuleStatiche.h** e **header_capsuleStatiche.cpp**: Header per strutture capsule (CapsuleGeo), parametri sicurezza (SafetyParams), funzioni calcolo distanze segmenti, generazione coefficienti traiettoria, salvataggio log CSV.
- **examples_common.h** e **examples_common.cpp**: Utilities Franka per controllo robot (setDefaultBehavior, MotionGenerator).
- **skeleton_zmq.h**: Gestione comunicazione ZMQ con protocolli binari, buffering double-buffered per ricezione asincrona dati capsule da Python.

### File Python
- **skeleton_yolo_and_transmission.py**: Script principale per visione. Acquisizione RealSense RGB-D, rilevamento pose YOLOv8 (modello yolov8x-pose.pt), filtri One Euro per smoothing, conversione pose in capsule, trasmissione ZMQ.
- **skeleton_yolo_and_transmission_originale.py**: Versione originale dello script (backup).

### File di Configurazione e Dati
- **marker_pos.txt**: Posizione 3D marker di calibrazione (x, y, z) per allineamento camera-robot.
- **rotation_matrix.txt**: Matrice 4x4 trasformazione per calibrazione coordinate camera→robot.
- **yolov8x-pose.pt**: Modello YOLOv8 pre-addestrato per rilevamento pose umane.
- **output_video_v1.avi**: Video di output generato durante esecuzione, utile per analisi visiva delle sequenze. La generazione o meno del video si può gestire tramite un semplice flag nello script python
    (save_video = True      # Imposta a True per salvare il video, False altrimenti
        video_filename = "output_video_v1.avi"
    )

### Script e Build
- **CMakeLists.txt**: Configurazione CMake per compilazione eseguibile `controllo_ammettenza_pick_and_place` con link Franka, Eigen, ZMQ.
- **Istruzioni_compilazione.txt**: Guida manuale compilazione (mkdir build, cmake, make).
- **run_system.sh**: Script automatizzato per avvio completo: compila C++, avvia Python visione, lancia controllo C++ con gestione cleanup.
- **build/**: Cartella build con eseguibili compilati.

## Come Utilizzare

1. **Compilazione**: Usa `run_system.sh` o segui `Istruzioni_compilazione.txt`.
2. **Esecuzione**: 
   - Avvia `run_system.sh` per setup completo.
   - O manualmente: 
        `python3 skeleton_yolo_and_transmission.py &` 
        poi 
        `./build/controllo_ammettenza_pick_and_place <IP_ROBOT>`
3. **Modalità**: Modifica `pattern` e `num_repetitions` in `controllo_ammettenza_pick_and_place.cpp` per sequenze. Usa log per distinguere stopAndGo vs seq.
4. **Calibrazione**: Aggiorna `marker_pos.txt` e `rotation_matrix.txt` per setup specifico.


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
- **Python**: OpenCV, NumPy, PyRealSense2, Ultralytics (YOLO), ZMQ.

## Note
- **Sequenze**: Configurabili via vettore `pattern` (indici waypoints). Esempio: {5,4,2,3,2,4} = F→E→C→D→C→E.
- **Sicurezza**: Stop immediato se distanza < r_inner, repulsione graduale per comfort.
- **Logging**: File CSV separati per success/error e modalità (seq/stopAndGo). Video output per debug visivo.
- **Ottimizzazioni**: Filtering forze repulsive, softmin per distanze multiple, modello dinamico completo per accuratezza.
