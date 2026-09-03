import time
import signal
import cv2
import math
import struct
import numpy as np
import zmq
import pyrealsense2 as rs
from ultralytics import YOLO

# ---------- One Euro (come tuo script) ----------
# Implementazione del filtro 1€ (One Euro Filter).
# È un filtro passa-basso adattivo che riduce il jitter (tremolio) quando il movimento è lento
# e riduce la latenza quando il movimento è veloce.
class OneEuroFilter:
    # costruttore del filtro __init__
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = float(min_cutoff) # cutoff minimo per il filtro, usato quando il segnale è quasi fermo
        self.beta = float(beta) # coefficiente di velocità. Determina quanto il cutoff si adatta alla velocità del segnale
        self.d_cutoff = float(d_cutoff) # cutoff per la derivata del segnale (altrimenti il rumore verrebbe interpretato come velocità)
         # stato iniziale (ultimi valori filtrati)
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0) # tempo dell'ultimo aggiornamento (time stamp)

    # Calcola il fattore di smoothing alpha in base al tempo trascorso e al cutoff
    def smoothing_factor(self, t_e, cutoff):
        r = 2.0 * math.pi * cutoff * t_e
        return r / (r + 1.0)

    # Applica l'esponenziale smoothing
    def exponential_smoothing(self, alpha, x, x_prev):
        return alpha * x + (1.0 - alpha) * x_prev

    # Aggiorna il filtro con un nuovo campione (t, x)
    def __call__(self, t, x):
        t_e = t - self.t_prev # t_e: time elapsed, periodo di campionamento istantaneo (dt)
        if t_e <= 0.0:
            return self.x_prev
        a_d = self.smoothing_factor(t_e, self.d_cutoff)
        # Prima di filtrare la posizione, il filtro deve stimare la velocità (è una velocità grezza)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(a_d, dx, self.dx_prev)
        # Adatta il cutoff in base alla velocità stimata
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        # Filtra la posizione con il cutoff adattato (filtraggio finale)
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat

# Classe wrapper per gestire il filtraggio indipendente delle coordinate (x, y, z)
# per ogni keypoint dello scheletro. Mantiene lo stato dei filtri tra i frame.
# Se ho 17 keypoints 3D, creo 17x3 = 51 filtri indipendenti (51 istanze OneEuroFilter indipendenti).
class Keypoints3DSmoother:
    # costruttore
    def __init__(self, num_kpts=17, min_cutoff=0.1, beta=1.0):
        self.num_kpts = num_kpts # numero di keypoints 3D da filtrare
        self.min_cutoff = min_cutoff
        self.beta = beta
        # Tempo di riferimento iniziale (time.monotonic() funzione di tempo che rappresenta il tempo trascorso dall'avvio del programma (in secondi come float) (il tempo non può mai diminuire))
        self.t0 = time.monotonic()
        self.initialized = False
        self.filters = []  # Lista di tuple: ogni elemento è (filter_x, filter_y, filter_z)
        self.last_valid = np.full((num_kpts, 3), np.nan, dtype=np.float32) # Memoria per gestire occlusioni temporanee

    # ciclo di aggiornamento del filtro
    def update(self, xyz, conf, conf_thr):
        t = time.monotonic() - self.t0
        # Inizializza i filtri al primo frame valido
        if not self.initialized:
            for i in range(self.num_kpts):
                x0 = float(xyz[i, 0]) if np.isfinite(xyz[i, 0]) else 0.0
                y0 = float(xyz[i, 1]) if np.isfinite(xyz[i, 1]) else 0.0
                z0 = float(xyz[i, 2]) if np.isfinite(xyz[i, 2]) else 0.0
                self.filters.append((
                    OneEuroFilter(t, x0, min_cutoff=self.min_cutoff, beta=self.beta),
                    OneEuroFilter(t, y0, min_cutoff=self.min_cutoff, beta=self.beta),
                    OneEuroFilter(t, z0, min_cutoff=self.min_cutoff, beta=self.beta),
                ))
            self.initialized = True

        out = np.copy(xyz).astype(np.float32)
        for i in range(self.num_kpts):
            # Se il punto non è valido (confidenza bassa o NaN), usa l'ultimo valore valido noto
            # Questo evita che il robot veda il punto sparire o andare a zero (Zero-Order Hold)
            valid = (conf[i] >= conf_thr) and np.all(np.isfinite(xyz[i]))
            if not valid:
                if np.all(np.isfinite(self.last_valid[i])):
                    # Mantiene l'ultima posizione valida (il punto "congela" dove era)
                    out[i] = self.last_valid[i]
                else:
                    out[i] = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
                continue
            # Applica il filtro su ogni asse
            fx, fy, fz = self.filters[i] # nota che fx,fy,fz sono istanze (oggetti) di OneEuroFilter!
            out[i, 0] = fx(t, float(xyz[i, 0]))
            out[i, 1] = fy(t, float(xyz[i, 1]))
            out[i, 2] = fz(t, float(xyz[i, 2]))
            self.last_valid[i] = out[i]
        return out

# ---------- Config (coerente coi tuoi script) ----------
# Definizione dei keypoint di interesse (es. escludendo piedi se non servono)
TARGET_KEYPOINTS = list(range(13))  # 0..12 pelvis-up (fino alle anche)
# Definizione delle connessioni (ossa) basata sullo standard COCO
COCO_SKELETON = [
    (0, 1), (0, 2), (1, 3), (2, 4), (3, 5), (4, 6),
    (5, 7), (7, 9), (6, 8), (8, 10),
    (5, 6), (5, 11), (6, 12), (11, 12),
    (11, 13), (13, 15), (12, 14), (14, 16)
]
# Filtra solo le connessioni che coinvolgono i keypoint target
EDGES = [(a, b) for (a, b) in COCO_SKELETON if a in TARGET_KEYPOINTS and b in TARGET_KEYPOINTS]

# Estrae la profondità (Z) in modo robusto calcolando la mediana dei valori di profondità
# in una finestra RxR attorno al pixel (u, v).
# Questo aiuta a ignorare i pixel con profondità mancante (0) o rumorosa.
def robust_depth_median(depth_frame, u, v, R=4):
    w, h = depth_frame.get_width(), depth_frame.get_height()
    uu, vv = int(round(u)), int(round(v)) # pixel centrali, round() arrotonda al più vicino intero
    zs = []
    for dy in range(-R, R + 1):
        y = vv + dy
        if y < 0 or y >= h:
            continue
        for dx in range(-R, R + 1):
            x = uu + dx
            if x < 0 or x >= w:
                continue
            z = depth_frame.get_distance(x, y)  # metri (classe.metodo() di pyrealsense2)
            if z > 0.0 and math.isfinite(z):
                zs.append(z)
    if not zs:
        return float("nan")
    zs.sort()
    return zs[len(zs) // 2]

# Carica la matrice di trasformazione omogenea (4x4) dal file TXT.
def load_T_base_cam(path_txt):
    T = np.loadtxt(path_txt, dtype=np.float64)
    assert T.shape == (4, 4)
    return T

# Applica una trasformazione rigida (rotazione + traslazione) ai punti 3D.
# Usata per passare dal sistema di riferimento della telecamera a quello della base del robot.
def transform_points(T, pts_xyz):
    # pts_h: punti omogenei (N,4), aggiungendo una colonna di 1 in coda
    pts_h = np.concatenate([pts_xyz, np.ones((pts_xyz.shape[0], 1))], axis=1)
    # T: trasformazione omogenea (4,4), pts_h.T: (4,N) (la trasposta), @ è il prodotto matriciale riga per colonna
    return (T @ pts_h.T).T[:, :3] # ritorna solo le prime 3 colonne (X,Y,Z) di tutte le righe

# ---------- ZMQ message: header + records ----------
# Definizione del protocollo binario per la trasmissione dati
# MAGIC = b"SKEL" # Identificatore del messaggio (4 byte) (il pacchetto inizia con questi 4 byte)
# HDR_FMT = "<4sHHQ": Definisce la struttura dell'Intestazione (Header) del pacchetto.
# <: Indica Little-Endian (ordine byte)
# 4s: 4 byte stringa (MAGIC) (per SKEL)
# H: unsigned short (2 byte) (VERSION)
# H: unsigned short (2 byte) (n_caps, numero di capsule nel messaggio)
# Q: unsigned long long (8 byte) (t_mono_ns, timestamp in nanosecondi)
# REC_FMT = "<8f": Definisce la struttura di ogni record di capsula.
# <: Little-Endian
# 8f: 8 float (4 byte ciascuno) (x1, y1, z1 per l'inizio, x2, y2, z2 per la fine, radius, conf)

MAGIC = b"SKEL" 
VERSION = 1
HDR_FMT = "<4sHHQ"     # magic, version, n_caps, t_mono_ns )
REC_FMT = "<8f"        # x1 y1 z1 x2 y2 z2 radius conf
MAX_CAPS = 32

def main():
    # Parametri rapidi
    conf_thr = 0.5          # Soglia di confidenza minima per considerare valido un keypoint
    human_radius = 0.20     # Raggio della capsula (cilindro) attorno all'osso (metri)
    endpoint = "ipc:///tmp/skeleton.ipc" # Indirizzo socket ZeroMQ (IPC per comunicazione locale veloce)
    # ipc sta per inter-process communication (comunicazione tra processi) (usa un file socket speciale nel filesystem si linux, invece che la rete TCP/IP)

    T_base_cam = load_T_base_cam("rotation_matrix.txt") # Carica calibrazione camera-robot
    # --- DEBUG: Usa una matrice identità per bypassare la calibrazione errata ---    
    # T_base_cam = np.identity(4)

    # Caricamento modello YOLOv8 per Pose Estimation
    model = YOLO("yolov8x-pose.pt")

    # Inizializzazione ZeroMQ (Publisher)
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB) # socket di tipo Publisher (trasmette dati a chiunque sia connesso, se nessuno è connesso i dati vengono persi)
    pub.setsockopt(zmq.LINGER, 0) # Evita che ZMQ blocchi la chiusura se ci sono messaggi pendenti
    pub.bind(endpoint) # Associa il socket all'endpoint specificato (questo script python crea e possiede il socket, gli altri processi si connettono a questo endpoint, come il cpp del controllo ammettenza)


    # Configurazione Pipeline RealSense
    pipe = rs.pipeline() # Pipeline per la gestione del flusso dati della camera (gestore principale)
    # rs.config() istanzia una classe della libreria pyrealsense2 che serve a contenere le tue preferenze.
    cfg = rs.config() 
    # preferenze
    cfg.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)
    pipe.start(cfg)

    # Oggetto per allineare la profondità al frame colore
    align = rs.align(rs.stream.color)

    # Inizializzazione filtri di smoothing
    smoother = Keypoints3DSmoother(num_kpts=17, min_cutoff=0.1, beta=1.0)

    # Gestione segnali per chiusura pulita (es. CTRL+C o kill da script bash)
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    while running:
        # Acquisizione frame (fs)
        fs = pipe.wait_for_frames()
        fs = align.process(fs) # Allineamento fondamentale per far corrispondere pixel RGB a Depth
        depth = fs.get_depth_frame()
        color = fs.get_color_frame()
        if not depth or not color:
            continue

        # Conversione immagine per YOLO
        color_img = np.asanyarray(color.get_data()) # trasforma i dati grezzi della telecamera in un array NumPy
        # Inferenza rete neurale
        results = model.predict(color_img, verbose=False)

        caps = []
        # Se è stata rilevata almeno una persona
        if results and results[0].keypoints is not None and len(results[0].keypoints.data) > 0:
            person = results[0].keypoints.data[0].cpu().numpy()  # (17,3) -> x, y, conf
            xy = person[:, :2]
            conf = person[:, 2]

            # Intrinseci della camera per la deproiezione
            intr = depth.profile.as_video_stream_profile().intrinsics

            # 1. Estrazione coordinate 3D nel frame Camera
            xyz_cam = np.full((17, 3), np.nan, dtype=np.float32)
            for k in TARGET_KEYPOINTS:
                if conf[k] < conf_thr:
                    continue
                u, v = float(xy[k, 0]), float(xy[k, 1])
                # Lettura robusta della profondità
                z = robust_depth_median(depth, u, v, R=4)
                if not math.isfinite(z):
                    continue
                # Deproiezione: da pixel 2D + depth, a punto 3D (metri)
                X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [u, v], z)
                xyz_cam[k] = np.array([X, Y, Z], dtype=np.float32)

            # 2. Filtraggio temporale (OneEuroFilter)
            xyz_cam_s = smoother.update(xyz_cam, conf, conf_thr)

            # --- ADATTAMENTO FRAME (Optical -> Geometric) --- 
            # (solo se uso la calibrazione vecchia con adattamenti degli assi)
            # [MODIFICA NUOVA CALIBRAZIONE]
            # Con la nuova calibrazione che considera il frame ottico (aruco_ros standard),
            # NON DOBBIAMO PIÙ SCAMBIARE GLI ASSI MANUALMENTE.
            # La matrice caricata trasforma direttamente da Optical Frame (x, y, z) a Base Frame.
            
            # (Codice originale commentato come richiesto)
            # xyz_cam_mapped = np.zeros_like(xyz_cam_s)
            # xyz_cam_mapped[:, 0] = xyz_cam_s[:, 0]  # x -> x
            # xyz_cam_mapped[:, 1] = xyz_cam_s[:, 2]  # y -> z (Forward)
            # xyz_cam_mapped[:, 2] = -xyz_cam_s[:, 1] # z -> -y (Up)

            # Trasformazione nel frame Base del Robot 
            # Usa xyz_cam_mapped invece di xyz_cam_s
            # xyz_base = transform_points(T_base_cam, xyz_cam_mapped.astype(np.float64)).astype(np.float32)
            # --- FINE ADATTAMENTO FRAME ---
            
            # 3. Trasformazione nel frame Base del Robot
            # Usa xyz_cam_s direttamente (frame ottico nativo RealSense) invece di xyz_cam_mapped
            xyz_base = transform_points(T_base_cam, xyz_cam_s.astype(np.float64)).astype(np.float32)

            # 4. Creazione delle capsule (segmenti)
            # (Volendo, si potrebbe considerare una logica per la quale se in questo momento non  vedo nulla, mando comunque l'ultima cosa valida, per evitare come faccio adesso di non mandare nulla.)
            for (a, b) in EDGES:
                if conf[a] < conf_thr or conf[b] < conf_thr:
                    continue
                pa = xyz_base[a]
                pb = xyz_base[b]
                # Verifica validità coordinate
                if not (np.all(np.isfinite(pa)) and np.all(np.isfinite(pb))):
                    continue
                if len(caps) >= MAX_CAPS:
                    break
                # Aggiunge capsula: p1, p2, raggio, confidenza minima
                caps.append((pa[0], pa[1], pa[2], pb[0], pb[1], pb[2], float(human_radius), float(min(conf[a], conf[b]))))
            
            # --- VISUALIZZAZIONE REAL-TIME ---
            # Disegna lo scheletro direttamente sull'immagine RGB per il debug a video
            for (u, v) in EDGES:
                if conf[u] >= conf_thr and conf[v] >= conf_thr:
                    pt1 = (int(xy[u, 0]), int(xy[u, 1]))
                    pt2 = (int(xy[v, 0]), int(xy[v, 1]))
                    cv2.line(color_img, pt1, pt2, (0, 255, 0), 2)
            for k in TARGET_KEYPOINTS:
                if conf[k] >= conf_thr:
                    cv2.circle(color_img, (int(xy[k, 0]), int(xy[k, 1])), 4, (0, 0, 255), -1)

        # 5. Serializzazione e invio dati (impacchettamento e invio nel loop)
        t_mono_ns = time.monotonic_ns()
        # Header: Magic, Versione, Numero Capsule, Timestamp
        # struct.pack(): converte i dati in una stringa di byte secondo il formato specificato
        header = struct.pack(HDR_FMT, MAGIC, VERSION, len(caps), t_mono_ns)
        # Payload: Lista di capsule (*rec serve a spacchettare la tupla della capsula in singoli argomenti - grazie all'asterisco -)
        payload = b"".join(struct.pack(REC_FMT, *rec) for rec in caps)
        # Invio messaggio completo (header + payload) (singolo messaggio atomico)
        pub.send(header + payload)
        
        # Mostra l'immagine a schermo (premere 'q' per uscire, anche se lo script bash lo chiuderà forzatamente)
        cv2.imshow("YOLO Skeleton Realtime", color_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Cleanup risorse (fondamentale per non bloccare la RealSense al riavvio)
    print("Chiusura pipeline e finestre...")
    pipe.stop()
    cv2.destroyAllWindows()
    pub.close()
    ctx.term()

if __name__ == "__main__":
    main()
