import cv2
import mediapipe as mp
import time

# Configurazione Path
# Usa lo stesso video di input del benchmark
VIDEO_PATH = "/home/lab/Videos/donaldo_ws_visione/Video_benchmarking/Dinamico_19122025.avi"
# Path per il video di output con lo scheletro disegnato
OUTPUT_VIDEO = "/home/lab/Videos/donaldo_ws_visione/Video_benchmarking/Dinamico_19122025_skeleton_mediapipe.avi"

def run_visualization():
    """
    Legge il video, applica MediaPipe Pose e salva un nuovo video con lo scheletro disegnato.
    """
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles

    # Configurazione Pose (stessa del benchmark per coerenza visiva)
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=2,  # Modello più accurato
        enable_segmentation=False,
        min_detection_confidence=0.5
    )

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"Errore apertura video: {VIDEO_PATH}")
        return

    # Configurazione Video Writer
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Codec per .avi (XVID è ampiamente supportato su Linux/OpenCV)
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (width, height))

    print(f"Inizio elaborazione video...")
    print(f"Input: {VIDEO_PATH}")
    print(f"Output: {OUTPUT_VIDEO}")

    frame_count = 0
    start_time = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # MediaPipe richiede immagini in RGB, OpenCV usa BGR
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Inferenza
        results = pose.process(image_rgb)

        # Disegno dei landmarks sul frame originale (che è in BGR)
        if results.pose_landmarks:
            # Disegna le connessioni e i punti dello scheletro usando lo stile di default
            mp_drawing.draw_landmarks(
                frame,
                results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style()
            )

        # Scrittura del frame nel video di output
        out.write(frame)
        
        frame_count += 1
        if frame_count % 50 == 0:
            print(f"Frame elaborati: {frame_count}")

    cap.release()
    out.release()
    elapsed = time.time() - start_time
    print(f"Elaborazione completata in {elapsed:.2f} secondi.")
    print(f"Video salvato in: {OUTPUT_VIDEO}")

if __name__ == "__main__":
    run_visualization()