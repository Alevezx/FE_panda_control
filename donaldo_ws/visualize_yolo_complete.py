from ultralytics import YOLO
import cv2
import time
import torch

# Configurazione Path
# Usa lo stesso video di input del benchmark
VIDEO_PATH = "/home/lab/Videos/donaldo_ws_visione/Video_benchmarking/Dinamico_19122025.avi"
# Path per il video di output con lo scheletro disegnato
OUTPUT_VIDEO = "/home/lab/Videos/donaldo_ws_visione/Video_benchmarking/Dinamico_19122025_skeleton_yolo.avi"
MODEL_NAME = "yolov8x-pose.pt"  # Usa lo stesso modello del benchmark

def run_visualization():
    """
    Legge il video, applica YOLOv8 Pose e salva un nuovo video con lo scheletro disegnato.
    """
    # Verifica disponibilità GPU
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"YOLO sta usando: {device}")

    # Caricamento del modello
    model = YOLO(MODEL_NAME)

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"Errore apertura video: {VIDEO_PATH}")
        return

    # Configurazione Video Writer
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Codec per .avi (XVID è ampiamente supportato)
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

        # Esecuzione dell'inferenza
        # verbose=False per ridurre l'output in console
        results = model.predict(frame, device=device, verbose=False)

        # Visualizzazione dei risultati sul frame
        # Il metodo plot() restituisce un array numpy (BGR) con le annotazioni disegnate
        annotated_frame = results[0].plot()

        # Scrittura del frame nel video di output
        out.write(annotated_frame)
        
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