import cv2
import numpy as np
import torch
import time
from pathlib import Path

import mmpose
from mmpose.apis import init_model, inference_topdown
from mmpose.registry import VISUALIZERS
from mmdet.apis import init_detector, inference_detector
from mmengine.registry import init_default_scope

# -------------------------
# Configurazione
# -------------------------
VIDEO_PATH = "/home/lab/Videos/donaldo_ws_visione/Video_benchmarking/Dinamico_19122025.avi"
OUTPUT_VIDEO = "/home/lab/Videos/donaldo_ws_visione/Video_benchmarking/Dinamico_19122025_skeleton_rtmpose.avi"

# Thresholds
DET_SCORE_THR = 0.3          # soglia bbox persona
POSE_SCORE_THR = 0.3         # soglia visualizzazione keypoints
PERSON_LABEL_ID = 0          # COCO: class 0 = person

# Device
DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"

# Root locale di mmpose
MMPOSE_ROOT = Path(mmpose.__file__).resolve().parents[1]

# Detector (MMDetection) config + checkpoint
DET_CONFIG = MMPOSE_ROOT / "demo/mmdetection_cfg/rtmdet_m_640-8xb32_coco-person.py"
DET_CHECKPOINT = (
    "https://download.openmmlab.com/mmpose/v1/projects/rtmpose/"
    "rtmdet_m_8xb32-100e_coco-obj365-person-235e8209.pth"
)

# Pose (MMPose RTMPose) config + checkpoint
POSE_CONFIG = MMPOSE_ROOT / "projects/rtmpose/rtmpose/body_2d_keypoint/rtmpose-m_8xb256-420e_coco-256x192.py"
POSE_CHECKPOINT = (
    "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/"
    "rtmpose-m_simcc-aic-coco_pt-aic-coco_420e-256x192-63eb25f7_20230126.pth"
)

def _to_numpy(x):
    """Converte Tensor/array/sequence in np.ndarray."""
    if x is None:
        return None
    if isinstance(x, np.ndarray):
        return x
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy()
    return np.asarray(x)

def get_best_person_bbox(det_result, score_thr=0.3, person_label=0):
    """
    Estrae la bbox (xyxy) della persona con score più alto.
    """
    # MMDet 3.x: det_result.pred_instances.{bboxes, scores, labels}
    if hasattr(det_result, "pred_instances"):
        inst = det_result.pred_instances
        bboxes = _to_numpy(getattr(inst, "bboxes", None))   # (N,4)
        scores = _to_numpy(getattr(inst, "scores", None))   # (N,)
        labels = _to_numpy(getattr(inst, "labels", None))   # (N,)

        if bboxes is None or scores is None or labels is None or len(bboxes) == 0:
            return None

        mask = (labels == person_label) & (scores >= score_thr)
        if not np.any(mask):
            return None

        idxs = np.where(mask)[0]
        best_i = idxs[np.argmax(scores[idxs])]
        return bboxes[best_i].astype(np.float32)
    return None

def run_visualization():
    """
    Legge il video, applica RTMPose (Top-Down) e salva un nuovo video con lo scheletro disegnato.
    """
    print(f"Device: {DEVICE}")
    print(f"DET_CONFIG: {DET_CONFIG}")
    print(f"POSE_CONFIG: {POSE_CONFIG}")

    # 1) Init detector sotto scope mmdet
    init_default_scope("mmdet")
    det_model = init_detector(str(DET_CONFIG), DET_CHECKPOINT, device=DEVICE)

    # 2) Init pose sotto scope mmpose
    init_default_scope("mmpose")
    pose_model = init_model(str(POSE_CONFIG), POSE_CHECKPOINT, device=DEVICE)

    # 3) Init Visualizer
    # Il visualizer è definito nella config del modello pose
    pose_model.cfg.visualizer.radius = 3
    pose_model.cfg.visualizer.line_width = 1
    visualizer = VISUALIZERS.build(pose_model.cfg.visualizer)
    # Imposta i metadati del dataset (colori scheletro, link, ecc.)
    visualizer.set_dataset_meta(pose_model.dataset_meta)

    cap = cv2.VideoCapture(VIDEO_PATH)
    if not cap.isOpened():
        print(f"Errore apertura video: {VIDEO_PATH}")
        return

    # Configurazione Video Writer
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (width, height))

    print(f"Inizio elaborazione video...")
    print(f"Input: {VIDEO_PATH}")
    print(f"Output: {OUTPUT_VIDEO}")

    frame_count = 0
    start_time = time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # --- Detection ---
        # Detector: assicurati scope mmdet prima di inference_detector
        init_default_scope("mmdet")
        det_result = inference_detector(det_model, frame)
        person_bbox = get_best_person_bbox(
            det_result, score_thr=DET_SCORE_THR, person_label=PERSON_LABEL_ID
        )

        pose_results = []
        if person_bbox is not None:
            bboxes = np.array([person_bbox], dtype=np.float32)  # (1,4) xyxy

            # --- Pose Estimation ---
            # Pose: torna a scope mmpose prima di inference_topdown
            init_default_scope("mmpose")
            pose_results = inference_topdown(
                pose_model, frame, bboxes=bboxes, bbox_format="xyxy"
            )

        # --- Visualization ---
        if pose_results:
            # Disegna i risultati sul frame
            visualizer.add_datasample(
                name='video',
                image=frame,
                data_sample=pose_results[0], # Disegna solo la persona migliore
                draw_gt=False,
                draw_bbox=True,
                draw_heatmap=False,
                show_kpt_idx=False,
                skeleton_style='mmpose',
                show=False,
                kpt_thr=POSE_SCORE_THR
            )
            annotated_frame = visualizer.get_image()
        else:
            annotated_frame = frame

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