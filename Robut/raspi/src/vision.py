import os
import cv2
import time
from ultralytics import YOLO
from picamera2 import Picamera2
import config

class VisionSystem:
    def __init__(self, model_path="my_model.pt"):
        self.model_path = model_path

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(
                f"Model tidak ditemukan: {self.model_path}. "
                "Pastikan file my_model.pt ada di folder yang sama atau gunakan path absolut."
            )
            
        print("[VISION] Memuat model YOLO...")
        self.model = YOLO(self.model_path)
        self._cached_frame = None
        self._cached_frame_id = None
        self._cached_detections = []
        
        print("[VISION] Menginisialisasi Picamera2...")
        self.picam2 = Picamera2()

        # KUNCI UTAMA: Kita pancing menggunakan "RGB888" agar hardware 
        # kamera Anda memuntahkan output "BGR" mentah yang dibutuhkan YOLO.
        camera_config = self.picam2.create_video_configuration(
            main={
                "size": (640, 480),
                "format": "RGB888"
            }
        )

        self.picam2.configure(camera_config)
        self.picam2.start()

        print("[VISION] Menunggu sensor stabil (2 detik)...")
        time.sleep(2.0)
        print("[VISION] Sistem Visi siap!")

    def capture_frame(self):
        try:
            # AMBIL DATA MENTAH. JANGAN PERNAH menambahkan cv2.cvtColor atau numpy[::-1] di sini!
            frame = self.picam2.capture_array("main")
            return frame
        except Exception as e:
            print(f"[VISION ERROR] Gagal menangkap gambar: {e}")
            return None

    def detect_environment(self, frame):
        if frame is None:
            return {"frame_size": (0, 0), "left_limit": 0, "right_limit": 0, "zones": {}}
            
        height, width = frame.shape[:2]
        detections = self._predict(frame)
        zones = {}
        
        for detection in detections:
            label = detection["class_name"]
            if "watering" in label or "pot" in label or "zone" in label:
                x1, y1, x2, y2 = detection["bbox"]
                zones[label] = {
                    "class_name": label,
                    "center": ((x1 + x2) // 2, (y1 + y2) // 2),
                    "bbox": detection["bbox"],
                    "area": max(0, x2 - x1) * max(0, y2 - y1),
                    "confidence": detection["confidence"],
                }
                
        return {
            "frame_size": (width, height),
            "left_limit": width // 3,
            "right_limit": (2 * width) // 3,
            "zones": zones,
        }

    def _predict(self, frame):
        if self._cached_frame is frame:
            return self._cached_detections
            
        results = self.model.predict(
            source=frame,
            conf=config.YOLO_CONFIDENCE_THRESHOLD,
            verbose=False,
        )
        
        detections = []
        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                class_name = str(self.model.names[class_id]).strip().lower()
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                x1 = max(0, min(x1, frame.shape[1] - 1))
                x2 = max(x1 + 1, min(x2, frame.shape[1]))
                y1 = max(0, min(y1, frame.shape[0] - 1))
                y2 = max(y1 + 1, min(y2, frame.shape[0]))
                detections.append({
                    "class_name": class_name,
                    "bbox": (x1, y1, x2, y2),
                    "confidence": float(box.conf[0]),
                })
                
        self._cached_frame = frame
        self._cached_frame_id = id(frame)
        self._cached_detections = detections
        return detections

    def detect_bottles(self, frame):
        if frame is None:
            return []
            
        width = frame.shape[1]
        left_limit = width // 3
        right_limit = 2 * width // 3
        bottles = []
        
        for detection in self._predict(frame):
            label = detection["class_name"]
            
            if "botol" not in label and "bottle" not in label:
                continue
                
            x1, y1, x2, y2 = detection["bbox"]
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            
            if center_x < left_limit:
                position = "LEFT"
            elif center_x < right_limit:
                position = "CENTER"
            else:
                position = "RIGHT"
                
            bottles.append({
                "color": label,
                "class_name": label,
                "center": (center_x, center_y),
                "bbox": detection["bbox"],
                "confidence": detection["confidence"],
                "zone": position,
            })
            
        return bottles

    def draw_debug(self, frame, bottles, env):
        if frame is None:
            return
            
        h, w = frame.shape[:2]
        left_limit = w // 3
        right_limit = 2 * w // 3
        
        cv2.line(frame, (left_limit, 0), (left_limit, h), (255, 255, 255), 2)
        cv2.line(frame, (right_limit, 0), (right_limit, h), (255, 255, 255), 2)
        cv2.putText(frame, "LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, "CENTER", (left_limit + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, "RIGHT", (right_limit + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        for bottle in bottles:
            x1, y1, x2, y2 = bottle["bbox"]
            cx, cy = bottle["center"]
            zone = bottle["zone"]
            conf = bottle["confidence"]
            color = bottle["color"]
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{color} {conf:.2f} | {zone}", (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            
        for zone in env.get("zones", {}).values():
            x1, y1, x2, y2 = zone["bbox"]
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(frame, f"ZONE {zone.get('class_name', '')}", (x1, max(y1 - 10, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

    def close(self):
        try:
            self.picam2.stop()
        except Exception as e:
            print(f"[VISION ERROR] Gagal mematikan kamera: {e}")
        finally:
            cv2.destroyAllWindows()
