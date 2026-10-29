from vision import VisionSystem
import cv2

def lacak_kamera():
    print("\n[SYSTEM] Memulai Modul AI Kamera...")
    try:
        vision = VisionSystem(model_path="my_model.pt") 
        
        print("[SYSTEM] Mengambil jepretan kamera...")
        frame = vision.capture_frame()
        
        if frame is None:
            print("\n[ERROR FATAL] Kamera gagal menangkap gambar!")
            return
            
        # --- TAMBAHKAN DUA BARIS INI ---
        cv2.imwrite("debug_kamera.jpg", frame)
        print("[SYSTEM] Gambar tersimpan! Buka fail 'debug_kamera.jpg' di folder Anda untuk melihat pandangan robot.")
        # -------------------------------
        
        print("\n[AI] Membaca semua objek mentah tanpa filter:")
        raw_detections = vision._predict(frame)
        if len(raw_detections) == 0:
            print(" -> YOLO tidak melihat objek apapun.")
        else:
            for d in raw_detections:
                print(f" -> Terdeteksi: '{d['class_name']}' (Akurasi: {d['confidence']:.2f})")
                
    except Exception as e:
        print(f"\n[ERROR SISTEM] {e}")
    finally:
        if 'vision' in locals():
            vision.close()

if __name__ == "__main__":
    lacak_kamera()
