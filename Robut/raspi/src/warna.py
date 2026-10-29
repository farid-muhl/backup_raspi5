from picamera2 import Picamera2
import cv2
import time

def tes_warna_absolut():
    print("[TEST] Memulai kamera untuk tes warna final...")
    picam2 = Picamera2()
    
    # Kita paksakan ke format standar RGB888 bawaan Raspberry Pi
    config = picam2.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
    picam2.configure(config)
    picam2.start()
    
    print("[TEST] Menunggu sensor stabil (2 detik)...")
    time.sleep(2)

    # Ambil 1 frame murni
    frame_mentah = picam2.capture_array("main")
    picam2.stop()

    print("[TEST] Gambar berhasil diambil. Memproses 3 versi warna...")

    # Versi 1: Mentah apa adanya (Langsung disimpan)
    cv2.imwrite("warna_1_mentah.jpg", frame_mentah)

    # Versi 2: Konversi resmi OpenCV (Tukar Red dan Blue)
    frame_cv2 = cv2.cvtColor(frame_mentah, cv2.COLOR_RGB2BGR)
    cv2.imwrite("warna_2_cv2.jpg", frame_cv2)

    # Versi 3: Konversi brutal menggunakan Numpy (Pemotongan Array)
    frame_numpy = frame_mentah[:, :, ::-1]
    cv2.imwrite("warna_3_numpy.jpg", frame_numpy)

    print("[TEST] Selesai!")
    print("Silakan buka folder Anda dan lihat ketiga fail ini:")
    print("1. warna_1_mentah.jpg")
    print("2. warna_2_cv2.jpg")
    print("3. warna_3_numpy.jpg")

if __name__ == "__main__":
    tes_warna_absolut()
