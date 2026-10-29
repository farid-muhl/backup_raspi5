import time
import cv2
import config
from serial_comm import ESPController
from vision import VisionSystem
# import lidar_nav (pastikan Anda meng-import modul Lidar Anda di sini)

class AutonomousRobot:
    def __init__(self):
        print("[SYSTEM] Menginisialisasi Robot...")
        
        # 1. Inisialisasi Sensor & Aktuator
        self.esp = ESPController(port=config.SERIAL_PORT, baudrate=config.SERIAL_BAUD, timeout=config.SERIAL_TIMEOUT)
        self.vision = VisionSystem(model_path="my_model.pt")
        # self.lidar = LidarNav() # (Aktifkan baris ini sesuai nama class Lidar Anda)
        
        # 2. STATUS MISI (Finite State Machine)
        self.state = "CARI_BOTOL"  # Status awal saat di Homebase
        
        # 3. Konstanta Jarak & Kecepatan
        self.BATAS_HALANGAN = 40.0  # cm (Kapan harus menghindar)
        self.JARAK_AMBIL = 11.0     # cm (Kapan botol/pot sudah pas di cengkeraman)
        self.KECEPATAN_CARI = 120
        self.KECEPATAN_DEKAT = 50

        # Posisikan capit bersiap (Terbuka dan Agak turun)
        self.set_capit(lift=45, grip=30)
        time.sleep(1)

    def set_capit(self, lift, grip):
        """Fungsi pembantu untuk mengirim sudut capit ke ESP32"""
        self.esp.current_lift = lift
        self.esp.current_grip = grip
        self.esp.stop() # Memaksa ESP32 memperbarui nilai servo tanpa menggerakkan roda

    def manuver_menghindar(self):
        """Logika menghindar rintangan generik"""
        print("[OBSTACLE] Halangan! Menghindar ke kiri...")
        self.esp.turn_left(self.KECEPATAN_CARI)
        time.sleep(0.8) # Sesuaikan waktu putar agar robot berputar sekitar 45-90 derajat
        self.esp.stop()
        time.sleep(0.2)

    def arahkan_ke_target(self, zona_target, kecepatan):
        """Mengarahkan robot agar target berada di tengah kamera"""
        if zona_target == "LEFT":
            self.esp.turn_right(kecepatan - 10)
        elif zona_target == "RIGHT":
            self.esp.turn_left(kecepatan - 10)
        else: # CENTER
            self.esp.forward(kecepatan)

    def run(self):
        print("[MISSION] Memulai Misi 3x3 Meter!")
        
        try:
            while True:
                # ==========================================
                # 1. BACA SEMUA MATA DAN TELINGA ROBOT
                # ==========================================
                frame = self.vision.capture_frame()
                bottles = self.vision.detect_bottles(frame)
                env = self.vision.detect_environment(frame)
                
                sensor_dist = self.esp.read_sensor_data()
                jarak_lidar = 999.0
                if hasattr(self, 'lidar') and self.lidar is not None:
                    try:
                        jarak_lidar = self.lidar.get_sectors().get('front', 999.0)
                    except: pass
                
                # Jarak paling kritis
                jarak_depan = min(sensor_dist, jarak_lidar)
                
                # ==========================================
                # 2. OTAK MESIN STATUS (STATE MACHINE)
                # ==========================================
                
                # ---- TAHAP 1: MENCARI BOTOL ----
                if self.state == "CARI_BOTOL":
                    if len(bottles) > 0:
                        print("[AI] Botol terlihat! Mulai mendekat...")
                        self.esp.stop()
                        self.state = "DEKATI_BOTOL"
                    elif jarak_depan < self.BATAS_HALANGAN:
                        self.manuver_menghindar()
                    else:
                        print("[NAV] Eksplorasi mencari botol...")
                        self.esp.forward(self.KECEPATAN_CARI)

                # ---- TAHAP 2: MENDEKATI BOTOL ----
                elif self.state == "DEKATI_BOTOL":
                    if len(bottles) == 0:
                        print("[AI] Kehilangan jejak botol!")
                        self.state = "CARI_BOTOL"
                    else:
                        target = bottles[0] # Fokus ke botol pertama yang dilihat
                        
                        if jarak_depan < self.JARAK_AMBIL:
                            print("[AI] Botol dalam jangkauan capit!")
                            self.esp.stop()
                            self.state = "AMBIL_BOTOL"
                        else:
                            print(f"[NAV] Mengunci botol di area: {target['zone']}")
                            self.arahkan_ke_target(target['zone'], self.KECEPATAN_DEKAT)

                # ---- TAHAP 3: PROSES CAPIT BOTOL ----
                elif self.state == "AMBIL_BOTOL":
                    print("[AKSI] Mencapit botol...")
                    # 1. Pastikan capit turun dan terbuka (sudah diset di awal)
                    # 2. Maju sedikit sekali untuk memastikan botol masuk mulut capit
                    self.esp.forward(40)
                    time.sleep(0.3)
                    self.esp.stop()
                    
                    # 3. Tutup capit (Grip Max)
                    self.set_capit(lift=45, grip=80)
                    time.sleep(1.0)
                    
                    # 4. Angkat botol (Lift Atas) agar tidak menyentuh lantai saat jalan
                    self.set_capit(lift=100, grip=80)
                    time.sleep(1.0)
                    
                    print("[AKSI] Botol diamankan! Mencari Watering Zone...")
                    self.state = "CARI_POT"

                # ---- TAHAP 4: MENCARI POT/WATERING ZONE ----
                elif self.state == "CARI_POT":
                    # Cukup cek apakah ada zona yang terdeteksi oleh vision.py
                    zona_pot_terlihat = len(env.get("zones", {})) > 0
                    
                    if zona_pot_terlihat:
                        print("[AI] Pot/Watering Zone terlihat! Mulai mendekat...")
                        self.esp.stop()
                        self.state = "DEKATI_POT"
                    elif jarak_depan < self.BATAS_HALANGAN:
                        self.manuver_menghindar()
                    else:
                        print("[NAV] Membawa botol, eksplorasi mencari pot...")
                        self.esp.forward(self.KECEPATAN_CARI)

                # ---- TAHAP 5: MENDEKATI POT ----
                elif self.state == "DEKATI_POT":
                    # Ambil daftar semua zona yang terdeteksi
                    zona_pot_list = list(env.get("zones", {}).values())
                    
                    if len(zona_pot_list) == 0:
                        print("[AI] Kehilangan jejak pot!")
                        self.state = "CARI_POT"
                    else:
                        target_pot = zona_pot_list[0]
                        # Hitung posisi pot (KIRI/KANAN/TENGAH)
                        cx = target_pot["center"][0]
                        kiri_limit = env["left_limit"]
                        kanan_limit = env["right_limit"]
                        
                        posisi_pot = "CENTER"
                        if cx < kiri_limit: posisi_pot = "LEFT"
                        elif cx > kanan_limit: posisi_pot = "RIGHT"
                        
                        if jarak_depan < self.JARAK_AMBIL + 5.0: 
                            print("[AI] Sampai di Watering Zone!")
                            self.esp.stop()
                            self.state = "TARUH_BOTOL"
                        else:
                            self.arahkan_ke_target(posisi_pot, self.KECEPATAN_DEKAT)
                # ---- TAHAP 6: MENARUH BOTOL (SELESAI) ----
                elif self.state == "TARUH_BOTOL":
                    print("[AKSI] Meletakkan botol di sebelah pot...")
                    # 1. Turunkan lift
                    self.set_capit(lift=45, grip=80)
                    time.sleep(1.0)
                    
                    # 2. Buka capit
                    self.set_capit(lift=45, grip=30)
                    time.sleep(1.0)
                    
                    # 3. Mundur sedikit agar botol terlepas dengan aman
                    self.esp.backward(50)
                    time.sleep(1.0)
                    self.esp.stop()
                    
                    print("[MISSION] MISI SELESAI! Botol berhasil diantarkan.")
                    self.state = "SELESAI"
                    
                elif self.state == "SELESAI":
                    # Misi selesai, robot diam di tempat (atau bisa diprogram pulang ke homebase)
                    self.esp.stop()
                    time.sleep(1)

                # ==========================================
                # Opsi Debugging (Tampilkan Kamera di Layar)
                # ==========================================
                # self.vision.draw_debug(frame, bottles, env)
                # cv2.imshow("Robot View", frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break

        except KeyboardInterrupt:
            print("\n[SYSTEM] Dihentikan oleh user.")
        finally:
            self.esp.stop()
            self.vision.close()
            # if hasattr(self, 'lidar'): self.lidar.stop()

if __name__ == "__main__":
    robot = AutonomousRobot()
    robot.run()