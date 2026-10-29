import time
from serial_comm import ESPController
from vision import VisionSystem
from lidar_nav import LidarNavigator
from mapper import ArenaMap
from lidar_obstacle import ObstacleDetector
import config

class MissionController:
    def __init__(self):
        self.esp = ESPController(
            port=config.SERIAL_PORT,
            baudrate=config.SERIAL_BAUD,
            timeout=config.SERIAL_TIMEOUT,
        )
        self.vision = VisionSystem(model_path="my_model.pt")
        self.lidar = LidarNavigator()
        self.lidar.start() # Nyalakan Lidar saat robot baru hidup
        self.map = ArenaMap()
        self.state = "EXPLORE"  # EXPLORE, NAV_STATION, SCANNING_BOTTLE, APPROACHING_BOTTLE, GRIPPING, NAV_ZONE, PLACING_BOTTLE, EVALUATING, RETURN_HOME
        self.bottles_delivered = {"red": 0, "yellow": 0, "green": 0, "blue": 0}
        self.target_bottle = None
        self.ultrasonic_dist = 999.0
        self.next_ultrasonic_read = 0.0
        self.obstacle_detector = ObstacleDetector()
        self.tunggu_setelah_putar = False

    def run(self):
        print("[MISSION] Starting autonomous mission...")
        try:
            while True:
                # 1. Ambil frame dari kamera
                frame = self.vision.capture_frame()
                if frame is None:
                    time.sleep(0.05)
                    continue
                    try:
                        lidar_points = self.lidar.get_scan_points()

                        navigation = self.obstacle_detector.analyze(
                            lidar_points
                        )

                        lidar_status = navigation["status"]
                        lidar_action = navigation["action"]

                        print(
                            f"[LIDAR] {lidar_status} -> {lidar_action}"
                        )

                    except Exception as error:
                        print("[LIDAR ERROR]", error)
                        self.esp.stop()
                        continue


                # 2. Deteksi environment dan botol
                env = self.vision.detect_environment(frame)
                bottles = self.vision.detect_bottles(frame)

                # 3. Update sensor data dari ESP
                now = time.monotonic()
                sensor_sample_fresh = now >= self.next_ultrasonic_read
                # 1. Baca data sensor Ultrasonik dari ESP32
                sensor_dist = self.esp.read_sensor_data()
                BATAS_HALANGAN = 30.0
                # 2. Inisialisasi nilai Lidar dengan jarak aman (default) terlebih dahulu
                jarak_lidar = 999.0
                
                # 3. Baca data dari Lidar dan timpa nilai default jika Lidar benar-benar menyala
                if hasattr(self, 'lidar') and self.lidar is not None:
                    try:
                        lidar_data = self.lidar.get_sectors()
                        # Gunakan .get() agar kebal error jika key 'front' tiba-tiba hilang
                        jarak_lidar = lidar_data.get('front', 999.0)
                    except Exception as e:
                        print(f"[WARNING] Gagal membaca data Lidar: {e}")
                        
                # 4. KUNCI INTEGRASI: Ambil jarak yang PALING DEKAT di antara kedua sensor
                jarak_depan = min(sensor_dist, jarak_lidar)
                
                # Cetak log jarak
                print(f"[NAV] Jarak Depan -> Ultrasonik: {sensor_dist:.1f} cm | Lidar: {jarak_lidar:.1f} cm | Final: {jarak_depan:.1f} cm")

                # 5. Logika penghindaran halangan
                if jarak_depan < BATAS_HALANGAN:
                    print("[OBSTACLE] Halangan di depan, berputar ke kiri!")
                    self.esp.turn_left(config.SEARCH_SPEED)
                    
                    time.sleep(0.8)
                    self.esp.stop()
                    time.sleep(0.2)
                else:
                    pass # Lanjutkan misi jika jalanan aman
                if self.state == "SCANNING_BOTTLE":
                    print("[SCAN] Berputar mencari botol...")
                    self.esp.turn_right(config.SEARCH_ROTATION_SPEED)

                    if len(bottles) > 0:
                        self.esp.stop()
                        self.target_bottle = bottles[0]
                        color_name = self.target_bottle.get('color', 'Unknown')
                        print(f"[VISION] Terkunci pada botol: {color_name}")
                        self.state = "APPROACHING_BOTTLE"

                elif self.state == "APPROACHING_BOTTLE":
                    if len(bottles) > 0:
                        self.target_bottle = bottles[0]
                        cx = self.target_bottle["center"][0]
                        pusat_kamera = config.FRAME_CENTER_X
                        error_x = cx - pusat_kamera
                        
                        # Cek deadzone untuk menyelaraskan arah pandang (yaw)
                        bottle_aligned = abs(error_x) <= config.ALIGNED_THRESHOLD

                        if (
                            sensor_sample_fresh
                            and sensor_dist is not None
                            and sensor_dist <= config.ULTRASONIC_APPROACH_DISTANCE_CM
                            and bottle_aligned
                        ):
                            print(f"[APPROACH] Jarak optimal ({sensor_dist:.1f} cm). Memulai gripping.")
                            self.esp.stop()
                            self.state = "GRIPPING"
                        else:
                            # Kontrol pergerakan proporsional
                            if bottle_aligned:
                                self.esp.forward(config.APPROACH_SPEED)
                            elif error_x < 0:
                                self.esp.arc_left(config.MIN_SPEED)
                            else:
                                self.esp.arc_right(config.MIN_SPEED)
                    else:
                        print("[APPROACH] Kehilangan visual target, kembali mencari...")
                        self.esp.stop()
                        self.state = "SCANNING_BOTTLE"  # Kembali ke tahap SCAN jika botol keluar frame

                elif self.state == "GRIPPING":
                    print("[ACTION] Mengeksekusi siklus jepitan...")
                    self.esp.stop() # Robot harus diam 100% sebelum mencapit
                    self.esp.grab_bottle(verbose=True)
                    time.sleep(config.GRAB_SEQUENCE_TIME) # Waktu tunggu servo bekerja
                    
                    print("[ACTION] Botol diamankan. Beralih ke Navigasi Zona.")
                    self.state = "NAV_ZONE"

                elif self.state == "NAV_ZONE":
                    if self.target_bottle is None:
                        self.esp.stop()
                        raise RuntimeError("Gagal: Botol target hilang dari memori.")

                    bottle_color = self.target_bottle.get("color")
                    target_zone = config.WATERING_ZONE_MAPPING.get(bottle_color)
                    
                    if target_zone is None:
                        self.esp.stop()
                        raise RuntimeError(f"Warna botol {bottle_color!r} tidak memiliki parameter zona.")

                    zone = env.get("zones", {}).get(target_zone)
                    if zone is None:
                        print(f"[NAV] Mencari zona peletakan {target_zone}...")
                        self.esp.turn_right(config.SEARCH_ROTATION_SPEED)
                    else:
                        zone_center_x = zone["center"][0]
                        error_x = zone_center_x - config.FRAME_CENTER_X
                        
                        if abs(error_x) <= config.ALIGNED_THRESHOLD:
                            if zone["area"] < config.WATERING_ZONE_STOP_AREA:
                                self.esp.forward(config.APPROACH_SPEED)
                            else:
                                self.esp.stop()
                                print(f"[NAV] Mencapai {target_zone}; bersiap melepaskan.")
                                self.state = "PLACING_BOTTLE"
                        elif error_x < 0:
                            self.esp.arc_left(config.MIN_SPEED)
                        else:
                            self.esp.arc_right(config.MIN_SPEED)

                elif self.state == "PLACING_BOTTLE":
                    print("[ACTION] Melepaskan botol ke zona...")
                    self.esp.stop()
                    self.esp.release_bottle(verbose=True)
                    time.sleep(config.RELEASE_SEQUENCE_TIME) # Jeda untuk pelepasan mekanis servo
                    
                    print("[ACTION] Bermanuver mundur (Post-Placement Retreat)...")
                    # Pastikan ESPController memiliki fungsi backward() jika baris ini diaktifkan:
                    # self.esp.backward(config.MIN_SPEED) 
                    # time.sleep(config.REVERSE_DURATION)
                    self.esp.stop()

                    bottle_class = self.target_bottle.get("class_name", bottle_color)
                    self.bottles_delivered[bottle_class] = self.bottles_delivered.get(bottle_class, 0) + 1
                    self.target_bottle = None
                    self.state = "EXPLORE"

                # 5. Visual debug
                if getattr(config, 'ENABLE_VISUALIZATION', False):
                    self.vision.draw_debug(frame, bottles, env)

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n[MISSION] Dihentikan paksa (KeyboardInterrupt). Mematikan sistem...")
            self.esp.stop()
            if hasattr(self.vision, 'close'):
                self.vision.close()

    def nav_to_station(self):
        print("[NAV] Moving to water station...")
        self.esp.forward(config.SEARCH_SPEED)
        time.sleep(2)
        self.esp.stop()
        self.lidar.stop()

if __name__ == "__main__":
    robot = MissionController()
    robot.run()
