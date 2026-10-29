import ydlidar
import math
import threading
import time
import config
class LidarNavigator:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.laser = ydlidar.CYdLidar()
        
        # Konfigurasi YDLIDAR
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, baudrate)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 3)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, True)
        
        # Inisialisasi jarak sektor maksimal (999.0 = tidak ada halangan)
        self.sectors = {
            "front": 999.0,
            "front_left": 999.0,
            "left": 999.0,
            "back": 999.0,
            "right": 999.0,
            "front_right": 999.0,
        }
        self.closest_obstacle = 999.0
        
        self.running = False
        self.thread = None

    def get_scan_points(self):
        """Ambil titik LiDAR dalam koordinat cm."""
        import math

        scan = ydlidar.LaserScan()

        if not self.laser.doProcessSimple(scan):
            raise RuntimeError("Scan LiDAR gagal")

        points = []
        offset = config.LIDAR_FRONT_OFFSET_DEG

        for point in scan.points:
            distance = float(point.range) * 100

            if not math.isfinite(distance):
                continue

            if distance <= 0:
                continue

            angle = math.degrees(point.angle)
            angle = (angle - offset + 180) % 360 - 180

            radians = math.radians(angle)

            points.append({
                "angle": angle,
                "distance": distance,
                "x": distance * math.cos(radians),
                "y": distance * math.sin(radians)
            })

        if len(points) < config.LIDAR_MIN_POINTS:
            raise RuntimeError("Titik LiDAR tidak cukup")

        return points

    def start(self):
        """Memulai proses Lidar di thread latar belakang."""
        ret = self.laser.initialize()
        if ret and self.laser.turnOn():
            self.running = True
            self.thread = threading.Thread(target=self._scan_loop, daemon=True)
            self.thread.start()
            print("[LIDAR] Sistem Navigasi aktif di latar belakang.")
            return True
        else:
            print("[LIDAR ERROR] Gagal menginisialisasi port Lidar.")
            return False

    def _scan_loop(self):
        """Loop internal untuk menarik data secara terus menerus."""
        scan = ydlidar.LaserScan()
        while self.running and ydlidar.os_isOk():
            r = self.laser.doProcessSimple(scan)
            if r:
                self._process_scan(scan)
            time.sleep(0.02) # Jeda ringan agar CPU tidak overload

    def _process_scan(self, scan):
        """Menerjemahkan data Point Cloud mentah ke dalam zona jarak sektor."""
        # Variabel penampung sementara untuk 1 putaran (scan)
        temp_sectors = {k: 999.0 for k in self.sectors.keys()}
        closest = 999.0
        
        for i in range(scan.points.size()):
            point = scan.points[i]
            if point.range == 0:
                continue  # Abaikan nilai cacat / pantulan gagal
            
            dist_cm = point.range * 100  # Konversi meter ke centimeter
            angle_deg = math.degrees(point.angle) # Konversi radian ke derajat
            
            # Normalisasi sudut dari rentang (-180..180) ke (0..360)
            if angle_deg < 0:
                angle_deg += 360
                
            if dist_cm < closest:
                closest = dist_cm
                
            # Pembagian Sektor Berdasarkan Derajat (0 derajat adalah depan)
            if angle_deg <= 30 or angle_deg >= 330:
                temp_sectors["front"] = min(temp_sectors["front"], dist_cm)
            elif 30 < angle_deg <= 90:
                temp_sectors["front_left"] = min(temp_sectors["front_left"], dist_cm)
            elif 90 < angle_deg <= 150:
                temp_sectors["left"] = min(temp_sectors["left"], dist_cm)
            elif 150 < angle_deg <= 210:
                temp_sectors["back"] = min(temp_sectors["back"], dist_cm)
            elif 210 < angle_deg <= 270:
                temp_sectors["right"] = min(temp_sectors["right"], dist_cm)
            elif 270 < angle_deg < 330:
                temp_sectors["front_right"] = min(temp_sectors["front_right"], dist_cm)
                
        # Perbarui nilai global
        self.sectors = temp_sectors
        self.closest_obstacle = closest

    def get_sectors(self):
        """Dipanggil oleh main.py untuk mendapatkan jarak (cm) rintangan terdekat per sektor."""
        return self.sectors

    def stop(self):
        """Mematikan motor Lidar dan menutup port dengan aman."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        self.laser.turnOff()
        self.laser.disconnecting()
        print("[LIDAR] Sistem Navigasi dihentikan.")
