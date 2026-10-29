import time
from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200  # Ganti ke 256000 jika tipe A2/A3 masih gagal

def main():
    print(f"Menghubungkan ke LiDAR di {PORT_NAME} ({BAUDRATE} baud)...")
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=3)

    try:
        # Bersihkan sisa data serial yang menumpuk di buffer
        lidar.clean_input()
        time.sleep(0.5)

        # Reset modul sensor untuk sinkronisasi ulang
        print("Mereset modul sensor...")
        lidar.reset()
        time.sleep(2.0)  # Beri jeda agar motor & board stabil

        lidar.clean_input()

        info = lidar.get_info()
        health = lidar.get_health()
        print(f"Info Perangkat : {info}")
        print(f"Status Sensor  : {health}")

        print("\nMembaca data scan...")
        for scan in lidar.iter_scans(max_buf_meas=500):
            print(f"Berhasil membaca frame pindaian: {len(scan)} titik laser")
            break

    except Exception as e:
        print(f"Terjadi kesalahan: {e}")
    finally:
        print("Menghentikan LiDAR...")
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass

if __name__ == '__main__':
    main()
