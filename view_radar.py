import serial
import time
import os

PORT = '/dev/ttyUSB0'
BAUD = 115200

def run_ascii_radar():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.reset_input_buffer()
    
    # Simpan jarak 360 derajat (dalam mm)
    scan_360 = [0.0] * 360
    last_draw = time.time()

    print("Memulai Radar Visual... Berdiri minimal 20 cm dari sensor.")
    time.sleep(1)

    try:
        while True:
            # 1. Cari Header 0xAA 0x55
            b1 = ser.read(1)
            if not b1 or b1[0] != 0xAA:
                continue
            b2 = ser.read(1)
            if not b2 or b2[0] != 0x55:
                continue

            ct = ser.read(1)
            lsn = ser.read(1)
            if not ct or not lsn:
                continue

            sample_count = lsn[0]
            if sample_count == 0:
                continue

            angles_and_cs = ser.read(6)
            if len(angles_and_cs) < 6:
                continue

            fsa = (angles_and_cs[0] | (angles_and_cs[1] << 8)) >> 1
            start_angle = fsa / 64.0

            lsa = (angles_and_cs[2] | (angles_and_cs[3] << 8)) >> 1
            end_angle = lsa / 64.0

            sample_bytes = ser.read(sample_count * 2)
            if len(sample_bytes) < sample_count * 2:
                continue

            angle_diff = end_angle - start_angle
            if angle_diff < 0:
                angle_diff += 360.0

            for i in range(sample_count):
                raw_dist = sample_bytes[2*i] | (sample_bytes[2*i + 1] << 8)
                dist_mm = raw_dist / 4.0

                if sample_count > 1:
                    angle = start_angle + (angle_diff / (sample_count - 1)) * i
                else:
                    angle = start_angle
                
                deg = int(angle % 360)
                # Filter noise jarak (hanya simpan data 15 cm s.d 8 meter)
                if 150 <= dist_mm <= 8000:
                    scan_360[deg] = dist_mm

            # Render tampilan radar setiap 0.1 detik
            if time.time() - last_draw > 0.1:
                last_draw = time.time()
                
                def get_min_dist(deg_range):
                    vals = [scan_360[d] for d in deg_range if scan_360[d] > 0]
                    return min(vals) if vals else 9999

                # Pembagian 8 Sektor Arah Mata Angin
                front = get_min_dist(list(range(345, 360)) + list(range(0, 16)))
                front_right = get_min_dist(range(16, 75))
                right = get_min_dist(range(75, 105))
                back_right = get_min_dist(range(105, 165))
                back = get_min_dist(range(165, 195))
                back_left = get_min_dist(range(195, 255))
                left = get_min_dist(range(255, 285))
                front_left = get_min_dist(range(285, 345))

                def fmt(val):
                    if val >= 9999:
                        return "   ---   "
                    return f"{val/10:6.1f} cm"

                # Bersihkan layar dan gambar HUD
                print("\033[H\033[J", end="")
                print("================ RADAR PEMINDAI 360 DERAJAT ================")
                print(f"                    [DEPAN 0°]                           ")
                print(f"                   {fmt(front)}                          ")
                print(f"  [KIRI DPN]                          [KANAN DPN]        ")
                print(f" {fmt(front_left)}                     {fmt(front_right)} ")
                print(f"                                                         ")
                print(f" [KIRI 270°]          [ROBOT]          [KANAN 90°]       ")
                print(f" {fmt(left)}                            {fmt(right)}     ")
                print(f"                                                         ")
                print(f"  [KIRI BLK]                          [KANAN BLK]        ")
                print(f" {fmt(back_left)}                      {fmt(back_right)} ")
                print(f"                   {fmt(back)}                           ")
                print(f"                  [BELAKANG 180°]                        ")
                print("=========================================================")
                print("Tips: Objek terlalu dekat (<15 cm) masuk zona blind spot.")

    except KeyboardInterrupt:
        print("\nRadar dimatikan.")
    finally:
        ser.close()

if __name__ == '__main__':
    run_ascii_radar()