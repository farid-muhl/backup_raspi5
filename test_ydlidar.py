import serial
import struct
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

def parse_ydlidar():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.reset_input_buffer()
    print("Memulai parsing data YDLIDAR (Tekan Ctrl+C untuk berhenti)...")

    try:
        while True:
            # 1. Cari Header 0xAA 0x55
            b1 = ser.read(1)
            if not b1 or b1[0] != 0xAA:
                continue
            b2 = ser.read(1)
            if not b2 or b2[0] != 0x55:
                continue

            # 2. Baca informasi paket (CT & LSN)
            ct = ser.read(1)
            lsn = ser.read(1)
            if not ct or not lsn:
                continue

            sample_count = lsn[0]
            if sample_count == 0:
                continue

            # 3. Baca FSA (Sudut Awal) & LSA (Sudut Akhir) + Checksum (Total 6 byte)
            angles_and_cs = ser.read(6)
            if len(angles_and_cs) < 6:
                continue

            fsa = (angles_and_cs[0] | (angles_and_cs[1] << 8)) >> 1
            start_angle = fsa / 64.0

            lsa = (angles_and_cs[2] | (angles_and_cs[3] << 8)) >> 1
            end_angle = lsa / 64.0

            # 4. Baca array titik jarak (2 byte per titik * sample_count)
            sample_bytes = ser.read(sample_count * 2)
            if len(sample_bytes) < sample_count * 2:
                continue

            # 5. Hitung interpolasi sudut & jarak
            angle_diff = end_angle - start_angle
            if angle_diff < 0:
                angle_diff += 360.0

            front_distances = []

            for i in range(sample_count):
                raw_dist = sample_bytes[2*i] | (sample_bytes[2*i + 1] << 8)
                dist_mm = raw_dist / 4.0  # Formula jarak YDLIDAR X2

                if sample_count > 1:
                    angle = start_angle + (angle_diff / (sample_count - 1)) * i
                else:
                    angle = start_angle
                angle = angle % 360.0

                # Filter sektor depan robot (345° sampai 15°)
                if (angle >= 345 or angle <= 15) and dist_mm > 0:
                    front_distances.append(dist_mm)

            # Tampilkan deteksi rintangan terdekat di sektor depan
            if front_distances:
                min_front = min(front_distances)
                print(f"[SEKTOR DEPAN] Rintangan terdekat: {min_front:.1f} mm")
            else:
                print("[SEKTOR DEPAN] Bersih.")

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nBerhenti.")
    finally:
        ser.close()

if __name__ == '__main__':
    parse_ydlidar()
