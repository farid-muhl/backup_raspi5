import serial
import time

PORT = '/dev/ttyUSB0'

def test_lidar_communication(baudrate):
    print(f"\n[{baudrate} bps] Mencoba komunikasi...")
    try:
        # Buka port serial
        ser = serial.Serial(PORT, baudrate, timeout=2)
        
        # Pada A1, pin DTR sering mengontrol motor (False = On/Spin)
        ser.setDTR(False)
        time.sleep(1)
        ser.reset_input_buffer()
        
        # 1. Kirim perintah STOP (0xA5 0x25)
        ser.write(b'\xA5\x25')
        time.sleep(0.1)
        ser.reset_input_buffer()
        
        # 2. Kirim perintah GET_INFO (0xA5 0x50)
        print("Mengirim perintah GET_INFO (0xA5 0x50)...")
        ser.write(b'\xA5\x50')
        
        # Baca balasan (RPLIDAR Info packet = 7 byte header + 20 byte payload)
        raw_data = ser.read(27)
        
        if len(raw_data) > 0:
            hex_data = " ".join([f"{b:02X}" for b in raw_data])
            print(f"Balasan heksadesimal ({len(raw_data)} bytes):")
            print(hex_data)
            
            # Cek apakah 2 byte pertama adalah A5 5A (Header yang benar)
            if raw_data.startswith(b'\xA5\x5A'):
                print(f"==> SUKSES! Sensor membaca dengan benar di baud rate {baudrate}.")
            else:
                print(f"==> GAGAL: Header salah. Data acak diterima.")
        else:
            print("==> TIMEOUT: Tidak ada balasan dari sensor.")
            
        ser.close()
    except Exception as e:
        print(f"Error membuka port: {e}")

if __name__ == '__main__':
    # Uji dua baud rate paling umum untuk sensor ini
    test_lidar_communication(115200)
    test_lidar_communication(256000)
    
