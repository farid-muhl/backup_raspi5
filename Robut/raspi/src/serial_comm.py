import serial

class ESPController:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200, timeout=1):
        self.ser = None
        self.connected = False
        self.current_lift = 45
        self.current_grip = 30
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            self.connected = True
            print(f"[SERIAL] Terhubung ke {port}")
        except Exception as e:
            print(f"[SERIAL ERROR] Gagal membuka port: {e}")

    def send_command(self, speed_left, speed_right):
        """Format pengiriman harus 4 angka: speedLeft, speedRight, liftAngle, gripAngle"""
        if self.connected and self.ser and self.ser.is_open:
            try:
                command = f"{speed_left},{speed_right},{self.current_lift},{self.current_grip}\n"
                self.ser.write(command.encode('utf-8'))
            except Exception as e:
                print(f"[SERIAL ERROR] Gagal mengirim perintah: {e}")

    def forward(self, speed):
        self.send_command(speed, speed)

    def backward(self, speed):
        self.send_command(-speed, -speed)

    def turn_left(self, speed):
        # Berputar ke kiri di tempat (Roda kiri mundur, roda kanan maju)
        self.send_command(-speed, speed)

    def turn_right(self, speed):
        # Berputar ke kanan di tempat (Roda kiri maju, roda kanan mundur)
        self.send_command(speed, -speed)

    def stop(self):
        self.send_command(0, 0)
        
    def read_sensor_data(self):
        """Membaca data sensor ultrasonik (format 'U:jarak') dari ESP32"""
        if not self.connected or not self.ser or not self.ser.is_open:
            return 999.0
        
        try:
            # Baca semua data yang menumpuk di buffer serial
            if self.ser.in_waiting > 0:
                lines = self.ser.readlines()
                # Kita ambil data paling terakhir (terbaru) saja
                for line in reversed(lines):
                    decoded = line.decode('utf-8').strip()
                    if decoded.startswith("U:"):
                        # Pecah string "U:10.5" menjadi float 10.5
                        return float(decoded.split(":")[1])
        except Exception as e:
            pass
            
        return 999.0  # Kembalikan jarak aman jika tidak ada data

    def close(self):
        if self.connected and self.ser:
            try:
                self.stop()
                self.ser.close()
            except:
                pass
            finally:
                self.connected = False

    def __del__(self):
        self.close()
