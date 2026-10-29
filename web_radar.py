import serial
import math
import json
import threading
from flask import Flask, render_template_string

PORT = '/dev/ttyUSB0'
BAUD = 115200

app = Flask(__name__)
points_data = []
lock = threading.Lock()

def lidar_worker():
    global points_data
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        ser.reset_input_buffer()
        temp_points = []
        last_angle = 0

        while True:
            b1 = ser.read(1)
            if not b1 or b1[0] != 0xAA:
                continue
            b2 = ser.read(1)
            if not b2 or b2[0] != 0x55:
                continue

            ct = ser.read(1)
            lsn = ser.read(1)
            if not ct or not lsn or lsn[0] == 0:
                continue
            
            sample_count = lsn[0]
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
                
                angle = angle % 360.0
                
                # Filter jarak valid (15 cm - 6 meter)
                if 150 < dist_mm < 6000:
                    rad = math.radians(angle)
                    # Konversi Polar (r, theta) ke Kartesius (x, y)
                    x = dist_mm * math.sin(rad)
                    y = dist_mm * math.cos(rad)
                    temp_points.append({"x": round(x, 1), "y": round(y, 1)})

            # Jika pindaian sudah selesai satu putaran (reset sudut)
            if start_angle < last_angle:
                with lock:
                    points_data = list(temp_points)
                temp_points = []
            last_angle = start_angle

    except Exception as e:
        print(f"Error LiDAR: {e}")

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Visualisasi Peta LiDAR 2D</title>
    <style>
        body { background: #111; color: #00ffcc; font-family: monospace; text-align: center; margin: 0; }
        canvas { background: #000; border: 1px solid #333; margin-top: 10px; }
    </style>
</head>
<body>
    <h2>Live Point Cloud LiDAR (YDLIDAR X2)</h2>
    <canvas id="radar" width="600" height="600"></canvas>
    <p>Skala: 1 pixel = 1 cm (Jangkauan Radius: 3 Meter)</p>
    <script>
        const canvas = document.getElementById('radar');
        const ctx = canvas.getContext('2d');
        const cx = 300, cy = 300;
        const scale = 0.1; // 10 mm = 1 px

        function drawGrid() {
            ctx.clearRect(0, 0, 600, 600);
            ctx.strokeStyle = '#222';
            ctx.lineWidth = 1;
            for(let r=50; r<=280; r+=50) {
                ctx.beginPath();
                ctx.arc(cx, cy, r, 0, 2*Math.PI);
                ctx.stroke();
            }
            // Sumbu silang
            ctx.strokeStyle = '#333';
            ctx.beginPath();
            ctx.moveTo(cx, 0); ctx.lineTo(cx, 600);
            ctx.moveTo(0, cy); ctx.lineTo(600, cy);
            ctx.stroke();

            // Posisi Robot
            ctx.fillStyle = '#ff0055';
            ctx.beginPath();
            ctx.arc(cx, cy, 6, 0, 2*Math.PI);
            ctx.fill();
        }

        async function updateScan() {
            try {
                const res = await fetch('/data');
                const pts = await res.json();
                drawGrid();
                
                ctx.fillStyle = '#00ff66';
                for(let p of pts) {
                    let px = cx + (p.x * scale);
                    let py = cy - (p.y * scale);
                    ctx.fillRect(px, py, 2, 2);
                }
            } catch(e) {}
            requestAnimationFrame(updateScan);
        }
        updateScan();
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/data')
def get_data():
    with lock:
        return json.dumps(points_data)

if __name__ == '__main__':
    t = threading.Thread(target=lidar_worker, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000)
