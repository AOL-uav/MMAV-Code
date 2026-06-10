from flask import Flask, render_template_string, request, jsonify
import serial
import time
import threading

app = Flask(__name__)

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# --- SERIAL SETUP ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)  # Wait for serial to initialize
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

telemetry = {"yaw": 1500, "pitch": 1500, "state": "UNKNOWN"}

def read_serial():
    global telemetry
    while True:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("POS:"):
                    parts = line[4:].split(',')
                    if len(parts) >= 2:
                        telemetry["yaw"] = int(parts[0])
                        telemetry["pitch"] = int(parts[1])
                        telemetry["state"] = "ACTIVE"
                elif "RELAXED" in line:
                    telemetry["state"] = "RELAXED"
                elif "CENTERED" in line:
                    telemetry["state"] = "CENTERED"
            except:
                pass
        time.sleep(0.05)

threading.Thread(target=read_serial, daemon=True).start()

def send_command(cmd):
    if ser:
        ser.write(f"{cmd}\n".encode())
        print(f"Sent: {cmd}")

# --- WEB ROUTES ---
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Gimbal Web Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: sans-serif; text-align: center; background: #121212; color: white; padding: 20px; }
        .card { background: #1e1e1e; padding: 20px; border-radius: 15px; margin-bottom: 20px; box-shadow: 0 4px 6px rgba(0,0,0,0.3); }
        .slider { width: 80%; margin: 20px 0; height: 25px; }
        button { background: #007bff; color: white; border: none; padding: 15px 30px; border-radius: 8px; font-size: 16px; margin: 5px; cursor: pointer; }
        button.red { background: #dc3545; }
        button.green { background: #28a745; }
        .telemetry { font-family: monospace; color: #00ff00; background: black; padding: 10px; border-radius: 5px; display: inline-block; margin-top: 10px; }
    </style>
</head>
<body>
    <h1>25kg Gimbal Control</h1>
    
    <div class="card">
        <h3>Yaw (Pin 0)</h3>
        <input type="range" min="60" max="120" value="90" class="slider" id="yawSlider" oninput="updateVal('Y', this.value)">
        <span id="yawLabel">90</span>°
    </div>

    <div class="card">
        <h3>Pitch (Pin 1)</h3>
        <input type="range" min="60" max="120" value="90" class="slider" id="pitchSlider" oninput="updateVal('P', this.value)">
        <span id="pitchLabel">90</span>°
    </div>

    <div class="card">
        <button onclick="sendCommand('C')">Center All</button>
        <button class="red" onclick="sendCommand('S')">Relax (Kill)</button>
        <button class="green" onclick="sendCommand('A')">Auto Mode</button>
    </div>

    <div class="card">
        <strong>Telemetry:</strong><br>
        <div class="telemetry">
            State: <span id="tele_state">---</span><br>
            Yaw: <span id="tele_yaw">---</span>us | 
            Pitch: <span id="tele_pitch">---</span>us
        </div>
    </div>

    <script>
        function updateVal(axis, val) {
            document.getElementById(axis == 'Y' ? 'yawLabel' : 'pitchLabel').innerText = val;
            fetch(`/move?axis=${axis}&val=${val}`);
        }
        function sendCommand(cmd) {
            fetch(`/cmd?c=${cmd}`);
        }
        function updateTelemetry() {
            fetch('/status').then(r => r.json()).then(data => {
                document.getElementById('tele_state').innerText = data.state;
                document.getElementById('tele_yaw').innerText = data.yaw;
                document.getElementById('tele_pitch').innerText = data.pitch;
            });
        }
        setInterval(updateTelemetry, 500);
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/move')
def move():
    axis = request.args.get('axis')
    val = request.args.get('val')
    send_command(f"{axis}{val}")
    return jsonify(success=True)

@app.route('/cmd')
def cmd():
    c = request.args.get('c')
    send_command(c)
    return jsonify(success=True)

@app.route('/status')
def status():
    return jsonify(telemetry)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
