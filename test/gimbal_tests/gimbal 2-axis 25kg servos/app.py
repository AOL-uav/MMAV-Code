from flask import Flask, render_template, request, jsonify, Response
import serial
import time
import os
import threading
import queue

app = Flask(__name__)

# Serial configuration
SERIAL_PORT = None
BAUD_RATE = 115200
ser = None
telemetry_queue = queue.Queue(maxsize=100)

def find_serial_port():
    ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']
    for p in ports:
        if os.path.exists(p):
            return p
    return None

def get_serial():
    global ser, SERIAL_PORT
    if ser is None or not ser.is_open:
        SERIAL_PORT = find_serial_port()
        if SERIAL_PORT:
            try:
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                time.sleep(1)
                print(f"Connected to {SERIAL_PORT}")
            except Exception as e:
                print(f"Reconnect failed: {e}")
                ser = None
    return ser

# Background thread to read serial and put into queue
def serial_reader():
    while True:
        s = get_serial()
        if s:
            try:
                if s.in_waiting > 0:
                    line = s.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        if telemetry_queue.full():
                            telemetry_queue.get()
                        telemetry_queue.put(line)
            except:
                pass
        time.sleep(0.01)

threading.Thread(target=serial_reader, daemon=True).start()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/stream')
def stream():
    def generate():
        while True:
            try:
                line = telemetry_queue.get(timeout=1)
                yield f"data: {line}\n\n"
            except queue.Empty:
                yield ": keep-alive\n\n"
    return Response(generate(), mimetype='text/event-stream')

@app.route('/control', methods=['POST'])
def control():
    s = get_serial()
    data = request.json
    axis = data.get('axis')
    angle = data.get('angle')
    
    if s and axis:
        try:
            if axis == 'C': s.write(b"C\n")
            elif axis == 'S': s.write(b"S\n")
            elif axis == 'A': s.write(b"A\n")
            elif angle is not None:
                cmd = f"{axis}{angle}\n"
                s.write(cmd.encode())
            return jsonify({"status": "ok"})
        except Exception as e:
            global ser
            ser = None
            return jsonify({"status": "error", "message": str(e)}), 500
            
    return jsonify({"status": "error", "message": "Serial not connected"}), 400

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
