# main.py
from flask import Flask, render_template
from flask_socketio import SocketIO
import threading
import time
import signal
import sys
import serial

app = Flask(__name__)
socketio = SocketIO(app)

ser = None
ser_lock = threading.Lock()

# 데이터 수신 쓰레드
def uart_listener():
    while True:
        with ser_lock:
            if ser and ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                socketio.emit('uart_data', {'data': data})
        time.sleep(0.1)

def signal_handler(sig, frame):
    print("Ctrl+C pressed. Closing serial port.")
    with ser_lock:
        if ser:
            ser.close()
            ser = None
    sys.exit(0)

# 기본 경로를 /로 설정하고 home.html을 렌더링하는 라우트 추가
@app.route('/')
def home():
    return render_template('home.html')

# 다른 엔드포인트에 대한 라우트 추가 (예: /index)
@app.route('/index.html')
def index():
    return render_template('index.html')


# 웹 소켓 핸들러
@socketio.on('connect')
def handle_connect():
    print('Client connected')
    open_serial_port()  # Automatically open serial port when a client connects

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')
    close_serial_port()  # Automatically close serial port when a client disconnects

def open_serial_port():
    global ser, ser_lock  # global 선언 추가
    with ser_lock:
        ser = serial.Serial('COM5', 115200)  # 포트와 보드레이트는 실제 설정에 맞게 변경해야 합니다.

def close_serial_port():
    global ser, ser_lock  # global 선언 추가
    with ser_lock:
        if ser:
            ser.close()
            ser = None

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    uart_thread = threading.Thread(target=uart_listener)
    uart_thread.daemon = True 
    uart_thread.start()

    socketio.on_event('open_port', open_serial_port)
    socketio.on_event('close_port', close_serial_port)

    socketio.run(app, debug=True)
