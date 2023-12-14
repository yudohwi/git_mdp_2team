from flask import Flask, render_template
import serial
import threading

app = Flask(__name__)

ser = serial.Serial("COM6", 115200)
received_data = "0,0"

def read_serial_data():
    global received_data
    while True:
        received_data = ser.readline().decode().strip()

# 시리얼 데이터를 읽는 스레드 함수
serial_thread = threading.Thread(target=read_serial_data)
serial_thread.daemon = True
serial_thread.start()

# 애플리케이션 종료 시 실행할 함수
@app.teardown_appcontext
def teardown_appcontext(exception=None):
    # 시리얼 포트 닫기
    ser.close()
    print("Serial port is closed.")

@app.route('/')
def index():
    print(received_data)
    data_from_stm32 = received_data
    return render_template('index.html', data=data_from_stm32)

if __name__ == '__main__':
    app.run(debug=True)
