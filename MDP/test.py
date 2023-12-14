from flask import Flask, render_template
import serial
import threading

ser = serial.Serial("COM9", 115200)

# 문자열을 저장할 변수를 생성합니다.
received_data = "0,0"

# 시리얼 데이터를 읽는 스레드 함수
def read_serial_data():
    global received_data
    while True:        
        # readline() 함수는 한 줄 읽을 때까지 계속 멈춰있다.
        received_data = ser.readline().decode().strip()

# 데몬 속성을 활성화 했다는 말은 백그라운드에서 계속 실행하고 있다는 말이야.
serial_thread = threading.Thread(target=read_serial_data)
serial_thread.daemon = True
serial_thread.start()

app = Flask(__name__)

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
