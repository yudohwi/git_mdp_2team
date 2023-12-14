import serial
import time

# 시리얼 포트 설정
serial_port = serial.Serial("COM6", 115200, timeout=1)

def send_data_to_flask(data):
    try:
        # 시리얼을 통해 데이터 전송
        serial_port.write(data.encode())
        print(f"Flask로 데이터 전송: {data}")
    except Exception as e:
        print(f"Flask로 데이터 전송 중 오류 발생: {e}")

if __name__ == "__main__":
    try:
        while True:
            # 이 부분을 센서 또는 다른 원본에서 데이터를 수집하는 로직으로 대체하세요
            chair1_data = "의자1데이터"  # 실제 데이터로 대체하세요
            chair2_data = "의자2데이터"  # 실제 데이터로 대체하세요

            # 데이터를 Flask가 이해할 수 있는 형식으로 결합 (예: CSV)
            combined_data = f"{chair1_data},{chair2_data}"

            # Flask 서버로 데이터 전송
            send_data_to_flask(combined_data)

            time.sleep(1)  # 필요에 따라 지연 시간을 조정하세요

    except KeyboardInterrupt:
        serial_port.close()
        print("시리얼 포트가 닫혔습니다.")