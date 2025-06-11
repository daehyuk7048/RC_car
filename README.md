# RC_car

-----------------------------------------
팀원 : 신대혁, 김태현
-
라즈베리파이, 아두이노 코딩, flask를 통한 웹 연결 코딩 - 김태현 <br>
라즈베리파이, 아두이노 코딩, servo.h 를 이용한 디코딩, 라인 디텍션코딩 - 신대혁 <br>
-----------------------------------------
영상
-
https://youtu.be/nqK6RFUjnTQ?si=gnZZ4QN7GVuIHTnE <br>

 ----------------------------------------
회로도 구성
-
 
| Receiver CH | 기능 (RC -> 차량)            | Arduino 핀 |
| :---------: | :----------------------- | :-------: |
|   **CH 5**  | 매뉴얼 / 라인-트레이싱 모드 스위치     |   **A2**  |
|   **CH 2**  | **Throttle** – 속도 제어     |   **A1**  |
|   **CH 1**  | **Steering** – 방향(조향) 제어 |   **A0**  |

-----------------------------------------

| Arduino 핀 | 연결 대상        | 용도                                  |
| :-------: | :----------- | :---------------------------------- |
|   **4**   | LED\_L       | 좌측 방향 LED                           |
|   **3**   | LED\_R       | 우측 방향 LED |
|   **9**   | ESC Signal   | 모터 ESC 스로틀 PWM                      | 
|   **10**  | Servo Signal | 조향 서보 PWM                           |
|   **11**  | (Reserved)   | 예비 / 향후 확장                          |

-------------------------------------------
주요 아이디어
-

Picamera2 + OpenCV 로 바닥 검은 선(Line)을 감지해서 ──▶   Arduino 로 'P' + steerByte + thrByte 명령 전송 <br>
Flask 로 MJPEG 실시간 스트림을 보여주며 (라즈베리파이용 경량 비동기 설계) <br>
라인 재탐색 기능 및 아두이노 3바이트 프로토콜 적용 <br>

 ──────────────────────────────────────────────────────────────────────── <br>
   전체 흐름 요약                                                              <br>
   ├─ Picamera2로 프레임 캡처 → ROI 이진화로 검은 선 중심 오프셋 계산           <br>
   │                                                                           <br>
   ├─ FSM(FORWARD·REVERSING·SEARCHING) 로 주행 상태 관리                       <br>
   │   • 라인 놓치면 → 후진(REVERSING) → 좌 / 우 까딱 탐색(SEARCHING) → 복귀    <br>
   │                                                                           <br>
   ├─ calculate_control_bytes()   : 오프셋·모드별 스티어/스로틀 바이트 산출     <br>
   ├─ send_control_command()      : ‘P + 2byte’ 프로토콜로 아두이노에 전송      <br>
   ├─ camera_loop()               : ↑ 모든 비전 + FSM + TX 수행, MJPEG 프레임   <br>
   └─ Flask /video_feed           : latest_frame 를 MJPEG 스트림으로 제공       <br>
 ──────────────────────────────────────────────────────────────────────── <br>

-------------------------------------------
자율주행 동작 설명 (라즈베리파이 코드)
-
함수 detect_center_offset_and_visualize <br>
-
-> 함수 설명 : ROI 안에서 라인을 감지하고 그 위치를 표시해주는 모듈입니다. <br>
<br>
              인식 박스안에 선이 있으면 그 라인 중심에 녹색원이 생깁니다.  <br>
<br>
![image](https://github.com/user-attachments/assets/2d61f13a-8bc2-4d88-8646-d132521526ea) <br>
![image](https://github.com/user-attachments/assets/2b647b36-6feb-4b2d-b641-827eaac6ec34) <br>

함수 calculate_control_bytes <br>
-
-> 함수 설명 : 조향과 스로틀을 담당하는 모듈입니다. <br>
<br>
               FORWARD 을 받으면 라인을 감지하고 전진하며 커브 값이 크면 속도가 줄어듭니다.<br>
               REVERSING 을 받으면 후진하고 선이 없어진 뱡향에 반대로 조향을 틀어줍니다. <br>
               SEARCHING 을 받으면 좌우로 꺾어서 선이 있는지 확인합니다. <br>
               <br>
![image](https://github.com/user-attachments/assets/fa7df671-9ea8-4aff-9827-024e1d1a0f8b) <br>
![image](https://github.com/user-attachments/assets/52d9155f-297d-4511-b399-48bca58bc4c2) <br>


함수 end_control_command <br>
-
-> 함수 설명 : 아두이노로 조향과 스로틀을 보냅니다. <br>
               상태를 확인하고 불필요한 명령은 보내지 않습니다. <br>
               <br>
![image](https://github.com/user-attachments/assets/ab8d41da-95e4-4af5-a19b-2505981f54f1) <br>

함수 camera_loop <br>
-
-> 함수 설명 : 앞에 나온 3개 함수를 모아 동작합니다 <br>
               라인을 발견하면 라인을 따라갑니다. <br>
               라인을 발견하지 못하면 선을 찾아보고 그래도 찾지 못한다면 후진하여 좌우를 살피게 됩니다. <br>
               <br>
![image](https://github.com/user-attachments/assets/a944bbfa-5a9c-45cd-94c2-5421e7ec94c2) <br>
![image](https://github.com/user-attachments/assets/3e519c94-6751-4f5b-8d5a-6b66da54939b) <br>
![image](https://github.com/user-attachments/assets/6d4324ad-d2a8-45a3-a8f6-d08795c05d31) <br>
![image](https://github.com/user-attachments/assets/6a18fa99-ccff-4501-b8ee-74ea95b651fa) <br>

----------------------------------------------
자율주행 동작 설명 (아두이노 코드) <br>
-

loop() (자율주행 코드 부분만 발췌)
-
-> 라즈베리파이로부터 'P' + steerByte + thrByte 값을 받아 RCcar 매핑 <br>
<br>
![image](https://github.com/user-attachments/assets/2c4d7451-a4f8-4db1-87f3-bccc73f01569) <br>
![image](https://github.com/user-attachments/assets/06f317b6-8928-44ea-950c-15e44b244663) <br>

-------------------------------------------
문제해결
-

1문제 : RC car 속도 조절을 위한 maping (decode) <br>
-
-> 문제 이유 : 조종기에서 보내는 펄스를 그대로 받아 maping하면
              속도가 너무 빨라 pulse를 최대로 줄 경우 장비 파손 문제 발생 <br>
              <br>
-> 해결 : 손수 임의 값을 대입하여 적당한 속도의 후진, 정지 ,적당한 속도의 전진 값을 찾아냈습니다. <br>
<br>
채널 대역폭 <br>
![image](https://github.com/user-attachments/assets/351ae541-a1d7-4aa3-a436-8c96093bbac9) <br>
속도 조절된 변수 <br>
![image](https://github.com/user-attachments/assets/946b2274-42a0-4090-ad39-9b144ba558ac) <br>

2문제 : 축이 틀어져 있어 축보정 <br>
-
-> 문제 이유 : 동작을 하지 않는 정지상태에서 기본적으로 축이 10도에서 20도 정도 축이 왼쪽으로 치우져 있는 문제 발생 <br>
<br>
-> 해결 : 조금씩 대입해서 최대한 중앙에 맞췄습니다. <br>
<br>
![image](https://github.com/user-attachments/assets/2e324b3c-b71b-4142-bc26-89349da05405)<br>


3문제 : line tracing 인식 박스 조절 <br>
-
-> 문제 이유 : 화면에 보이는 검은 선을 따라 라인 트레이싱을 해야하는데 <br>
             차량 앞에 나온 범퍼(검정색)로 인해 부적합한 인식을 하게 되는 문제 발생 <br>
             <br>
-> 해결 : 인식 범위를 범퍼 위로 수정했습니다.<br>
<br>
기존 코드 <br>
![image](https://github.com/user-attachments/assets/31e8441e-d1d3-4768-9182-f7a294634a7a) <br>
![image](https://github.com/user-attachments/assets/597e1981-e031-4db1-a981-261950807e33) <br>

아래로 변경 <br>
![image](https://github.com/user-attachments/assets/45004ddd-bf83-472b-8c6c-9f856eba87c0)<br>
 <br>

4문제 : 기본 RC car 설정 변경 <br>
-
-> 문제 이유 : 기본 RC car 설정은 전진중 후진을 입력 받으면 정지됨 그러므로 코드를 두번 연속으로 입력 받아야하는 문제 발생 <br>
               이를 해결하기 위해 코드 상에서 연속으로 줄 것인지 시스템을 바꿀지 선택. <br>
<br>
-> 해결 : 시스템을 변경하는 것이 쉬운 선택이라 판단 <br>
          전진중 후진을 입력 받으면 즉시 후진으로 변경했습니다. <br>
          <br>
          
5문제 : line detection 에서 선을 발견 못하면 멈춰서 찾기만 하는 문제. <br>
-
-> 문제 이유 : 코드 상에서 후진 후에 다시 선을 찾는 코드를 해놨으나, <br>
               너무 주기가 빨라 뒤로 움직이기도 전에 다시 좌우를 확인하여 선을 찾는 행위하는 문제가 발생 <br>
<br>
-> 해결 : 보낼 때 원하는 시간 만큼 차이를 두고 전송하는 코드를 넣었습니다. <br>
![image](https://github.com/user-attachments/assets/5e73ed50-8239-4db3-9a5a-34ed73a0ba15) <br>

--------------------------------------------------------------------------------------------------
통신 프로토콜 : UART <br>
-
정신 없이 일단 만들다 보니 프로젝트 후반부에 오면서 통신을 SPI로 사용하지 않은게 많이 아쉬웠습니다. <br>





