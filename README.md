# RC_LED

-----------------------------------------
영상

https://youtu.be/C98HGoryuOc?si=kkGU_v5Z_UBAd2JV

 ----------------------------------------
| RC 채널 매핑 | <-> | 핀 매핑 |
|:-----:|-------:|:----------------|
| CH5 | A2 | line tracing mode on/off |
|CH2| A1 | RC car 방향 조절 |
|CH1| A0| RC car 속도 조절|

-----------------------------------------

| A2 연결 핀| A1 연결 핀| A0 연결 핀|
|:---------:|:---------:|:-----------:|
|4|3|9,10,11|

PWM 기능을 위해 3,9,10,11 번 사용했습니다.

----------------------------------------
회로도 구성

아두이노 pin A0, A1, A2 , 4, 3 ,9 ,10 ,11

수신기 CH 1, 2, 5


-------------------------------------------
주요 아이디어
-

Picamera2 + OpenCV 로 바닥 검은 선(Line)을 감지해서 ──▶  Arduino 로 L / R / F / B 명령 전송 <br>
Flask 로 MJPEG 실시간 스트림을 보여주며 (라즈베리파이용 경량 비동기 설계) <br>

1) camera_loop ──> Picamera2 프레임 캡처 → 방향 분석 → 주기적으로 시리얼 전송 <br>
2) detect_direction() ──> ROI(관심영역) 내에서 adaptiveThreshold + contour 로 선 중심 계산
   · 선이 없으면 'B' (후진) 반환 <br>
3) send_dir() ──> 같은 방향이라도 RESEND_FRAMES 마다 한 번 더 전송 (ESC 브레이크↔후진 대응) <br>
4) Flask 는 latest_frame 바이트를 MJPEG boundary 로 스트리밍 <br>

※ "L" 헤더 + 1 문자 명령 형태(LF/LL/LR/LB) : Arduino 스케치와 맞춤
※ 좌우 보정: 카메라 영상을 1 회 flip 한 좌표계로 통일

-------------------------------------------
문제해결
-

1문제 : RC car 속도 조절을 위한 maping (decode) <br>
-
-> 문제 이유 : 조종기에서 보내는 펄스를 그대로 받아 maping하면
              속도가 너무 빨라 pulse를 최대로 줄 경우 장비 파손 문제 발생 <br>
-> 해결 : 손수 임의 값을 대입하여 적당한 속도의 후진, 정지 ,적당한 속도의 전진 값을 찾아냈습니다. <br>

2문제 : line tracing 인식 박스 조절 <br>
-
-> 문제 이유 : 화면에 보이는 검은 선을 따라 라인 트레이싱을 해야하는데
             차량 앞에 나온 범퍼(검정색)로 인해 부적합한 인식을 하게 되는 문제 발생 <br>
-> 해결 : 인식 범위를 범퍼 위로 수정했습니다.
기존 코드 <br>
![image](https://github.com/user-attachments/assets/31e8441e-d1d3-4768-9182-f7a294634a7a) <br>
![image](https://github.com/user-attachments/assets/597e1981-e031-4db1-a981-261950807e33) <br>

아래로 변경 <br>
![image](https://github.com/user-attachments/assets/b4039fb6-66c8-4685-9860-9c70a18f2486) <br>
![image](https://github.com/user-attachments/assets/b640d3a9-1ead-4a60-9290-4e4eaea93f95) <br>

3문제 : flask를 활용하는 웹 스트리밍 속도 <br>
-
-> 문제 이유 : 화면에 나오는 선을 따라 line tracing 을 하게 되는데 지연율로 인해 화면이 멈추게 되는 문제 발생 <br>

-> 해결 : 해상도를 낮추어 저지연율로 전송 <br>

기존 FRAME_W, FRAME_H = 640 , 480 에서 320 , 240 으로 만들었습니다. <br>
![image](https://github.com/user-attachments/assets/dfe33734-58a2-4fd8-89a7-cfc4dcd478c8) <br>
![image](https://github.com/user-attachments/assets/aa53b3be-07c5-4309-af7d-649e14a1fd73) <br>


4문제 : 기본 RC car 설정 변경 <br>
-
-> 문제 이유 : 기본 RC car 설정은 전진중 후진을 입력 받으면 정지됨 그러므로 코드를 두번 연속으로 입력 받아야함 <br>
               이를 해결하기 위해 코드 상에서 연속으로 줄 것인지 시스템을 바꿀지 선택해야 함 <br>

-> 해결 : 시스템을 변경하는 것이 쉬운 선택이라 판단 <br>
          전진중 후진을 입력 받으면 즉시 후진으로 변경 <br>







