#include <IRremote.hpp>
#include <TimerFreeTone.h>
#include <Servo.h>

// iR 리시브 센서
#define IR_RECEIVE_PIN 3

// 서보 모터
#define MOTOR_IN_PIN 6
#define MOTOR_OUT_PIN 5

// 부저
#define PIEZO_PIN 4

// 서보모터
Servo servoIn; 
Servo servoOut;

// 모터 핀 정의
int MOTOR_IN_ANGLE = 0;
int MOTOR_OUT_ANGLE = 0;

// 모터 움직임 시작 시간
unsigned long inStartTime = 0;
unsigned long outStartTime = 0;

// 모터 상태
bool inActive = false;
bool outActive = false;

// 피에조 변수
int inMelodyIndex = 0;
unsigned long inMelodyTime = 0;
bool inMelodyPlaying = false;
int outMelodyIndex = 0;
unsigned long outMelodyTime = 0;
bool outMelodyPlaying = false;

const int inMelody[] = { 
  262, 330, 392, 523
};
const int outMelody[] = { 
  523, 392, 330, 262                 
};

void setup(){
  Serial.begin(9600);

  pinMode(PIEZO_PIN, OUTPUT);

  // 모터 접근
  servoIn.attach(MOTOR_IN_PIN);
  servoOut.attach(MOTOR_OUT_PIN);

  // 모터 각도 초기화
  servoIn.write(0);
  servoOut.write(180);

  // IR 리시브 수신 시작
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

//In 부저 소리
void piezoIn(){
  int size = sizeof(inMelody) / sizeof(inMelody[0]);
  for(int i = 0; i < size; i++){
    TimerFreeTone(PIEZO_PIN, inMelody[i], 200);
  }
}

//Out 부저 소리
void piezoOut(){
  int size = sizeof(outMelody) / sizeof(outMelody[0]);
  for(int i = 0; i < size; i++){
    TimerFreeTone(PIEZO_PIN, outMelody[i], 200);
  }
}

//In 모터
void inMotorOpen(){
  if(!inActive){
    servoIn.write(90); 
    piezoIn();
    inStartTime = millis();
    inActive = true;
  }
}

void inMotorClose(){
  servoIn.write(0);
  inActive = false;
}

//Out 모터
void outMotorOpen(){
  if(!outActive){
    servoOut.write(90); 
    piezoOut();
    outStartTime = millis();
    outActive = true;
  }
}

void outMotorClose(){
  servoOut.write(180);
  outActive = false;
}

void loop(){
  // IR 리시브 신호
  if (IrReceiver.decode()) {  // 적외선 센서의 수신값 해석
    uint8_t cmd = IrReceiver.decodedIRData.command;

    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      if (cmd == 12) { // 1번 버튼 (버튼값은 리모컨마다 다를 수 있음)
        Serial.println("Action: In Motor");
        inMotorOpen();
      }
      else if (cmd == 24) { // 2번 버튼
        Serial.println("Action: Out Motor");
        outMotorOpen();
      }
    }

    IrReceiver.resume();
  }

  // 닫힘 처리
  if(inActive && millis() - inStartTime > 5000){
    inMotorClose();
  }

  if(outActive && millis() - outStartTime > 5000){
    outMotorClose();
  }
}