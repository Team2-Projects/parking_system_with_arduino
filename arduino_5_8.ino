#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <IRremote.hpp>
#include <TimerFreeTone.h>
#include <Servo.h>

// =====================================================
// 기본 설정
// =====================================================

// iR 리시브 센서
const int IR_RECEIVE_PIN = 3;

// 서보 모터
const int MOTOR_IN_PIN = 6;
const int MOTOR_OUT_PIN = 5;

// 부저
const int PIEZO_PIN = 4;

// 서보모터
Servo servoIn; 
Servo servoOut;

const int inMelody[] = { 
  262, 330, 392, 523
};
const int outMelody[] = { 
  523, 392, 330, 262                 
};

// LCD 주소: 안 뜨면 0x3F로 변경
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 초음파 센서 1
const int TRIG1 = 13;
const int ECHO1 = 12;

// 초음파 센서 2
const int TRIG2 = 11;
const int ECHO2 = 10;

// DS1302 RTC 핀
const int RTC_CLK = 9;
const int RTC_DAT = 8;
const int RTC_RST = 7;

// ThreeWire 순서 중요: DAT, CLK, RST
ThreeWire myWire(RTC_DAT, RTC_CLK, RTC_RST);
RtcDS1302<ThreeWire> Rtc(myWire);

// 감지 설정
const int DETECT_DISTANCE = 10;                   // 10cm 이하 감지
const unsigned long DETECT_TIME = 2000;          // 1초 이상 감지
const unsigned long STOP_TIME = 1000;            // STOP 1초 표시
const unsigned long OPEN_SHOW_TIME = 1500;       // OPEN 최소 표시 시간
const unsigned long ENTRY_NO_DETECT_TIME = 2000; // OPEN → PARKED로 상태를 전환할 때, 입차 센서가 일정 시간 동안 감지되지 않는 기준을 의미
const unsigned long WRONG_EXIT_DETECT_TIME = 500; // 주차 상태에서 입차 센서에 특정 시간 이상 감지되면 오류라고 판단.
const unsigned long EXIT_SENSOR_DETECT_TIME = 1000; // 주차 상태에서 출차 센서에 감지 and 입차 센서에 미감지가 특정 시간 이상 지속되면 출차 단계로 전환.

const unsigned long RESULT_SHOW_TIME = 3000;     // 결과 3초 표시
const unsigned long WRONG_SHOW_TIME = 2000;      // 잘못된 방향 2초 표시

const unsigned long Exit_duration = 2000; // 출차하는 시간을 고려하여 REDAY로 전환한다.

int melodyIndex = 0; //소리 출력 인덱스(크기: 4) 
unsigned long lastToneTime = 0; // ()

unsigned long inPreviousMillis = 0; // 
unsigned long outPreviousMillis = 0;
const long interval = 15;
int inAngle = 0;
int outAngle = 180;

bool remoteControlFlag = false;

// =====================================================
// 전체 상태 관리
// =====================================================

enum SystemState {
  READY,
  ENTRY_DETECTING,
  SHOW_STOP,
  SHOW_OPEN,
  PARKED,
  EXIT_SENSOR1_DETECTED,
  WRONG_EXIT_ORDER,
  EXIT_OPEN,
  SHOW_RESULT,
  REMOTE_MOTOR_IN_OPEN,
  REMOTE_MOTOR_IN_CLOSE,
  REMOTE_MOTOR_OUT_OPEN,
  REMOTE_MOTOR_OUT_CLOSE
};

SystemState state = READY;
unsigned long stateStartMillis = 0;

// =====================================================
// 전역 데이터
// =====================================================

// 센서 상태
double sensor1Distance = 0;
double sensor2Distance = 0;

bool sensor1Detected = false;
bool sensor2Detected = false;
bool anyDetected = false;

// 입차/출차 시간
RtcDateTime enterTime;
RtcDateTime exitTime;

bool hasEnterTime = false;

// 입차 감지 시작 시간
unsigned long entryDetectStartMillis = 0;

// 입차 센서가 물체를 감지하지 못한 시각.(여기서 특정 시간동안 지나도 감지를 못하면 PARKED)
unsigned long entryDetect_for_parked_Millis = 0;

// 주차 상태에서 입차 센서에 물체가 감지된 시각.
unsigned long wrongExitDetectStartMillis = 0;

// 주차 상태에서 입차 센서에 미감지 및 출차 센서에 감지가 된 시각.
unsigned long exitSensorDetectStartMilli = 0;

// 출차 상황에서 출차 센서에 감지가 안된 시각(시점).
unsigned long exitClearStartMillis = 0;

// 주차 결과
unsigned long parkingSeconds = 0;
unsigned long parkingFee = 0;

// =====================================================
// 센서 읽기
// =====================================================

double readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) {
    return -1;
  }

  double distance = duration * 0.0343 / 2;

// -1이 나오는 경우가 생각보다 빈번하게 일어나므로 0보다 낮게 나오면 최대값인 400으로 반환되게끔 방어 로직 구성.
  if (distance < 0 || distance > 400) {
    return 400;
  }

  return distance;
}

void updateSensors() {
  sensor1Distance = readDistanceCM(TRIG1, ECHO1);
  delay(50);

  sensor2Distance = readDistanceCM(TRIG2, ECHO2);
  delay(50);

  sensor1Detected = sensor1Distance >= 0 && sensor1Distance <= DETECT_DISTANCE;
  sensor2Detected = sensor2Distance >= 0 && sensor2Distance <= DETECT_DISTANCE;

  anyDetected = sensor1Detected || sensor2Detected;
}


// =====================================================
// LCD 출력
// =====================================================

void lcdPrintTwoDigits(int value) {
  if (value < 10) {
    lcd.print("0");
  }

  lcd.print(value);
}

void lcdPrintTime(RtcDateTime time) {
  lcdPrintTwoDigits(time.Hour());
  lcd.print(":");
  lcdPrintTwoDigits(time.Minute());
  lcd.print(":");
  lcdPrintTwoDigits(time.Second());
}

void lcdPrintDuration(unsigned long totalSeconds) {
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;

  if (hours < 10) lcd.print("0");
  lcd.print(hours);
  lcd.print(":");

  if (minutes < 10) lcd.print("0");
  lcd.print(minutes);
  lcd.print(":");

  if (seconds < 10) lcd.print("0");
  lcd.print(seconds);
}

void showReady() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready");

  lcd.setCursor(0, 1);
  lcd.print("Wait Car");
}

void showStop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("STOP");
}

void showOpen() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Time ");
  lcdPrintTime(enterTime);

  lcd.setCursor(0, 1);
  lcd.print("OPEN");
}

void showParked() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("PARKED");

  lcd.setCursor(0, 1);
  lcd.print("Wait Exit");
}

void showWrongExitOrder() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("WRONG WAY");

  lcd.setCursor(0, 1);
  lcd.print("S1 -> S2");
}

void showParkingResult() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Park ");
  lcdPrintDuration(parkingSeconds);

  lcd.setCursor(0, 1);
  lcd.print("Fee ");
  lcd.print(parkingFee);
  lcd.print("W");
}


// =====================================================
// RTC 시간 처리
// =====================================================

void setupRTC() {
  Rtc.Begin();

  Rtc.SetIsWriteProtected(false);
  Rtc.SetIsRunning(true);

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

  if (!Rtc.IsDateTimeValid()) {
    Rtc.SetDateTime(compiled);
  }

  // RTC 시간을 다시 맞출 때만 아래 줄 주석 해제해서 한 번 업로드
  // 시간이 맞으면 다시 주석 처리하고 업로드
  // Rtc.SetDateTime(compiled);
}

RtcDateTime getCurrentTime() {
  return Rtc.GetDateTime();
}

unsigned long getTimeDifferenceSeconds(RtcDateTime startTime, RtcDateTime endTime) {
  uint32_t startSeconds = startTime.TotalSeconds();
  uint32_t endSeconds = endTime.TotalSeconds();

  if (endSeconds >= startSeconds) {
    return endSeconds - startSeconds;
  }

  return 0;
}


// =====================================================
// 요금 계산
// =====================================================

unsigned long calculateFee(unsigned long seconds) {
  if (seconds == 0) {
    return 0;
  }

  // 10초당 100원, 1~10초도 100원으로 계산
  return ((seconds + 9) / 10) * 100;
}


// =====================================================
// 시리얼 출력 보조
// =====================================================

void serialPrintTwoDigits(int value) {
  if (value < 10) {
    Serial.print("0");
  }

  Serial.print(value);
}

void serialPrintTime(RtcDateTime time) {
  serialPrintTwoDigits(time.Hour());
  Serial.print(":");
  serialPrintTwoDigits(time.Minute());
  Serial.print(":");
  serialPrintTwoDigits(time.Second());
}

void printDebugSensor() {
  Serial.print("S1: ");
  Serial.print(sensor1Distance);
  Serial.print(" cm / S2: ");
  Serial.print(sensor2Distance);
  Serial.println(" cm");
}


// =====================================================
// 전체 상태 관리
// =====================================================

void changeState(SystemState nextState) {
  state = nextState;
  stateStartMillis = millis();

  switch (state) {
    case READY:
      showReady();
      Serial.println("STATE: READY");
      break;

    case ENTRY_DETECTING:
      Serial.println("STATE: ENTRY_DETECTING");
      break;

    case SHOW_STOP:
      showStop();
      Serial.println("STATE: SHOW_STOP");
      break;

    case SHOW_OPEN:
      showOpen();
      Serial.println("STATE: SHOW_OPEN");
      break;

    case PARKED:
      noTone(PIEZO_PIN);
      showParked();
      Serial.println("STATE: PARKED");
      break;

    case EXIT_SENSOR1_DETECTED:
      Serial.println("STATE: EXIT_SENSOR1_DETECTED");
      break;

    case WRONG_EXIT_ORDER:
      showWrongExitOrder();
      Serial.println("STATE: WRONG_EXIT_ORDER");
      break;

    case SHOW_RESULT:
      showParkingResult();
      Serial.println("STATE: SHOW_RESULT");
      break;
    
    case EXIT_OPEN:
      Serial.println("STATE: EXIT_OPEN");
      break;
  }
}


// =====================================================
// 입차 처리
// =====================================================

void handleReady() {
  if (sensor1Detected) {
    entryDetectStartMillis = millis();
    changeState(ENTRY_DETECTING);
  }
}

void handleEntryDetecting() {
  if (!sensor1Detected) {
    changeState(READY);
    return;
  }

  if (millis() - entryDetectStartMillis >= DETECT_TIME) {
    enterTime = getCurrentTime();
    hasEnterTime = true;

    Serial.print("Enter Time: ");
    serialPrintTime(enterTime);
    Serial.println();

    changeState(SHOW_STOP);
  }
}

void handleShowStop() {
  if (millis() - stateStartMillis >= STOP_TIME) {
    changeState(SHOW_OPEN);
  }
}

void handleShowOpen() {
// OPEN을 특정 시간동안 보여준다.
  //if (millis() - stateStartMillis < OPEN_SHOW_TIME) {
    //return;
  //}

  if (!sensor1Detected) {

// 센서 1에 미감지된 순간에 entryDetect_for_parked_Millis에 현재 시각 대입. (목적: 미감지 상태를 얼마나 유지하는가를 측정.)
    if (entryDetect_for_parked_Millis == 0) {
      entryDetect_for_parked_Millis = millis();
    }

    if (millis() - entryDetect_for_parked_Millis >= ENTRY_NO_DETECT_TIME) {
      changeState(PARKED);
      entryDetect_for_parked_Millis = 0;
    }

  } 
  // 만일 감지가 되면 다시 0으로 초기화.
  else {
    entryDetect_for_parked_Millis = 0;
  }
}


// =====================================================
// 출차 처리
// =====================================================

void handleParked() {

  // 입차 센서가 일정 시간 이상 연속 감지되면 역주행으로 판단..이라는 현실적인 상황이 있지만 (이 시스템에서는 맞지않는 부분이 많다.)

  // 센서가 튀는 값으로 인한 상태 전환 방지용 로직이다.
  if (sensor1Detected) {

    // 처음 감지된 시점 저장
    if (wrongExitDetectStartMillis == 0) {
      wrongExitDetectStartMillis = millis();
    }

    // 감지 상태가 일정 시간 유지되면 오류 상태 전환
    if (millis() - wrongExitDetectStartMillis >= WRONG_EXIT_DETECT_TIME) {
      changeState(WRONG_EXIT_ORDER);
      wrongExitDetectStartMillis = 0;
    }

  } 
  else {

    // 만일 미감지 상태면 0으로 초기화.
    wrongExitDetectStartMillis = 0;
  }

  // 정상적인 출차 시작 감지
  if (!sensor1Detected && sensor2Detected) {

    // 처음 출차 센서가 감지된 시점 저장
    if (exitSensorDetectStartMilli == 0) {
      exitSensorDetectStartMilli = millis();
    }

    // 감지 상태가 일정 시간 유지되면 출차 상태 전환
    if (millis() - exitSensorDetectStartMilli >= EXIT_SENSOR_DETECT_TIME) {
      changeState(EXIT_SENSOR1_DETECTED);
      exitSensorDetectStartMilli = 0;
    }

  } 
  else {

    // 만일 미감지 상태면 0으로 초기화.
    exitSensorDetectStartMilli = 0;
  }
}

void handleExitSensor1Detected() {
// 출차 과정은 연속적인 과정이므로 상태가 변하는 시점을 기준으로 계산해도 무방하다.
// 자연스러운 상황을 위해, 조금의 딜레이만 주자.
  if (sensor2Detected && millis() - stateStartMillis > 200) {
    exitTime = getCurrentTime();

    parkingSeconds = getTimeDifferenceSeconds(enterTime, exitTime);
    parkingFee = calculateFee(parkingSeconds);

    Serial.print("Exit Time: ");
    serialPrintTime(exitTime);
    Serial.println();

    Serial.print("Parking Seconds: ");
    Serial.println(parkingSeconds);

    Serial.print("Fee: ");
    Serial.print(parkingFee);
    Serial.println(" won");

    hasEnterTime = false;

    changeState(SHOW_RESULT);
    return;
  }
}

void handleWrongExitOrder() {
  // 잘못된 방향 화면을 2초 이상 보여주고, 센서가 비워지면 다시 출차 대기 상태로 돌아감
  if (millis() - stateStartMillis >= WRONG_SHOW_TIME && !anyDetected) {
    changeState(PARKED);
  }
}

void handleShowResult() {
  // 결과를 5초 보여주고, 모터 동작
  if (millis() - stateStartMillis >= RESULT_SHOW_TIME) {
    changeState(EXIT_OPEN);
  }
}

void handleExitOpen() {

  // 아무것도 감지되지 않을 때
  if (!anyDetected) {

    // 아무것도 감지되지 않는 시점 저장.
    if (exitClearStartMillis == 0) {
      exitClearStartMillis = millis();
    }

    // 비감지 상태가 일정 시간 유지되면 READY
    if (millis() - exitClearStartMillis >= Exit_duration) {
      changeState(READY);
      exitClearStartMillis = 0;
    }

  } else {

    // 다시 감지되면 초기화
    exitClearStartMillis = 0;
  }
}

//부저 ()
void piezo(){
  //state 가 SWHO_OPEN일 경우 입차할때 소리 울림
  if (state == SHOW_OPEN) {
    if (millis() - lastToneTime > 200) {
      TimerFreeTone(PIEZO_PIN, inMelody[melodyIndex], 200);
      //TimerFreeTone(핀넘버, 주파수, 소리 출력시간(ms))

      melodyIndex++;
      if (melodyIndex >= 4) melodyIndex = 0;

      lastToneTime = millis();
    }
  } else if (state == EXIT_OPEN) { // 출차할 때 소리 울림
    if (millis() - lastToneTime > 200) {
      TimerFreeTone(PIEZO_PIN, outMelody[melodyIndex], 180);

      melodyIndex++;
      if (melodyIndex >= 4) melodyIndex = 0;

      lastToneTime = millis();
    }
  }else { // 둘다 아니라면 인덱스 초기화
    melodyIndex = 0;
  }
}

//모터 컨트롤
void motorControl(){
  unsigned long currentMillis = millis(); // 현재 시간 저장

  // state가 입차 또는 리모컨 동작 시
  if (state == SHOW_OPEN || state == REMOTE_MOTOR_IN_OPEN) { // in motor open (0 ~ 90)
    if (currentMillis - inPreviousMillis >= interval) {
      inPreviousMillis = currentMillis;

      if (inAngle < 90) {
        inAngle +=6;   // X 단계씩 이동
        servoIn.write(inAngle);
      }
    }
  } 
  if (state == PARKED || state == REMOTE_MOTOR_IN_CLOSE) { // in motor close (90 ~ 0)
    if (currentMillis - inPreviousMillis >= interval) {
      inPreviousMillis = currentMillis;

      if (inAngle > 0) {
        inAngle-=6;   // 한 단계씩 이동
        servoIn.write(inAngle);
      }
    }
  } 
  if (state == EXIT_OPEN || state == REMOTE_MOTOR_OUT_OPEN) { // out motor open (90 ~ 180)
    if (currentMillis - outPreviousMillis >= interval) {
      outPreviousMillis = currentMillis;

      if (outAngle < 180) {
        outAngle+=6;   // 한 단계씩 이동
        servoOut.write(outAngle);
      }
    }
  } 
  if (state == READY || state == REMOTE_MOTOR_OUT_CLOSE) { // out motor close (180 ~ 90)
    if (currentMillis - outPreviousMillis >= interval) {
      outPreviousMillis = currentMillis;

      if (outAngle > 90) {
        outAngle-=6;   // 한 단계씩 이동
        servoOut.write(outAngle);
      }
    }
  }
}


//IR 리시버 센서
void IrReceiverSensor(){
  if (IrReceiver.decode()) {  // 적외선 센서의 수신값 해석
    uint8_t cmd = IrReceiver.decodedIRData.command;
    Serial.print("cmd: ");
    Serial.println(cmd);
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      //수동 모드로 전환
      //remoteControlFlag = true;
      //state READY로 바꿔놓음
      //state = READY;
      //showReady();

      if (cmd == 12) { // 1번 in motor open
        remoteControlFlag = true;
        Serial.println("Action: In Motor");
        state = REMOTE_MOTOR_IN_OPEN;
      }else if (cmd == 24) { // 2번 in motor close
        remoteControlFlag = true;
        Serial.println("Action: Out Motor");
        state = REMOTE_MOTOR_IN_CLOSE;
      }else if (cmd == 94) {  // 3번 out motor open
        remoteControlFlag = true;
        Serial.println("IR: OUT OPEN");
        state = REMOTE_MOTOR_OUT_OPEN;
      }
      else if (cmd == 8) {  // 4번 out motor close
        remoteControlFlag = true;
        Serial.println("IR: OUT CLOSE");
        state = REMOTE_MOTOR_OUT_CLOSE;
      }else if(cmd == 22){  // 0번 리모컨 컨트롤 해제
        remoteControlFlag = false;
        sensor1Detected = false;
        sensor2Detected = false;
        state = READY;
        showReady();
      }
    }

    IrReceiver.resume();
  }
}


// =====================================================
// Arduino 기본 함수
// =====================================================

void setup() {
  Serial.begin(9600);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  lcd.init();
  lcd.backlight();

  setupRTC();

  changeState(READY);

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

void loop() {
  updateSensors();
  // 초음파 센서를 기반으로 입/출차 센서 감지 상태를 수정한다.
  IrReceiverSensor();
  // 수동으로 차단기 조절. 1번을 누르면 REMOTE_MOTOR_IN_OPEN상태가 되는데, 이 상태에서는 motorControl함수의 영향으로 입차 모터가 개방된다. 
  // 수동 및 자동 조절에서 모터의 조절은 motorControl에 의해 수행되며, state에 맞게끔 모터가 조절된다. (ex: SHOW_OPEN 및 REMOTE_MOTOR_IN_OPEN일 때, 입차 모터 개방.)
  // IrReceiverSensor는 적외선 리모컨의 동작을 기반으로 자동 차단기 조절과 관련된 state를 수정한다.
  motorControl();
  // 각 state에 맞는 모터 동작을 수행.
  printDebugSensor();
  // 입/출차 센서에 감지되는 물체와의 거리를 시리얼 모니터에 출력. (주석처리해도 알고리즘에는 문제 없다.)

  // 아래 구문은 수동 조절 상황이 아닐 때(remoteControlFlag = false) 동작.
  if(!remoteControlFlag){
    switch (state) {
      case READY:
        handleReady();
        //만일 입차 센서에 감지가 되고, 출차 센서에는 감지가 안되면 state를 ENTRY_DETECTING로 바꾼다.
        // 혹시나 초음파센서에 값이 튀는 값을 잘못 인식할 수도 있지만, handleEntryDetecting함수의 기능으로 다시 Ready로 돌아온다.
        break;

      case ENTRY_DETECTING:
        handleEntryDetecting();
        // 입차 센서에 감지가 안되면 READY, 1초 이상 감지된 상태를 유지하면, 입차 상태를 true로 바꾸고 SHOW_STOP으로 넘어간다.
        break;

      case SHOW_STOP:
        handleShowStop();
        // 특정 시간(STOP_TIME)이상 정차해 있으면, OPEN으로 상태가 변한다.
        break;

      case SHOW_OPEN:
        handleShowOpen();
        // 일단 입차 모터가 올라간다. 그리고 원래는 입/출차 센서에 감지가 안되면 PARKED가 되는 거였지만, 입차 센서에 특정 시간(ENTRY_NO_DETECT_TIME)이상 감지가 안되면 PARKED가 되는 거로 수정. 
        break;

      case PARKED:
        handleParked();
        // PARKED상태는 물체가 입차 센서 범위를 통과했다는 의미다. (PARKED상태가 된 순간, 모터가 닫히기 시작한다.)
        // 입차 센서에 특정 시간이상 물체가 감지되면 WRONG
        // 입차 센서에 미감지 and 출차 센서에 감지가 특정 시간 유지되면 EXIT_SENSOR1_DETECTED
        break;

      case EXIT_SENSOR1_DETECTED:
        handleExitSensor1Detected();
        //출차 센서에 특정 시간 이상 감지되면 요금료 및 주차 시간 계산, 그리고 SHOW_RESULT로 전환.
        break;

      case WRONG_EXIT_ORDER:
        handleWrongExitOrder();
        break;

      case SHOW_RESULT:
        handleShowResult();
        // 특정시간동안 주차시간 및 요금료 LCD에 출력하고 EXITOPEN
        break;

      case EXIT_OPEN:
        handleExitOpen();
        //출차 모터가 열린다. 그리고 아무것도 감지가 안되면 READY로 전환.
        break;
    }
  }

  piezo();
}