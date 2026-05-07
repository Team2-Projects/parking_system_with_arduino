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

int angle_in = 0;
int angle_out = 180;
// 부저
const int PIEZO_PIN = 4;

// 서보모터
Servo servoIn; 
Servo servoOut;

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
const unsigned long DETECT_TIME = 1000;          // 1초 이상 감지
const unsigned long STOP_TIME = 1000;            // STOP 1초 표시
const unsigned long OPEN_SHOW_TIME = 1500;       // OPEN 최소 표시 시간
const unsigned long RESULT_SHOW_TIME = 5000;     // 결과 5초 표시
const unsigned long WRONG_SHOW_TIME = 2000;      // 잘못된 방향 2초 표시

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
  SHOW_RESULT
};

SystemState state = READY;
unsigned long stateStartMillis = 0;

// =====================================================
// 전역 데이터
// =====================================================

// 센서 상태
double sensor1Distance = -1;
// 입차 센서가 측정한 물체와의 거리.
double sensor2Distance = -1;
// 출차 센서가 측정한 물체와의 거리.
bool sensor1Detected = false;
// 입차 센서와 물체와의 거리가 특정 거리이하인가? 
bool sensor2Detected = false;
// 출차 센서와 물체와의 거리가 특정 거리이하인가? 
bool anyDetected = false;
// 입 or 출차 센서와 물체와의 거리가 특정 거리이하인가? 

// 입차/출차 시간
RtcDateTime enterTime;
RtcDateTime exitTime;

bool hasEnterTime = false;
// ??


// 입차 감지 시작 시간
unsigned long entryDetectStartMillis = 0;

// 주차 결과 (각각 주차장 이용시간: (exitTime - enterTime)/ 요금료)
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

  return duration * 0.0343 / 2;
}

// 초음파 센서와 물체와의 거리를 계산하는 하뭇

void updateSensors() {
  sensor1Distance = readDistanceCM(TRIG1, ECHO1);
  delay(50);

  sensor2Distance = readDistanceCM(TRIG2, ECHO2);
  delay(50);

  sensor1Detected = sensor1Distance > 0 && sensor1Distance <= DETECT_DISTANCE;
  sensor2Detected = sensor2Distance > 0 && sensor2Distance <= DETECT_DISTANCE;

  anyDetected = sensor1Detected || sensor2Detected;
}
// 알고리즘의 핵심 함수로 입/출차 센서가 물체를 감지하였는지를 확인하는 코드 (ex: 입차센서가 물체를 감지하면 sensor1Detected = True)

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

// 위 함수들은 LCD에 시간 출력 형식을 지정하는 함수. 

void showReady() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready");

  lcd.setCursor(0, 1);
  lcd.print("Wait Car");
}
// Ready상태에서 동작하는 함수로, Ready/ Wait Car를 LCD에 출력한다.
void showStop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("STOP");
}
// Stop상태에서 동작하는 함수로, Ready/ Wait Car를 LCD에 출력한다.

void showOpen() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcdPrintTime(enterTime);

  lcd.setCursor(0, 1);
  lcd.print("OPEN");
}
// Open상태에서 동작하는 함수로, Time/ Wait Car를 LCD에 출력한다. 
 

void showParked() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("PARKED");

  lcd.setCursor(0, 1);
  lcd.print("Wait Exit");
}
// 물체가 입차센서를 완전히 통과하면 Parked: PARKED/Wait Exit 출력.

void showWrongExitOrder() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("WRONG WAY");

  lcd.setCursor(0, 1);
  lcd.print("S1 -> S2");
}

// ()

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

// 출차센서에 감지되면 exittime을 설정하고, 주차 시간 및 요금을 출력한다.

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
      // Ready출력
      Serial.println("STATE: READY");
      break;

    case ENTRY_DETECTING:
      Serial.println("STATE: ENTRY_DETECTING");
      break;

    case SHOW_STOP:
      showStop();
      // Stop출력
      Serial.println("STATE: SHOW_STOP");
      break;

    case SHOW_OPEN:
      showOpen();
      // 현재 시간 및 Open출력
      Serial.println("STATE: SHOW_OPEN");
      break;

    case PARKED:
      showParked();
      // Parked 및 Wait exit출력
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
    
      piezoOut();
      showParkingResult();
      // 주차 시간 및 요금 출력.
      Serial.println("STATE: SHOW_RESULT");
      break;
  }
}


// =====================================================
// 입차 처리
// =====================================================

void handleReady() {
  if (anyDetected) {
    entryDetectStartMillis = millis();
    changeState(ENTRY_DETECTING);
  }
}
// loop에서 Ready상태일 때, 실행할 함수. 입/출차 센서에 물체가 감지되면 ENTRY_DETECTING으로 상태가 변한다.
void handleEntryDetecting() {
  if (!anyDetected) {
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
// loop에서 ENTRY_DETECTING상태일 때, 실행할 함수. 만일 아무것도 감지가 안되면 READY로 돌아가고, 감지된 상태면 hasEntertime (입차 기록)이 true가 된다. 그리고 입차시각을 enterTime에 저장.
void handleShowStop() {
  if (millis() - stateStartMillis >= STOP_TIME) {
    changeState(SHOW_OPEN);
  }
}
// loop에서 STOP상태일 때, 실행할 함수. 특정 시간(STOP_TIME)동안 STOP상태가 지속되고, 이후에는 OPEN으로 상태를 바꾼다.
void handleShowOpen() {
  // OPEN 화면을 최소 시간 동안 보여줌
  if (millis() - stateStartMillis < OPEN_SHOW_TIME) {
    return;
  }

  // 센서가 완전히 비워진 뒤에 PARKED 상태로 전환
  if (!anyDetected) {
    changeState(PARKED);
  }
}
// loop에서 OPEN상태일 때, 실행할 함수. 센서가 완전히 비워질 때까지 OPEN유지. 비워지면 PARKED로 전환.

// =====================================================
// 출차 처리
// =====================================================

void handleParked() {
  // 정상 출차 시작:
  // 센서1이 감지되면 센서2가 같이 감지되어도 정상 순서 시작으로 인정
  if (sensor2Detected) {
    changeState(EXIT_SENSOR1_DETECTED);
    return;
  }
// loop에서 PARKED상태일 때, 실행할 함수. 출차 센서에 감지되면 EXIT_SENSOR1_DETECTED로 전환.

  // 잘못된 순서:
  // 센서1은 전혀 안 잡히고 센서2만 먼저 잡힌 경우
  
}

void handleExitSensor1Detected() {
  // 센서1 이후 센서2가 감지되면 출차 처리
  if (sensor2Detected) {
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
// loop에서 EXIT_SENSOR1_DETECTED상태일 때, 실행할 함수. exitTime에 출차시각 저장하고, 주차 시간 및 요금 계산 hasEnterTime은 false로 변화 기록. 그리고 SHOW_RESULT로 전환.

// 나갈 때는 물체가 나가지 않아도 소리가 멈추고 모터가 내려가는 이유는 EXIT_SENSOR1_DETECTED상태일 때, 모터와 부저가 작동하기 때문이다. 즉 위 함수가 끝나면 부저는 꺼진다. (이건 물체 감지 유무와는 상관이 없음.)
void handleWrongExitOrder() {
  // 잘못된 방향 화면을 2초 이상 보여주고, 센서가 비워지면 다시 출차 대기 상태로 돌아감
  if (millis() - stateStartMillis >= WRONG_SHOW_TIME && !anyDetected) {
    changeState(PARKED);
  }
}

void handleShowResult() {
  // 결과를 5초 보여주고, 센서가 비워지면 Ready로 복귀
  if (millis() - stateStartMillis >= RESULT_SHOW_TIME && !anyDetected) {
    changeState(READY);
  }
}
// loop에서 SHOW_RESULT상태일 때, 실행할 함수. 특정 시간(RESULT_SHOW_TIME) 이 지남 및 물체가 감지되지 않으면 다시 Ready로 상태를 전환한다. 즉 이 상태일때, outMotorOpen()를 실행하면 될 듯 하다.

//In 부저 소리
void piezoIn(){
  int size = sizeof(inMelody) / sizeof(inMelody[0]);
  for(int i = 0; i < size; i++){
    TimerFreeTone(PIEZO_PIN, inMelody[i], 200);
    if (angle_in<=90){
      servoIn.write(angle_in);
      angle_in +=5;
    }
    
  }
}

//Out 부저 소리
void piezoOut(){
  int size = sizeof(outMelody) / sizeof(outMelody[0]);
  for(int i = 0; i < size; i++){
    TimerFreeTone(PIEZO_PIN, outMelody[i], 200);
    if (angle_out>=90){
      servoOut.write(angle_out);
      angle_out -=5;
    }
  }
}

//In 모터
void inMotorOpen(){
  piezoIn();
    
}

void inMotorClose(){
  while (angle_in >0){
    servoIn.write(angle_in);
    angle_in -=1;
    delay(10);
  }
}

//Out 모터
void outMotorOpen(){
    piezoOut();
}

void outMotorClose(){
  while (angle_out <180){
    servoOut.write(angle_out);
    angle_out +=1;
    delay(10);
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

  switch (state) {
    case READY:
      outMotorClose();
      handleReady();
      break;

    case ENTRY_DETECTING:
      handleEntryDetecting();
      break;

    case SHOW_STOP:
      handleShowStop();
      break;

    case SHOW_OPEN:
      handleShowOpen();
      inMotorOpen();
      break;

    case PARKED:
      noTone(PIEZO_PIN);
      handleParked();
      inMotorClose();
      break;

    case EXIT_SENSOR1_DETECTED:
      handleExitSensor1Detected();
      
      break;

    case WRONG_EXIT_ORDER:
      handleWrongExitOrder();
      break;

    case SHOW_RESULT:
      outMotorOpen();
      handleShowResult();
      break;
  }
}