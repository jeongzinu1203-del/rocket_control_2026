#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_BNO08x.h"
#include <ESP32Servo.h>
#include <SD.h>

// ================= [사용자 설정 파라미터] =================
// 1. 낙하산 사출 조건 (미터)
const float APOGEE_DROP_THRESHOLD = 2.0f; 

// 2. 발사 감지 높이 (미터) - 이 높이 이상 올라가야 상승으로 인정 (지상 오작동 방지)
const float LAUNCH_DETECT_ALTITUDE = 5.0f; 

// 3. 서보 모터 설정 (마이크로초 단위)
// 본인 기구물에 맞춰서 값 수정 필요 (보통 1000~2000 사이)
const int SERVO_LOCKED_US = 1000; // 잠금 상태 (출발 전)
const int SERVO_OPEN_US = 2000;   // 사출 상태 (낙하산 개방)
const int SERVO_PIN = 25;

// ========================================================

// 로켓 상태 정의
enum RocketState {
  STATE_GROUND,   // 지상 대기
  STATE_ASCENT,   // 상승 중 (최고 고도 추적)
  STATE_DESCENT   // 사출 완료 및 하강
};
RocketState currentState = STATE_GROUND;

// 구조체 정의 (기존 유지)
union Vec3 {
  struct { float x; float y; float z; };
  struct { float roll; float pitch; float yaw; };
};

struct Quat {
  float w; float x; float y; float z;
};

// 전역 변수
static const int SD_MISO = 2;
static const int SD_SCK = 14;
static const int SD_CS = 13;
static const int SD_MOSI = 23;
static const float seaLevelPressure = 1013.25f;

Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x(-1);
Servo servo;

float maxAltitude = 0.0;     // 최고 고도 저장용
float groundAltitude = 0.0;  // 지상 고도 (0점)
bool bnoOK = false;
bool sdOK = false;
bool isParachuteDeployed = false; // 사출 여부 플래그

Vec3 latestAcc = {0,0,0};
Vec3 latestPose = {0,0,0};
Quat latestPoseQuat = {0,0,0,0};
String fileName = "";

// 함수 선언
static float clampf(float, float, float);
void ScanI2C();
void setReports();
float GetAltitude();
void PollBNO();
void InitSD();
void LogCSV(float, bool);
Vec3 RotateLocalToWorld(Quat, const Vec3&);
static inline Vec3 Cross(Vec3 a, Vec3 b);
static inline Quat QuatNormalize(Quat q);

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(21, 22);
  ScanI2C();

  // 1. 서보 초기화 (잠금 상태)
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(SERVO_LOCKED_US);
  Serial.println("Servo Locked.");

  // 2. BMP390 초기화
  while (!bmp.begin_I2C()) {
    Serial.println("Connecting BMP390...");
    delay(1000);
  }
  Serial.println("BMP390 Connected!");
  // IIR 필터 중요: 센서 노이즈로 인한 오작동 방지
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(500);

  // 3. 지상 고도 0점 조절 (평균내기)
  Serial.print("Calibrating Ground Level...");
  float totalAlt = 0;
  for(int i = 0; i < 20; i++) {
    if(bmp.performReading()) {
      totalAlt += bmp.readAltitude(seaLevelPressure);
    }
    delay(50);
  }
  groundAltitude = totalAlt / 20.0;
  Serial.print(" Done. Ground Alt: "); Serial.print(groundAltitude); Serial.println(" m");

  // 4. BNO08x 초기화
  Serial.println("Connecting to BNO08x...");
  if (bno08x.begin_I2C(0x4B) || bno08x.begin_I2C(0x4A)) {
    Serial.println("[BNO08x] Connected!");
    setReports();
    bnoOK = true;
  } else {
    Serial.println("[WARNING] BNO08x Not Found.");
  }

  // 5. SD카드 초기화
  InitSD();
  
  Serial.println("\n>>> SYSTEM READY for LAUNCH <<<");
}

// ================= LOOP =================
void loop() {
  if (bnoOK && bno08x.wasReset()) setReports();

  // 1. 데이터 읽기
  float currentAlt = GetAltitude(); // 지상 기준 상대 고도 (m)
  PollBNO();
  Vec3 worldAcc = RotateLocalToWorld(latestPoseQuat, latestAcc);

  // 2. 상태 머신 (로켓 사출 로직)
  switch (currentState) {
    
    // [단계 1] 지상 대기
    case STATE_GROUND:
      // 고도가 5m 이상 올라가면 발사로 판단 (손으로 들고 이동할 때 오작동 방지)
      if (currentAlt > LAUNCH_DETECT_ALTITUDE) {
        currentState = STATE_ASCENT;
        maxAltitude = currentAlt; // 현재 고도를 초기 최고점으로 설정
        Serial.println("\n!!! LAUNCH DETECTED (Ascent Mode) !!!");
      }
      break;

    // [단계 2] 상승 중 (최고 고도 추적)
    case STATE_ASCENT:
      // 최고 고도 갱신
      if (currentAlt > maxAltitude) {
        maxAltitude = currentAlt;
      }

      // *** 핵심 사출 조건 ***
      // 최고점보다 2m 이상 떨어졌는가?
      if (currentAlt < (maxAltitude - APOGEE_DROP_THRESHOLD)) {
        // 사출 실행
        servo.writeMicroseconds(SERVO_OPEN_US); 
        isParachuteDeployed = true;
        currentState = STATE_DESCENT;
        
        Serial.println("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println("!!! APOGEE DETECTED - DEPLOY !!!");
        Serial.print("Max Alt: "); Serial.print(maxAltitude); Serial.println(" m");
        Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      }
      break;

    // [단계 3] 하강 중 (로그만 기록)
    case STATE_DESCENT:
      // 이미 사출되었으므로 추가 행동 없음
      break;
  }

  // 3. 로그 저장 및 출력
  LogCSV(currentAlt, isParachuteDeployed);

  // 시리얼 출력 (너무 자주하면 느려질 수 있으니 필요시 주석)
  // Serial.print("State:"); Serial.print(currentState);
  // Serial.print(" Alt:"); Serial.print(currentAlt);
  // Serial.print(" Max:"); Serial.println(maxAltitude);

  // 4. 딜레이 설정 (중요: 너무 느리면 정점을 놓침)
  // 20ms = 50Hz (초당 50회 체크)
  delay(20); 
}

// ================= 함수 정의 =================

float GetAltitude() {
  if (!bmp.performReading()) return groundAltitude; // 읽기 실패시 이전 값 유지
  return bmp.readAltitude(seaLevelPressure) - groundAltitude;
}

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline Vec3 Cross(Vec3 a, Vec3 b) {
  Vec3 result = {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  };
  return result;
}

static inline Quat QuatNormalize(Quat q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n > 0.0f) {
    float inv = 1.0f / n;
    q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
  } else {
    q = Quat{1.0f, 0.0f, 0.0f, 0.0f};
  }
  return q;
}

void ScanI2C() {
  // (기존 코드 유지)
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning I2C...");
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Found at 0x"); Serial.println(address, HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println("No I2C devices found");
}

void setReports() {
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000); // 10ms (100Hz)
  bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
}

void PollBNO() {
  if (!bnoOK) return;
  sh2_SensorValue_t sv;
  while (bno08x.getSensorEvent(&sv)) {
    if (sv.sensorId == SH2_LINEAR_ACCELERATION) {
      latestAcc.x = sv.un.linearAcceleration.x;
      latestAcc.y = sv.un.linearAcceleration.y;
      latestAcc.z = sv.un.linearAcceleration.z;
    } 
    else if (sv.sensorId == SH2_ROTATION_VECTOR) {
      float w = sv.un.rotationVector.real;
      float x = sv.un.rotationVector.i;
      float y = sv.un.rotationVector.j;
      float z = sv.un.rotationVector.k;
      latestPoseQuat.x = x; latestPoseQuat.y = y; latestPoseQuat.z = z; latestPoseQuat.w = w;

      float norm = w*w + x*x + y*y + z*z;
      if (norm < 1e-9f) return;
      float inv = 1.0f / sqrtf(norm);
      w *= inv; x *= inv; y *= inv; z *= inv;

      float yaw = atan2f(2.0f*(x*y + z*w), (w*w + x*x - y*y - z*z));
      float pitch = asinf(clampf(-2.0f*(x*z - y*w), -1.0f, 1.0f));
      float roll = atan2f(2.0f*(y*z + x*w), (w*w - x*x - y*y + z*z));

      latestPose.yaw = yaw * 180.0f / PI;
      latestPose.pitch = pitch * 180.0f / PI;
      latestPose.roll = roll * 180.0f / PI;
    }
  }
}

void InitSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("[SD] Mount failed.");
    sdOK = false;
    return;
  }
  sdOK = true;
  const String logName = "/log_";
  int digit = 0;
  fileName = logName + String(digit) + ".csv";
  while (SD.exists(fileName)) {
    ++digit;
    fileName = logName + String(digit) + ".csv";
  }
  File f = SD.open(fileName, FILE_WRITE);
  if (f) {
    f.println("ms,alt_m,acc_x,acc_y,acc_z,roll,pitch,yaw,servo_on");
    f.close();
    Serial.print("[SD] Logging to: "); Serial.println(fileName);
  }
}

void LogCSV(float altitude, bool servoOn) {
  if (!sdOK) return;
  File f = SD.open(fileName, FILE_APPEND);
  if (!f) return;

  f.print(millis()); f.print(",");
  f.print(altitude); f.print(",");
  f.print(latestAcc.x); f.print(",");
  f.print(latestAcc.y); f.print(",");
  f.print(latestAcc.z); f.print(",");
  f.print(latestPose.roll); f.print(",");
  f.print(latestPose.pitch); f.print(",");
  f.print(latestPose.yaw); f.print(",");
  f.println(servoOn ? 1 : 0);
  f.close();
}

Vec3 RotateLocalToWorld(Quat q, const Vec3& v) {
  q = QuatNormalize(q);
  Vec3 u{q.x, q.y, q.z};
  Vec3 t = Cross(u, v);
  Vec3 uxt = Cross(u, t);
  Vec3 result = {
    v.x + 2.0f * q.w * t.x + 2.0f * uxt.x,
    v.y + 2.0f * q.w * t.y + 2.0f * uxt.y,
    v.z + 2.0f * q.w * t.z + 2.0f * uxt.z
  };
  return result;
}