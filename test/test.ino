#include <math.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include "Adafruit_BNO08x.h"
#include <ESP32Servo.h>
#include <SD.h>

// 파라미터 설정
const float APOGEE_DROP_THRESHOLD = 2.0f; // 2m 하강 시 사출
const float LAUNCH_DETECT_ALTITUDE = 5.0f; 
const int SERVO_LOCKED_US = 1000;
const int SERVO_OPEN_US = 2000;
const int SERVO_PIN = 25;

enum RocketState { STATE_GROUND, STATE_ASCENT, STATE_DESCENT };

union Vec3 {
  struct { float x; float y; float z; };
  struct { float roll; float pitch; float yaw; };
};

struct Quat { float w; float x; float y; float z; };

static const int SD_MISO = 2;
static const int SD_SCK = 14;
static const int SD_CS = 13;
static const int SD_MOSI = 23;
static const float seaLevelPressure = 1013.25f;

RocketState currentState = STATE_GROUND;
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x(-1);
Servo servo;

float maxAltitude = 0.0;
float groundAltitude = 0.0;
bool bnoOK = false;
bool sdOK = false;
bool isParachuteDeployed = false;

Vec3 latestAcc = {0,0,0};
Vec3 latestPose = {0,0,0};
Quat latestPoseQuat = {0,0,0,0};
String fileName = "";

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

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(21, 22);
  ScanI2C();

  // BMP390
  while (!bmp.begin_I2C()) {
    Serial.println("Conncecting BMP390...");
    delay(1000);
  }
  Serial.println("BMP390 Connected!");
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  delay(500);

  float totalAlt = 0;
  for(int i = 0; i < 10; i++) {
    if(bmp.performReading()) {
      totalAlt += bmp.readAltitude(seaLevelPressure);
    }
    delay(50);
  }
  groundAltitude = totalAlt / 10.0;
  Serial.print("Ground Alt: "); Serial.print(groundAltitude); Serial.println(" m");

  // BNO08x
  Serial.println("Connecting to BNO08x...");
  if (bno08x.begin_I2C(0x4B)) {
    Serial.println("[BNO08x] Connected at 0x4B!");
    setReports();
    bnoOK = true;
  } else if (bno08x.begin_I2C(0x4A)) {
    Serial.println("[BNO08x] Connected at 0x4A!");
    setReports();
    bnoOK = true;
  } else {
    Serial.println("[WARNING] BNO08x Not Found via library.");
    return;
  }

  // Servo
  servo.attach(servoPin);

  // SD
  InitSD();
}

void loop() {
  if (bnoOK && bno08x.wasReset()) setReports();

  float currentAlt = GetAltitude();
  PollBNO();
  Vec3 worldAcc = RotateLocalToWorld(latestPoseQuat, latestAcc);

  switch (currentState) {
    case STATE_GROUND:
      if (currentAlt > LAUNCH_DETECT_ALTITUDE) {
        currentState = STATE_ASCENT;
        maxAltitude = currentAlt;
      }
      break;

    case STATE_ASCENT:
      if (currentAlt > maxAltitude) {
        maxAltitude = currentAlt;
      }

      // 최고 고도 대비 2m 하강 감지 시 사출
      if (currentAlt < (maxAltitude - APOGEE_DROP_THRESHOLD)) {
        servo.writeMicroseconds(SERVO_OPEN_US);
        isParachuteDeployed = true;
        currentState = STATE_DESCENT;
      }
      break;

    case STATE_DESCENT:
      break;
  }

  LogCSV(currentAlt, isParachuteDeployed);

  Serial.print("Altitude: "); Serial.println(altitude);
  Serial.print("Local Acc: ");
  Serial.print(latestAcc.x); Serial.print(", ");
  Serial.print(latestAcc.y); Serial.print(", ");
  Serial.print(latestAcc.z); Serial.println();
  Serial.print("World Acc: ");
  Serial.print(worldAcc.x); Serial.print(", ");
  Serial.print(worldAcc.y); Serial.print(", ");
  Serial.print(worldAcc.z); Serial.println();
  Serial.print("Pose: ");
  Serial.print(latestPose.roll); Serial.print(", ");
  Serial.print(latestPose.pitch); Serial.print(", ");
  Serial.print(latestPose.yaw); Serial.println();

  Serial.println();

  delay(100);
}

float GetAltitude() {
  if (!bmp.performReading()) return groundAltitude;
  return bmp.readAltitude(seaLevelPressure) - groundAltitude;
}

static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline Vec3 Cross(Vec3 a, Vec3 b) {
  Vec3 result = { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
  return result;
}

static inline Quat QuatNormalize(Quat q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n > 0.0f) {
    float inv = 1.0f / n;
    q.w *= inv; q.x *= inv; q.y *= inv; q.z *= inv;
  } else { q = Quat{1.0f, 0.0f, 0.0f, 0.0f}; }
  return q;
}

void ScanI2C() {
  byte error, address;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
  }
}

void setReports() {
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 10000);
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
      latestPoseQuat = {w, x, y, z};

      float norm = w*w + x*x + y*y + z*z;
      if (norm < 1e-9f) return;
      float inv = 1.0f / sqrtf(norm);
      w *= inv; x *= inv; y *= inv; z *= inv;

      latestPose.yaw = atan2f(2.0f*(x*y + z*w), (w*w + x*x - y*y - z*z)) * 180.0f / PI;
      latestPose.pitch = asinf(clampf(-2.0f*(x*z - y*w), -1.0f, 1.0f)) * 180.0f / PI;
      latestPose.roll = atan2f(2.0f*(y*z + x*w), (w*w - x*x - y*y + z*z)) * 180.0f / PI;
    }
  }
}

void InitSD() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  Serial.println("Connecting microSD...");
  if (!SD.begin(SD_CS, SPI)) {
    Serial.println("[SD] Mount failed. Check wiring / CS pin / card format(FAT32).");
    sdOK = false;
    return;
  }
  sdOK = true;
  Serial.println("[SD] OK!");

  const String logName = "/log_";
  int digit = 0;
  const String ext = ".csv";
  fileName = logName + String(digit) + ext;

  while (SD.exists(fileName)) {
    ++digit;
    fileName = logName + String(digit) + ext;
  }

  File f = SD.open(fileName, FILE_WRITE);
  if (f) {
    f.println("ms,alt_m,acc_x,acc_y,acc_z,roll_deg,pitch_deg,yaw_deg,servo_on");
    f.close();
    Serial.println("[SD] log.csv header created.");
  } else {
    Serial.println("[SD] Failed to create log.csv");
  }
}

void LogCSV(float altitude, bool servoOn) {
  File f = SD.open(fileName, FILE_APPEND);
  if (!f) {
    Serial.println("[SD] open failed");
    return;
  }

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

  Serial.println("SD Saved!");
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
