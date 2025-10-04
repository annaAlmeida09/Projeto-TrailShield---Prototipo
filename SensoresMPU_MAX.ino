// ==== ESP32-C3: MAX30102 (IR) + MPU6050 (IMU) ====
// Ligações (3V3 e GND compartilhados):
//   I2C -> SDA = GPIO 8, SCL = GPIO 9

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "MAX30105.h"   // SparkFun MAX3010x (vale para MAX30102)

// --------- PINOS ---------
constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;

// --------- OBJETOS ---------
Adafruit_MPU6050 mpu;
MAX30105 particleSensor;
bool mpu_ok = false, max_ok = false;

// --------- CONFIG LED ---------
uint8_t ledBrightness = 60;  // ajuste inicial de brilho

// --------- TIMER ---------
unsigned long t_lastPrint = 0;
const unsigned long PRINT_INTERVAL = 10000; // 10s

// --------- FUNÇÕES ---------
void initI2C() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
}

void initMPU() {
  if (mpu.begin(0x68, &Wire)) {
    mpu_ok = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[mpu] ok");
  } else {
    Serial.println("[mpu] falhou");
  }
}

void initMAX() {
  if (particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    max_ok = true;
    // ledMode=2 (Red+IR), mas você pode desligar RED se quiser menos calor
    particleSensor.setup(ledBrightness, 8, 2, 100, 411, 16384);
    // Se quiser reduzir aquecimento, troque 0x1F por 0 para RED:
    particleSensor.setPulseAmplitudeRed(0x1F);       // ou 0 para desligar
    particleSensor.setPulseAmplitudeIR(0x1F);
    particleSensor.setPulseAmplitudeGreen(0);
    Serial.println("[max] ok");
  } else {
    Serial.println("[max] falhou");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n[boot] ESP32-C3: MAX30102 (IR) + MPU6050");

  initI2C();
  initMAX();
  initMPU();
}

void loop() {
  // --- PRINT A CADA 10 SEGUNDOS ---
  if (millis() - t_lastPrint >= PRINT_INTERVAL) {
    t_lastPrint = millis();

    // MAX30102 (apenas IR)
    if (max_ok) {
      long ir = particleSensor.getIR();
      Serial.printf("[MAX] IR=%ld\n", ir);
    } else {
      Serial.println("[MAX] nao inicializado");
    }

    // MPU6050
    if (mpu_ok) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      Serial.printf("[MPU] Acc(x=%.2f y=%.2f z=%.2f) m/s² | Gyro(x=%.2f y=%.2f z=%.2f) °/s | Temp=%.1f °C\n",
                    a.acceleration.x, a.acceleration.y, a.acceleration.z,
                    g.gyro.x, g.gyro.y, g.gyro.z, t.temperature);
    } else {
      Serial.println("[MPU] nao inicializado");
    }

    Serial.println("-------------------------------");
  }

  // loop leve
  delay(5);
}
