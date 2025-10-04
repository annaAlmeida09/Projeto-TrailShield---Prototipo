// ==== ESP32-C3: GPS + MPU6050 + MAX30102 ====
// Ligações / Pinos:
//   I2C (SDA=8, SCL=9)  -> MPU6050 e MAX30102
//   UART1 (RX=20, TX=21) -> GPS (NMEA 9600 bps)
//
// Bibliotecas necessárias (Arduino IDE -> Biblioteca):
//   - TinyGPSPlus (mikaelpatel/TinyGPSPlus da TinyGPS++)  -> "TinyGPSPlus.h"
//   - Adafruit MPU6050, Adafruit Unified Sensor           -> "Adafruit_MPU6050.h"
//   - SparkFun MAX3010x Pulse and Proximity Sensor Library-> "MAX30105.h", "heartRate.h"
//
// Serial Monitor: 115200
// Impressão completa a cada 20 s (configurável em `intervalo`)

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "MAX30105.h"    // SparkFun MAX3010x (MAX30102 compatível)
#include "heartRate.h"   // detecção de batimento

#include <TinyGPSPlus.h>

// --------- PINOS ---------
constexpr int PIN_SDA = 8;
constexpr int PIN_SCL = 9;
constexpr int GPS_RX  = 20;  // RX do ESP32-C3 ligado ao TX do GPS
constexpr int GPS_TX  = 21;  // TX do ESP32-C3 ligado ao RX do GPS

// --------- OBJETOS ---------
Adafruit_MPU6050 mpu;
MAX30105         particleSensor;
TinyGPSPlus      gps;

// --------- FLAGS ---------
bool mpu_ok = false;
bool max_ok = false;

// --------- CONTROLE DE IMPRESSÃO ---------
unsigned long ultimaImpressao = 0;
const unsigned long intervalo = 20000UL; // 20 s

// --------- BATIMENTOS (atualizados continuamente) ---------
static unsigned long ultimoBeatMs = 0;
static float bpm_atual = 0.0f;

// --------- FUSO (BR) ---------
const int TZ_OFFSET_HOURS = -3; // BRT = UTC-3

// --- Utilitários de Data/Hora ---
// Determina se é ano bissexto
bool isLeap(int y) {
  return ( (y % 4 == 0) && ( (y % 100 != 0) || (y % 400 == 0) ) );
}

// Dias no mês
int daysInMonth(int m, int y) {
  static const int dpm[] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
  if (m == 2) return dpm[1] + (isLeap(y) ? 1 : 0);
  return dpm[m-1];
}

// Ajusta hora com offset (pode ser negativo/positivo) e corrige data
void applyHourOffset(int &dia, int &mes, int &ano, int &hora, int offsetHours) {
  int h = hora + offsetHours;
  while (h < 0) {
    h += 24;
    // voltar um dia
    dia -= 1;
    if (dia < 1) {
      mes -= 1;
      if (mes < 1) {
        mes = 12;
        ano -= 1;
      }
      dia = daysInMonth(mes, ano);
    }
  }
  while (h >= 24) {
    h -= 24;
    // avançar um dia
    dia += 1;
    int dim = daysInMonth(mes, ano);
    if (dia > dim) {
      dia = 1;
      mes += 1;
      if (mes > 12) {
        mes = 1;
        ano += 1;
      }
    }
  }
  hora = h;
}

// ======= SETUP =======
void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("   PrototipoTrailShield (ESP32-C3)   ");
  Serial.println("   GPS + MPU6050 + MAX30102          ");
  Serial.println("=====================================\n");

  // I2C
  Wire.begin(PIN_SDA, PIN_SCL);

  // MPU6050
  if (mpu.begin()) {
    mpu_ok = true;
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[OK] MPU6050 inicializado.");
  } else {
    Serial.println("[!] MPU6050 não encontrado (cheque ligações).");
  }

  // MAX30102
  if (particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    max_ok = true;
    particleSensor.setup(); // configurações padrão da lib (void, sem retorno)
    // Ajustes mínimos de amplitude (pode calibrar depois)
    particleSensor.setPulseAmplitudeRed(0x2A);    // LED vermelho ativo
    particleSensor.setPulseAmplitudeIR(0x2A);     // LED IR ativo
    particleSensor.setPulseAmplitudeGreen(0x00);  // Verde off (economia)
    Serial.println("[OK] MAX30102 inicializado.");
  } else {
    Serial.println("[!] MAX30102 não encontrado (cheque I2C e alimentação).");
  }

  // GPS (UART1)
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("[OK] GPS UART iniciado @9600.\n");
}

// ======= LOOP =======
void loop() {
  // Alimenta o parser do GPS continuamente
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Leitura contínua do MAX30102 para manter BPM atualizado
  if (max_ok) {
    long irValue = particleSensor.getIR(); // valor IR (0 se nada detectado)
    if (irValue > 5000) { // simples filtro para evitar ruído muito baixo
      if (checkForBeat(irValue)) {
        unsigned long agora = millis();
        if (ultimoBeatMs != 0) {
          float delta = (float)(agora - ultimoBeatMs);
          if (delta > 0.0f) {
            bpm_atual = 60000.0f / delta;  // BPM = 60.000 ms / intervalo entre batidas
          }
        }
        ultimoBeatMs = agora;
      }
    }
  }

  // Impressão formatada a cada 'intervalo'
  unsigned long t = millis();
  if (t - ultimaImpressao >= intervalo) {
    ultimaImpressao = t;
    imprimirBloco();
  }
}

// ======= IMPRESSÃO FORMATADA =======
void imprimirBloco() {
  Serial.println("───────────── Monitoramento (20 s) ─────────────");

  // ---- Data/Hora ----
  if (gps.date.isValid() && gps.time.isValid()) {
    int d = gps.date.day();
    int m = gps.date.month();
    int y = gps.date.year();
    int hh = gps.time.hour();
    int mm = gps.time.minute();
    int ss = gps.time.second();

    // Mostra UTC
    char bufUTC[22];
    snprintf(bufUTC, sizeof(bufUTC), "%02d/%02d/%04d %02d:%02d:%02d",
             d, m, y, hh, mm, ss);

    // Converte para BRT (UTC-3)
    int dB = d, mB = m, yB = y, hB = hh;
    applyHourOffset(dB, mB, yB, hB, TZ_OFFSET_HOURS);
    char bufBRT[22];
    snprintf(bufBRT, sizeof(bufBRT), "%02d/%02d/%04d %02d:%02d:%02d",
             dB, mB, yB, hB, mm, ss);

    Serial.printf("Data/Hora (UTC): %s\n", bufUTC);
    Serial.printf("Data/Hora (BRT): %s\n", bufBRT);
  } else {
    Serial.println("Data/Hora: --/--/---- --:--:-- (aguardando fixo do GPS)");
  }

  // ---- GPS ----
  if (gps.location.isValid()) {
    Serial.printf("Localização:  Lat=%.6f  Lon=%.6f\n",
                  gps.location.lat(), gps.location.lng());
  } else {
    Serial.println("Localização:  (inválida / sem fixo)");
  }

  if (gps.satellites.isValid()) {
    Serial.printf("Satélites:    %u\n", gps.satellites.value());
  } else {
    Serial.println("Satélites:    --");
  }

  if (gps.hdop.isValid()) {
    Serial.printf("HDOP:         %.2f\n", gps.hdop.hdop());
  } else {
    Serial.println("HDOP:         --");
  }

  if (gps.altitude.isValid()) {
    Serial.printf("Altitude:     %.2f m\n", gps.altitude.meters());
  } else {
    Serial.println("Altitude:     --");
  }

  if (gps.speed.isValid()) {
    Serial.printf("Velocidade:   %.2f km/h\n", gps.speed.kmph());
  } else {
    Serial.println("Velocidade:   --");
  }

  if (gps.course.isValid()) {
    Serial.printf("Curso:        %.2f°\n", gps.course.deg());
  } else {
    Serial.println("Curso:        --");
  }

  Serial.println();

  // ---- MPU6050 ----
  if (mpu_ok) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.printf("Acelerômetro [m/s²]:  X=%6.2f  Y=%6.2f  Z=%6.2f\n",
                  a.acceleration.x, a.acceleration.y, a.acceleration.z);
    Serial.printf("Giroscópio   [°/s]:   X=%6.2f  Y=%6.2f  Z=%6.2f\n",
                  g.gyro.x, g.gyro.y, g.gyro.z);
    Serial.printf("Temperatura   [°C]:   %6.2f\n", temp.temperature);
  } else {
    Serial.println("MPU6050:      não inicializado.");
  }

  Serial.println();

  // ---- MAX30102 (BPM) ----
  if (max_ok) {
    Serial.printf("Batimentos:   %.1f BPM\n", bpm_atual);
    
  } else {
    Serial.println("Batimentos:   sensor não inicializado.");
  }

  Serial.println("────────────────────────────────────────────────\n");
}
