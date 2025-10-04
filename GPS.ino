// ==== ESP32-C3 SuperMini + GY-NEO6MV2 (NEO-6M) ====
// TinyGPS++ "clean": atualiza a cada 20 s, data/hora em BR (UTC-3), prints bonitos.

#include <TinyGPSPlus.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>   // setenv, getenv, unsetenv

TinyGPSPlus gps;

constexpr int ESP_RX1 = 20;           // RX do ESP (vai no TX do GPS)
constexpr int ESP_TX1 = 21;           // TX do ESP (vai no RX do GPS)
constexpr uint32_t GPS_BAUD = 9600;   // NEO-6M padrão
constexpr uint32_t PRINT_INTERVAL_MS = 20000; // 20 s

// ---- helper: timegm compatível (converte struct tm UTC -> epoch) ----
time_t timegm_compat(struct tm *tm_utc) {
  const char* old_tz = getenv("TZ");
  setenv("TZ", "UTC0", 1); tzset();
  time_t t = mktime(tm_utc);          // interpretado como UTC por causa do TZ acima
  if (old_tz) setenv("TZ", old_tz, 1); else unsetenv("TZ");
  tzset();
  return t;
}

// Converte gps.date/time (UTC) para "Brasil (UTC-3)" e formata "dd/mm/aaaa HH:MM:SS (BRT)"
void formatDateTimeBR(char *out, size_t out_sz) {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    snprintf(out, out_sz, "--/--/---- --:--:--");
    return;
  }
  struct tm tm_utc{};
  tm_utc.tm_year = gps.date.year() - 1900;
  tm_utc.tm_mon  = gps.date.month() - 1;
  tm_utc.tm_mday = gps.date.day();
  tm_utc.tm_hour = gps.time.hour();
  tm_utc.tm_min  = gps.time.minute();
  tm_utc.tm_sec  = gps.time.second();

  time_t epoch_utc = timegm_compat(&tm_utc);
  // America/Sao_Paulo (sem horário de verão atualmente): UTC-3
  const long offset = -3 * 3600;
  time_t epoch_brt = epoch_utc + offset;

  struct tm *tm_brt = gmtime(&epoch_brt); // gmtime em cima do epoch ajustado
  snprintf(out, out_sz, "%02d/%02d/%04d %02d:%02d:%02d (BRT)",
           tm_brt->tm_mday, tm_brt->tm_mon + 1, tm_brt->tm_year + 1900,
           tm_brt->tm_hour, tm_brt->tm_min, tm_brt->tm_sec);
}

void prettyPrint() {
  char datetime_brt[40];
  formatDateTimeBR(datetime_brt, sizeof(datetime_brt));

  // Cabeçalho
  Serial.println();
  Serial.println("┌──────────────────────── GPS STATUS ────────────────────────┐");

  // Data/Hora BR
  Serial.print  ("│ Data/Hora (Brasil): ");
  Serial.print  (datetime_brt);
  Serial.println("                          ");

  // Localização
  if (gps.location.isValid()) {
    double lat = gps.location.lat();
    double lon = gps.location.lng();
    Serial.print("│ Latitude:  "); Serial.print(lat, 6);
    Serial.print("   Longitude: "); Serial.print(lon, 6); Serial.println("               ");
  } else {
    Serial.println("│ Latitude/Longitude: (sem fix)                              ");
  }

  // Altitude
  Serial.print("│ Altitude:  ");
  if (gps.altitude.isValid()) { Serial.print(gps.altitude.meters(), 1); Serial.println(" m                           "); }
  else                         { Serial.println("(indisponível)                           "); }

  // Satélites e HDOP
  Serial.print("│ Satélites: ");
  if (gps.satellites.isValid()) Serial.print(gps.satellites.value()); else Serial.print("-");
  Serial.print("   HDOP: ");
  if (gps.hdop.isValid()) Serial.print(gps.hdop.hdop(), 2); else Serial.print("-");
  Serial.println("                                 ");

  // Velocidade e curso
  Serial.print("│ Velocidade: ");
  if (gps.speed.isValid()) { Serial.print(gps.speed.kmph(), 2); Serial.print(" km/h"); }
  else                     { Serial.print("-"); }
  Serial.print("   Curso: ");
  if (gps.course.isValid()) { Serial.print(gps.course.deg(), 1); Serial.print("°"); }
  else                      { Serial.print("-"); }
  Serial.println("                       ");

  Serial.println("└────────────────────────────────────────────────────────────┘");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[boot] ESP32-C3 + TinyGPS++ (limpo) @ UART1 (GPIO20 RX / GPIO21 TX)");
  Serial1.begin(GPS_BAUD, SERIAL_8N1, ESP_RX1, ESP_TX1);
}

void loop() {
  // Consome NMEA continuamente
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // Atualiza a cada 20 s
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    prettyPrint();
  }

  delay(1); // cede tempo e evita WDT
}
