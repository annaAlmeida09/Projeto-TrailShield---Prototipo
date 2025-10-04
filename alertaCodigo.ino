/*  TrailShield — Motor de Alertas (sem BD)
    Autor: você + ChatGPT
    Alvos: ESP32 / ESP32-C3 / Arduino compatível
    Só lógica: receba medidas -> retorna strings de alerta.
*/

#include <Arduino.h>
#include <math.h>
#include <vector>

// ===================== PARÂMETROS AJUSTÁVEIS =====================
struct AlertConfig {
  // Queda
  float impact_g              = 4.5f;   // g
  uint32_t immob_after_impact = 15000;  // ms parado após impacto
  float immob_speed_ms        = 0.5f;   // m/s
  float immob_stdA_g          = 0.20f;  // g (variabilidade baixa = parado)

  // Imobilidade prolongada
  uint32_t immob_timeout_ms   = 20UL * 60UL * 1000UL; // 20 min
  float immob_idle_speed_ms   = 0.20f; // m/s
  float immob_idle_stdA_g     = 0.10f; // g

  // Cardio
  int bpm_tachy               = 150;
  int bpm_brady               = 40;
  uint16_t cardio_confirm_n   = 3;     // leituras consecutivas
  uint32_t cardio_window_ms   = 30000; // p/ médias (apenas lógico)

  // SpO2
  int spo2_low                = 90;    // %
  int spo2_crit               = 85;    // %
  uint16_t spo2_confirm_n     = 3;

  // Velocidade anômala
  float speed_max_ms          = 8.0f;  // ~28.8 km/h

  // Supressão de duplicatas
  uint32_t suppress_window_ms = 10UL * 60UL * 1000UL; // 10 min

  // Desvio de rota (opcional — use feedRouteDeviation())
  float route_dev_radius_m    = 200.0f;
  uint32_t route_dev_time_ms  = 5UL * 60UL * 1000UL;
};

// ===================== MEDIDAS =====================
struct Measurement {
  uint32_t id_medida;    // opcional: id da amostra
  double   lat, lon;     // graus
  float    bpm;          // batimentos
  int      spo2;         // %
  float    speed_ms;     // m/s (0 se não disponível)
  float    ax_g, ay_g, az_g; // aceleração em g
  // gyro não é necessário para as regras básicas
  uint32_t t_ms;         // timestamp (millis)
};

// ===================== UTILITÁRIOS =====================
static inline float accelMagG(float ax, float ay, float az) {
  return sqrtf(ax*ax + ay*ay + az*az);
}

class RollingVar {
public:
  RollingVar(size_t capacity) : cap(capacity) { buf.reserve(capacity); }
  void add(float v) {
    if (buf.size() < cap) { buf.push_back(v); sum += v; sum2 += v*v; }
    else {
      float old = buf[idx];
      sum -= old; sum2 -= old*old;
      buf[idx] = v; sum += v; sum2 += v*v;
      idx = (idx + 1) % cap;
      filled = true;
      return;
    }
    if (buf.size() == cap) filled = true;
  }
  bool ready() const { return filled || buf.size() >= 3; }
  float mean() const { return buf.empty() ? 0.f : sum / buf.size(); }
  float stddev() const {
    if (buf.size() < 2) return 0.f;
    float m = mean();
    float v = (sum2 / buf.size()) - m*m;
    return v > 0.f ? sqrtf(v) : 0.f;
  }
  void clear() { buf.clear(); sum=sum2=0; idx=0; filled=false; }
private:
  std::vector<float> buf;
  size_t cap;
  size_t idx=0;
  bool filled=false;
  float sum=0, sum2=0;
};

// ===================== MOTOR DE ALERTAS =====================
class AlertEngine {
public:
  AlertEngine(const AlertConfig& cfg, uint16_t hz_accel = 20)
  : C(cfg),
    accelVar(max<size_t>(hz_accel * 2, 20)) // ~2s janela p/ stddev
  {}

  // Se quiser informar desvio de rota periodicamente:
  void feedRouteDeviation(float distance_m, uint32_t min_fora_rota) {
    last_route_dev_m = distance_m;
    last_route_dev_minutes = min_fora_rota;
    route_dev_last_update = millis();
  }

  // Retorna "" se nada; caso contrário, texto pronto para log/notificação
  String evaluate(const Measurement& m) {
    // Atualiza estatística de aceleração
    float A = accelMagG(m.ax_g, m.ay_g, m.az_g);
    accelVar.add(A);

    // ===== 1) Queda: impacto + imobilidade =====
    if (A > C.impact_g) {
      last_impact_ms = m.t_ms;
      impact_detected = true;
    }
    String alert = checkFallAndImmobility(m, A);
    if (alert.length()) return suppressIfNeeded("FALL", alert, m.t_ms);

    // ===== 2) Imobilidade prolongada =====
    alert = checkLongImmobility(m);
    if (alert.length()) return suppressIfNeeded("IMMOB_LONG", alert, m.t_ms);

    // ===== 3) Cardio: taquicardia / bradicardia =====
    alert = checkCardio(m);
    if (alert.length()) return alert; // cardio tem mensagens distintas, suprimidas por tipo interno

    // ===== 4) SpO2 baixo =====
    alert = checkSpO2(m);
    if (alert.length()) return alert;

    // ===== 5) Velocidade anômala =====
    alert = checkSpeed(m);
    if (alert.length()) return suppressIfNeeded("SPEED", alert, m.t_ms);

    // ===== 6) Desvio de rota (opcional) =====
    alert = checkRouteDeviation(m);
    if (alert.length()) return suppressIfNeeded("ROUTE_DEV", alert, m.t_ms);

    // sem alertas
    updateMovementBaselines(m); // mantém timers
    return "";
  }

private:
  AlertConfig C;

  // Estado interno
  RollingVar accelVar;
  uint32_t last_move_ms = 0;
  uint32_t last_impact_ms = 0;
  bool impact_detected = false;

  // contadores p/ confirmações
  uint16_t tachy_count = 0, brady_count = 0, spo2_low_count = 0, spo2_crit_count = 0;

  // supressão simples por tipo
  struct Stamp { String key; uint32_t t; };
  std::vector<Stamp> last_alerts;

  // rota opcional
  float last_route_dev_m = 0.0f;
  uint32_t last_route_dev_minutes = 0;
  uint32_t route_dev_last_update = 0;

  // ---------- Helpers ----------
  String suppressIfNeeded(const String& key, const String& msg, uint32_t now_ms) {
    // evita duplicar o mesmo tipo por janela
    for (auto &s : last_alerts) {
      if (s.key == key && (now_ms - s.t) < C.suppress_window_ms) {
        return ""; // suprimido
      }
    }
    // registra
    last_alerts.push_back({key, now_ms});
    // mantém vetor curto
    if (last_alerts.size() > 16) last_alerts.erase(last_alerts.begin());
    return msg;
  }

  void updateMovementBaselines(const Measurement& m) {
    bool moving = (m.speed_ms > C.immob_idle_speed_ms) || (accelVar.stddev() > C.immob_idle_stdA_g);
    if (moving) last_move_ms = m.t_ms;
    // zera impacto antigo
    if (impact_detected && (m.t_ms - last_impact_ms) > 600000UL) { // 10 min
      impact_detected = false;
    }
  }

  // ---------- Regras ----------
  String checkFallAndImmobility(const Measurement& m, float A) {
    // confirmar imobilidade após impacto
    if (impact_detected) {
      bool lowSpeed = (m.speed_ms <= C.immob_speed_ms);
      bool lowVarA  = accelVar.ready() && (accelVar.stddev() <= C.immob_stdA_g);
      if (lowSpeed && lowVarA) {
        if ((m.t_ms - last_impact_ms) >= C.immob_after_impact) {
          impact_detected = false; // consome evento
          char buf[220];
          snprintf(buf, sizeof(buf),
            "CRÍTICO — Queda detectada. Impacto A=%.1fg; imobilidade %lus; lat:%.6f lon:%.6f; id_medida=%lu.",
            A, (unsigned long)((m.t_ms - last_impact_ms)/1000UL), m.lat, m.lon, (unsigned long)m.id_medida);
          return String(buf);
        }
      } else {
        // ainda se mexendo → recomeça janela
        last_impact_ms = m.t_ms;
      }
    }
    return "";
  }

  String checkLongImmobility(const Measurement& m) {
    // atualiza referência de movimento
    bool moving = (m.speed_ms > C.immob_idle_speed_ms) || (accelVar.ready() && accelVar.stddev() > C.immob_idle_stdA_g);
    if (moving) last_move_ms = m.t_ms;

    if (last_move_ms == 0) last_move_ms = m.t_ms;

    uint32_t parado_ms = m.t_ms - last_move_ms;
    if (parado_ms >= C.immob_timeout_ms) {
      char buf[200];
      snprintf(buf, sizeof(buf),
        "ALTO — Imobilidade: %lu min sem movimento. lat:%.6f lon:%.6f; id_medida=%lu.",
        (unsigned long)(parado_ms/60000UL), m.lat, m.lon, (unsigned long)m.id_medida);
      // após alertar, empurra baseline p/ evitar spam
      last_move_ms = m.t_ms;
      return String(buf);
    }
    return "";
  }

  String checkCardio(const Measurement& m) {
    String out = "";
    // taquicardia
    if (m.bpm > C.bpm_tachy) {
      tachy_count++;
      if (tachy_count >= C.cardio_confirm_n) {
        char buf[180];
        const char* sev = (m.bpm >= 180) ? "CRÍTICO" : "ALTO";
        snprintf(buf, sizeof(buf),
          "%s — Taquicardia: %d bpm (%u leituras). lat:%.6f lon:%.6f; id_medida=%lu.",
          sev, (int)m.bpm, tachy_count, m.lat, m.lon, (unsigned long)m.id_medida);
        out = String(buf);
        tachy_count = 0; // reset pós alerta
      }
    } else {
      tachy_count = 0;
    }

    if (out.length()) return out;

    // bradicardia
    if (m.bpm > 0 && m.bpm < C.bpm_brady) {
      brady_count++;
      if (brady_count >= C.cardio_confirm_n) {
        char buf[180];
        snprintf(buf, sizeof(buf),
          "ALTO — Bradicardia: %d bpm (%u leituras). lat:%.6f lon:%.6f; id_medida=%lu.",
          (int)m.bpm, brady_count, m.lat, m.lon, (unsigned long)m.id_medida);
        out = String(buf);
        brady_count = 0;
      }
    } else {
      brady_count = 0;
    }

    return out;
  }

  String checkSpO2(const Measurement& m) {
    if (m.spo2 <= 0) { spo2_low_count = spo2_crit_count = 0; return ""; }

    if (m.spo2 < C.spo2_crit) {
      spo2_crit_count++;
      if (spo2_crit_count >= C.spo2_confirm_n) {
        char buf[180];
        snprintf(buf, sizeof(buf),
          "CRÍTICO — SpO2 muito baixa: %d%% (%u leituras). lat:%.6f lon:%.6f; id_medida=%lu.",
          (int)m.spo2, spo2_crit_count, m.lat, m.lon, (unsigned long)m.id_medida);
        spo2_crit_count = spo2_low_count = 0;
        return String(buf);
      }
    } else if (m.spo2 < C.spo2_low) {
      spo2_low_count++;
      if (spo2_low_count >= C.spo2_confirm_n) {
        char buf[180];
        snprintf(buf, sizeof(buf),
          "ALTO — SpO2 baixa: %d%% (%u leituras). lat:%.6f lon:%.6f; id_medida=%lu.",
          (int)m.spo2, spo2_low_count, m.lat, m.lon, (unsigned long)m.id_medida);
        spo2_low_count = 0;
        return String(buf);
      }
    } else {
      spo2_low_count = spo2_crit_count = 0;
    }
    return "";
  }

  String checkSpeed(const Measurement& m) {
    if (m.speed_ms > C.speed_max_ms) {
      char buf[160];
      snprintf(buf, sizeof(buf),
        "MÉDIO — Velocidade incomum: %.1f km/h. lat:%.6f lon:%.6f; id_medida=%lu.",
        m.speed_ms * 3.6f, m.lat, m.lon, (unsigned long)m.id_medida);
      return String(buf);
    }
    return "";
  }

  String checkRouteDeviation(const Measurement& m) {
    // Só avalia se recebeu atualização recente
    if (route_dev_last_update == 0) return "";
    if ((millis() - route_dev_last_update) > 5000UL) {
      // dados de rota ficaram velhos, não alertar
      return "";
    }
    if (last_route_dev_m > C.route_dev_radius_m &&
        last_route_dev_minutes * 60000UL >= C.route_dev_time_ms) {
      char buf[200];
      snprintf(buf, sizeof(buf),
        "MÉDIO — Desvio de rota: %.0f m fora do trajeto por %u min. lat:%.6f lon:%.6f; id_medida=%lu.",
        last_route_dev_m, (unsigned)last_route_dev_minutes, m.lat, m.lon, (unsigned long)m.id_medida);
      // zera para não repetir
      last_route_dev_minutes = 0;
      return String(buf);
    }
    return "";
  }
};

// ===================== EXEMPLO DE USO =====================
// Simula recebimento de amostras e imprime alertas no Serial.
AlertConfig CFG;          // personalize se quiser
AlertEngine  ALERT(CFG);  // motor

uint32_t t0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("[TrailShield] Motor de alertas iniciado (sem BD)."));
  t0 = millis();
}

void loop() {
  // >>> Aqui você deve alimentar 'm' com dados REAIS dos seus sensores <<<
  // Abaixo: exemplo didático. Troque pelos seus valores.
  static uint32_t id_seq = 1;
  Measurement m;
  m.id_medida = id_seq++;
  m.t_ms      = millis();
  m.lat = -22.257518; m.lon = -45.697088;

  // Exemplos alternando situações (apenas demonstração)
  float phase = (millis() - t0) / 1000.0f;

  // aceleração "normal" ~1g com ruído
  m.ax_g = 0.02f * sinf(phase) + 0.00f;
  m.ay_g = 0.02f * cosf(phase) + 0.00f;
  m.az_g = 1.00f + 0.02f * sinf(0.4f*phase);

  // velocidade quase parada
  m.speed_ms = (fmod(phase, 60.0f) < 5.0f) ? 0.0f : 0.3f;

  // cardio/SpO2 baseline
  m.bpm  = 98;
  m.spo2 = 96;

  // Simular eventos:
  // 1) Queda forte aos 15s
  if (phase > 15.0f && phase < 15.5f) {
    m.ax_g = 4.8f; m.ay_g = 0.2f; m.az_g = 0.1f; // impacto
    m.speed_ms = 0.0f; // depois ficará parado
  }
  // 2) Taquicardia aos 40–45s
  if (phase > 40.0f && phase < 45.0f) m.bpm = 165;
  // 3) SpO2 baixa aos 65–70s
  if (phase > 65.0f && phase < 70.0f) m.spo2 = 86;
  // 4) Velocidade anômala aos 90–92s
  if (phase > 90.0f && phase < 92.0f) m.speed_ms = 10.0f;

  // (Opcional) informar desvio de rota quando tiver esse cálculo externo
  // ALERT.feedRouteDeviation(distancia_em_metros, minutos_fora);

  // Avaliar
  String alert = ALERT.evaluate(m);
  if (alert.length()) {
    Serial.println(alert);
  }

  delay(200); // ~5 Hz de avaliação
}
