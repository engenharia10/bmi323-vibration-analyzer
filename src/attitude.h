#pragma once
#include <Arduino.h>
#include <bf_filter.h>

// ---------------------------------------------------------------------------
// Estimativa de atitude, com foco em NAO PERDER A REFERENCIA DA GRAVIDADE
// mesmo com o sensor chacoalhando.
//
// O problema: sob vibracao o acelerometro mede gravidade + a vibracao inteira.
// Se voce joga isso direto num atan2, o angulo pula junto com a vibracao; se
// voce simplesmente ignora o acelerometro quando |a| foge de 1 g, o giroscopio
// fica sem correcao e a deriva come o angulo.
//
// A solucao aqui tem tres partes:
//
//  1. PASSA-BAIXA PESADO no acelerometro (PT2, ~3 Hz por padrao) so para
//     estimar a gravidade. Vibracao mecanica vive acima de algumas dezenas de
//     Hz; gravidade e DC. Separar as duas coisas no dominio da frequencia
//     resolve a maior parte do problema antes de qualquer logica.
//
//  2. CONFIANCA GRADUAL em vez de liga/desliga: quanto mais |a| filtrado se
//     afasta de 1 g, menos peso o acelerometro tem. Sem degraus, sem
//     chaveamento nervoso.
//
//  3. CORRECAO PI (estilo Mahony): o termo integral estima o offset do
//     giroscopio continuamente. E o que segura o angulo durante os trechos em
//     que o acelerometro nao merece confianca - o giro ja esta compensado de
//     antes, entao ele sozinho aguenta bem mais tempo.
//
// roll e pitch, portanto, tem referencia absoluta e sobrevivem a vibracao.
// yaw continua sendo integracao pura de gz: sem magnetometro nao ha referencia,
// ele DERIVA. Serve para rotacao relativa, nao para rumo.
//
// A origem do sinal e escolhida por Config::source (ver enum abaixo). O padrao
// combina giroscopio filtrado com acelerometro cru: filtrar o giro antes de
// integrar limpa o angulo, e o acelerometro precisa manter a gravidade, que o
// DC-block da cadeia de filtros removeria.
// ---------------------------------------------------------------------------

namespace att {

// De onde vem o sinal que alimenta o estimador.
//  SRC_RAW  - tudo cru, como o sensor entrega
//  SRC_GYRO - giroscopio JA FILTRADO pela cadeia, acelerometro cru. E o
//             padrao: filtrar o giro antes de integrar tira o ruido de
//             vibracao do angulo, e o acelerometro precisa da gravidade,
//             que o DC-block da cadeia removeria.
//  SRC_BOTH - tudo filtrado. So faz sentido com o DC-block DESLIGADO;
//             com ele ligado nao sobra gravidade para referenciar nada,
//             entao o firmware volta ao acelerometro cru sozinho.
enum Source : uint8_t { SRC_RAW = 0, SRC_GYRO = 1, SRC_BOTH = 2 };

struct Config {
  uint8_t source  = SRC_GYRO;
  float gravLpfHz = 3.0f;    // corte do passa-baixa que isola a gravidade
  float kp        = 1.5f;    // ganho proporcional (1/kp ~ constante de tempo)
  float ki        = 0.05f;   // ganho integral: aprende o offset do giroscopio
  float tolG      = 0.35f;   // desvio de |a| em que a confianca chega a zero
};

struct State {
  float roll = 0, pitch = 0, yaw = 0;   // graus
  float gx = 0, gy = 0, gz = 0;         // graus/s, ja sem o offset
  float accMag = 0;                     // |a| instantaneo, em g
  float accMagF = 0;                    // |a| depois do passa-baixa (deve dar ~1)
  float trust = 0;                      // 0..1: peso dado ao acelerometro agora
  float grav[3] = {0, 0, 1};            // vetor gravidade unitario, no corpo
  float bias[3] = {0, 0, 0};            // offset estimado do giroscopio
  float vibG = 0;                       // amplitude da vibracao removida, em g
  bool  calibrating = false;
  bool  valid = false;                  // ha giroscopio alimentando
  bool  accFiltered = false;            // o acelerometro usado veio filtrado
  bool  gyrFiltered = false;
};

class Estimator {
 public:
  void begin(float sampleRateHz);
  void configure(const Config &cfg);
  // acc em g, gyr em graus/s. Passe gyr = nullptr se nao houver giroscopio:
  // roll e pitch continuam saindo do acelerometro filtrado, yaw fica zerado.
  void update(const float acc[3], const float gyr[3]);
  // marca de onde vieram as amostras desta chamada, so para relatorio
  void setSourceInfo(bool accFilt, bool gyrFilt) { accFilt_ = accFilt; gyrFilt_ = gyrFilt; }

  void startCalibration();   // remede o offset do giroscopio (fique parado)
  void zeroYaw() { yaw_ = 0.0f; }

  const Config &config() const { return cfg_; }
  State state() const;

 private:
  Config cfg_;
  float fs_ = 1600.0f, dt_ = 1.0f / 1600.0f;
  float roll_ = 0, pitch_ = 0, yaw_ = 0;
  float bias_[3] = {0, 0, 0};
  double biasAcc_[3] = {0, 0, 0};
  uint32_t calCount_ = 0, calTarget_ = 0;
  bool  calibrating_ = false;
  bool  haveGyro_ = false;
  bool  primed_ = false;
  bf::Pt2 gravLpf_[3];
  float lastRate_[3] = {0, 0, 0};
  float accMag_ = 0, accMagF_ = 0, trust_ = 0, vib_ = 0;
  float grav_[3] = {0, 0, 1};
  int   decim_ = 0;
  float rollAcc_ = 0, pitchAcc_ = 0;
  bool  accFilt_ = false, gyrFilt_ = false;

  void rebuildLpf();
};

extern Estimator est;

}  // namespace att
