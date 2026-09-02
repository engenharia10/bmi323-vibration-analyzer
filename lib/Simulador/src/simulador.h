#pragma once
#include <Arduino.h>
#include <bf_filter.h>

// ---------------------------------------------------------------------------
// Gerador de vibracao sintetica.
//
// Substitui o FIFO do sensor por um sinal montado a partir de N fontes
// rotativas, cada uma com fundamental, harmonicas, deriva lenta de rotacao e
// jitter. Depois o conjunto passa por uma ressonancia estrutural (a "placa
// presa na peca") e ganha um piso de ruido.
//
// Serve para dois casos:
//  * projetar e validar os filtros sem ter a maquina ligada na bancada;
//  * exercitar a cadeia de filtros e o notch dinamico de verdade - o sinal
//    entra no mesmo caminho das amostras do BMI323, nao e um desenho na tela.
//
// A deriva (drift) e o que faz o notch dinamico valer a pena: com ela a
// frequencia passeia como um motor acelerando, e da para ver os notches
// perseguindo os picos.
// ---------------------------------------------------------------------------

namespace sim {

constexpr int SOURCES      = 6;
constexpr int HARMONICS_MAX = 4;

enum AxisSel : uint8_t { AX_X = 0, AX_Y = 1, AX_Z = 2, AX_ALL = 3 };

struct Source {
  bool    enabled      = false;
  float   freqHz       = 100.0f;   // fundamental
  float   ampMg        = 30.0f;    // amplitude da fundamental, em mg
  uint8_t harmonics    = 1;        // 1..HARMONICS_MAX
  uint8_t harmDecayPct = 50;       // amplitude da harmonica n = amp * decay^(n-1)
  float   driftHz      = 0.0f;     // excursao da deriva (+/-)
  float   driftPeriodS = 10.0f;    // periodo da deriva
  float   jitterHz     = 0.0f;     // erro aleatorio de rotacao (passeio lento)
  uint8_t axis         = AX_ALL;
};

struct Config {
  bool  enabled     = false;
  float noiseMg     = 2.0f;        // piso de ruido branco
  bool  gravity     = true;        // 1 g constante em Z (da trabalho ao DC-block)
  bool  resonanceOn = false;       // ressonancia da estrutura/fixacao
  float resonanceHz = 300.0f;
  float resonanceQ  = 6.0f;
  float resonanceGainDb = 10.0f;
  Source src[SOURCES];

  // --- movimento do corpo, para a janela de atitude ---
  bool  attOn        = false;
  float rollAmpDeg   = 10.0f;   // balanco em roll
  float pitchAmpDeg  = 7.0f;    // balanco em pitch
  float attPeriodS   = 6.0f;    // periodo do balanco
  float yawRateDps   = 15.0f;   // giro continuo em yaw
};

class Generator {
 public:
  void init(const Config &cfg, float sampleRateHz);
  // Gera uma amostra: aceleracao em g e velocidade angular em graus/s.
  // Passe gyr = nullptr se nao precisar do giroscopio.
  void next(float acc[3], float gyr[3] = nullptr);

  const Config &config() const { return cfg_; }
  float sampleRate() const { return fs_; }
  // Frequencia instantanea de uma fonte (com deriva aplicada), para a UI.
  float currentFreq(int i) const { return curFreq_[i]; }

 private:
  Config cfg_;
  float  fs_ = 1600.0f;
  float  dt_ = 1.0f / 1600.0f;
  double t_  = 0.0;

  float  phase_[SOURCES][HARMONICS_MAX] = {};
  float  jitter_[SOURCES] = {};
  float  curFreq_[SOURCES] = {};
  float  resGain_ = 0.0f;
  float  attRoll_ = 0, attPitch_ = 0, attYaw_ = 0;
  bf::Biquad res_[3];
  uint32_t rng_ = 0x1234567u;

  float rand01();
  float noise();
};

extern Generator gen;

}  // namespace sim
