#pragma once
#include <Arduino.h>
#include "bf_filter.h"
#include "bf_sdft.h"

// ---------------------------------------------------------------------------
// Cadeia de filtros do Betaflight, portada para uso com um acelerometro.
//
// Ordem de aplicacao, identica a de src/main/sensors/gyro_filter_impl.c:
//
//   [DC-block] -> LPF2 -> RPM -> notch1 -> notch2 -> LPF1 -> notch dinamico
//
// Blocos e adaptacoes:
//
//  * DC-block   -> NAO existe no Betaflight. La o sinal e do giroscopio, que
//                  nao tem componente continua. Aqui filtramos aceleracao: sem
//                  ele a gravidade (1 g) domina tudo.
//  * RPM filter -> no Betaflight a frequencia vem da telemetria bidirecional
//                  DShot dos ESCs. Aqui a fonte e escolhida pelo usuario:
//                  manual (Hz/RPM) ou automatica (pico dominante do espectro).
//                  Parametros e logica de peso/fade sao os do original.
//  * LPF1 dyn   -> no Betaflight o corte varia com o acelerador. Aqui a mesma
//                  curva e dirigida por um "nivel de carga" 0..1 informado
//                  pela aplicacao (setLoad).
//
// Uso, uma amostra por vez:
//   for (int ax = 0; ax < 3; ax++) out[ax] = chain.apply(ax, in[ax]);
//   chain.update();   // maquina de estados do notch dinamico + RPM
// ---------------------------------------------------------------------------

namespace filtros {

constexpr int AXES                 = 3;
constexpr int DYN_NOTCH_COUNT_MAX  = 3;
constexpr int RPM_HARMONICS_MAX    = 3;

// Betaflight desativa o notch dinamico abaixo de 2 kHz de loop porque a maquina
// de estados precisa de 12 iteracoes para atualizar os 3 eixos. Aqui a dinamica
// e muito mais lenta que a de um drone, entao o piso e menor.
constexpr float DYN_NOTCH_MIN_SAMPLE_RATE = 500.0f;

enum RpmSource : uint8_t { RPM_OFF = 0, RPM_MANUAL = 1, RPM_AUTO_PEAK = 2 };

struct Config {
  // --- extra, fora do Betaflight -------------------------------------------
  bool  dcBlock   = true;
  float dcBlockHz = 2.0f;
  // Escala global aplicada a todos os cortes de passa-baixa, em %
  // (equivalente a simplified_gyro_filter_multiplier)
  uint16_t lpfMultiplier = 100;

  // --- gyro_lpf1_* ---------------------------------------------------------
  bf::LowpassType lpf1Type   = bf::FILTER_PT1;
  float           lpf1Hz     = 250.0f;   // gyro_lpf1_static_hz
  bool            lpf1Dyn    = false;    // usa a faixa dinamica no lugar do estatico
  float           lpf1DynMin = 250.0f;   // gyro_lpf1_dyn_min_hz
  float           lpf1DynMax = 500.0f;   // gyro_lpf1_dyn_max_hz
  uint8_t         lpf1DynExpo = 5;       // gyro_lpf1_dyn_expo (0..10)

  // --- gyro_lpf2_* ---------------------------------------------------------
  bf::LowpassType lpf2Type = bf::FILTER_PT1;
  float           lpf2Hz   = 500.0f;     // gyro_lpf2_static_hz

  // --- gyro_notch1 / gyro_notch2 (0 = desligado) ---------------------------
  float notch1Hz = 0.0f, notch1Cutoff = 0.0f;
  float notch2Hz = 0.0f, notch2Cutoff = 0.0f;

  // --- dyn_notch_* (defaults de src/main/pg/dyn_notch.c) -------------------
  uint8_t  dynNotchCount = 3;
  uint16_t dynNotchQ     = 300;          // x100, como no CLI
  float    dynNotchMinHz = 100.0f;
  float    dynNotchMaxHz = 600.0f;

  // --- rpm_filter_* (defaults de src/main/pg/rpm_filter.c) -----------------
  RpmSource rpmSource       = RPM_OFF;
  float     rpmBaseHz       = 100.0f;    // fundamental quando rpmSource = MANUAL
  uint8_t   rpmHarmonics    = 3;         // rpm_filter_harmonics (0..3)
  uint16_t  rpmQ            = 500;       // rpm_filter_q (x100)
  float     rpmMinHz        = 100.0f;    // rpm_filter_min_hz
  float     rpmFadeRangeHz  = 50.0f;     // rpm_filter_fade_range_hz
  float     rpmLpfHz        = 150.0f;    // rpm_filter_lpf_hz (0 = desliga)
  uint8_t   rpmWeights[RPM_HARMONICS_MAX] = {100, 100, 100};  // rpm_filter_weights
};

class Chain {
 public:
  void  init(const Config &cfg, float sampleRateHz);
  float apply(int axis, float input);
  void  update();

  // Nivel 0..1 que dirige o LPF1 dinamico (no Betaflight seria o acelerador)
  void  setLoad(float load01);
  // Fundamental para o filtro RPM quando a fonte e automatica
  void  setDetectedFundamental(float hz);

  const Config &config() const { return cfg_; }
  float sampleRate() const { return fs_; }

  bool  dynNotchActive() const { return dynCount_ > 0; }
  int   dynCount() const { return dynCount_; }
  float dynCenterFreq(int axis, int p) const { return centerFreq_[axis][p]; }
  float dynResolutionHz() const { return sdftResolutionHz_; }
  float dynSampleRateHz() const { return sdftSampleRateHz_; }

  int   rpmHarmonics() const { return rpmNumHarmonics_; }
  float rpmFreq(int h) const { return rpmFreqHz_[h]; }
  float rpmFundamentalHz() const { return rpmFundamental_; }
  float rpmWeight(int h) const { return rpmWeightOut_[h]; }
  float lpf1CutoffNow() const { return lpf1CutoffNow_; }

 private:
  enum Step : int { STEP_WINDOW = 0, STEP_DETECT_PEAKS, STEP_CALC_FREQUENCIES, STEP_UPDATE_FILTERS, STEP_COUNT };
  static constexpr int   DYN_NOTCH_CALC_TICKS = AXES * STEP_COUNT;  // 12
  static constexpr float DYN_NOTCH_SMOOTH_HZ  = 4.0f;

  struct Peak { int bin; float value; };

  Config cfg_;
  float  fs_ = 1600.0f;

  // ---- cadeia estatica, por eixo
  bf::Pt1     dc_[AXES];
  bool        dcOn_ = false;
  bf::Lowpass lpf1_[AXES], lpf2_[AXES];
  bf::Biquad  notch1_[AXES], notch2_[AXES];
  bool        notch1On_ = false, notch2On_ = false;

  // ---- LPF1 dinamico
  float load_ = 0.0f;
  float lpf1CutoffNow_ = 0.0f;
  uint32_t lpf1UpdateTick_ = 0;

  // ---- filtro RPM
  int        rpmNumHarmonics_ = 0;
  float      rpmQf_ = 5.0f, rpmMaxHz_ = 0;
  float      rpmFreqHz_[RPM_HARMONICS_MAX] = {0};
  float      rpmWeightOut_[RPM_HARMONICS_MAX] = {0};
  float      rpmFundamental_ = 0.0f;     // fundamental ja suavizada
  float      rpmFundamentalRaw_ = 0.0f;  // fonte crua: manual ou pico detectado
  bf::Pt1    rpmFreqLpf_;                // rpm_filter_lpf_hz
  bf::Biquad rpmNotch_[AXES][RPM_HARMONICS_MAX];

  // ---- notch dinamico
  int       dynCount_ = 0;
  float     dynQ_ = 3.0f, dynMinHz_ = 100, dynMaxHz_ = 600;
  bf::Sdft  sdft_[AXES];
  float     sdftData_[bf::SDFT_BIN_COUNT] = {0};
  float     sdftNoiseThreshold_ = 0;
  float     sdftSampleRateHz_ = 0, sdftResolutionHz_ = 0;
  int       sdftStartBin_ = 1, sdftEndBin_ = bf::SDFT_BIN_COUNT - 1;
  float     pt1LooptimeS_ = 0;
  int       sampleIndex_ = 0, sampleCount_ = 1;
  float     sampleCountRcp_ = 1.0f;
  float     sampleAccumulator_[AXES] = {0};
  float     sampleAvg_[AXES] = {0};
  Peak      peaks_[DYN_NOTCH_COUNT_MAX] = {};
  float     centerFreq_[AXES][DYN_NOTCH_COUNT_MAX] = {};
  bf::Biquad dynNotch_[AXES][DYN_NOTCH_COUNT_MAX];
  struct { int tick = 0; int step = 0; int axis = 0; } state_;

  float lpfCut(float hz) const { return hz * (cfg_.lpfMultiplier / 100.0f); }
  void  dynNotchInit();
  void  dynNotchProcess();
  void  rpmInit();
  void  rpmUpdate();
  void  lpf1Update();
};

}  // namespace filtros
