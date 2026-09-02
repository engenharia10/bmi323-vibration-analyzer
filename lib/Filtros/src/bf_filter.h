#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Porte dos filtros do Betaflight (src/main/common/filter.c) para este projeto.
// Nomes, formulas e constantes mantidos identicos ao original para facilitar a
// comparacao. Betaflight e GPLv3.
//
// Diferenca de assinatura: o original recebe refreshRate em microssegundos;
// aqui usamos a taxa de amostragem em Hz, que e o que temos a mao.
// ---------------------------------------------------------------------------

namespace bf {

// BIQUAD_Q = 1/sqrt(2) -> secao Butterworth de 2a ordem
constexpr float BIQUAD_Q = 0.70710678f;

// Correcao de corte dos PTn: 1 / sqrt(2^(1/n) - 1)
constexpr float CUTOFF_CORRECTION_PT2 = 1.553773974f;
constexpr float CUTOFF_CORRECTION_PT3 = 1.961459177f;

// Ordem identica a do enum lowpassFilterType_e do Betaflight
enum LowpassType : uint8_t { FILTER_PT1 = 0, FILTER_BIQUAD = 1, FILTER_PT2 = 2, FILTER_PT3 = 3 };
enum BiquadType  : uint8_t { BQ_LPF = 0, BQ_NOTCH = 1, BQ_BPF = 2 };

float pt1FilterGain(float fCut, float dT);

struct Pt1 {
  float state = 0, k = 1;
  void  init(float gain) { state = 0; k = gain; }
  void  updateCutoff(float gain) { k = gain; }
  inline float apply(float input) {
    state = state + k * (input - state);
    return state;
  }
};

struct Pt2 {
  float state = 0, state1 = 0, k = 1;
  void  init(float gain) { state = state1 = 0; k = gain; }
  inline float apply(float input) {
    state1 = state1 + k * (input - state1);
    state  = state + k * (state1 - state);
    return state;
  }
};

struct Pt3 {
  float state = 0, state1 = 0, state2 = 0, k = 1;
  void  init(float gain) { state = state1 = state2 = 0; k = gain; }
  inline float apply(float input) {
    state1 = state1 + k * (input - state1);
    state2 = state2 + k * (state1 - state2);
    state  = state + k * (state2 - state);
    return state;
  }
};

// Q de um notch dado o centro (f0) e a frequencia de corte inferior (f1).
// Q = f0 / (f2 - f1), com f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq);

struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  float weight = 1.0f;

  void update(float filterFreq, float sampleRateHz, float q, BiquadType type, float w);
  void init(float filterFreq, float sampleRateHz, float q, BiquadType type, float w);
  void initLPF(float filterFreq, float sampleRateHz) { init(filterFreq, sampleRateHz, BIQUAD_Q, BQ_LPF, 1.0f); }
  void reset() { x1 = x2 = y1 = y2 = 0; }

  // DF1: menos preciso que DF2, mas aguenta troca de coeficientes em tempo real
  // (e por isso que o notch dinamico usa esta forma).
  inline float applyDF1(float input) {
    const float result = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = input;
    y2 = y1; y1 = result;
    return result;
  }
  inline float applyDF1Weighted(float input) {
    const float result = applyDF1(input);
    return weight * result + (1 - weight) * input;
  }
  // DF2 transposta: mais precisa, so para coeficientes fixos
  inline float applyDF2(float input) {
    const float result = b0 * input + x1;
    x1 = b1 * input - a1 * result + x2;
    x2 = b2 * input - a2 * result;
    return result;
  }
};

// Filtro passa-baixa generico com o tipo selecionavel, como o gyro.lowpassFilter
// do Betaflight (PT1 / BIQUAD / PT2 / PT3, ou desligado quando hz <= 0).
struct Lowpass {
  LowpassType type = FILTER_PT1;
  bool        enabled = false;
  // Quando o corte muda com o filtro em regime, os coeficientes do biquad sao
  // reescritos com o estado quente e so a DF1 aguenta isso. O Betaflight faz o
  // mesmo: em gyro_init.c o passa-baixa biquad vira biquadFilterApplyDF1
  // quando USE_DYN_LPF esta ligado. Com DF2 daria transiente a cada update.
  bool        biquadDF1 = false;
  Pt1 pt1; Pt2 pt2; Pt3 pt3; Biquad bq;

  // dynamicCutoff: o corte sera reescrito depois do init (caso do LPF1 dinamico)
  void init(LowpassType t, float cutoffHz, float sampleRateHz, bool dynamicCutoff = false);
  inline float apply(float input) {
    if (!enabled) return input;
    switch (type) {
      case FILTER_PT1:    return pt1.apply(input);
      case FILTER_PT2:    return pt2.apply(input);
      case FILTER_PT3:    return pt3.apply(input);
      case FILTER_BIQUAD: return biquadDF1 ? bq.applyDF1(input) : bq.applyDF2(input);
    }
    return input;
  }
};

}  // namespace bf
