#include "simulador.h"
#include <math.h>

namespace sim {

Generator gen;

// Pesos por eixo quando a fonte e "todos": nao sao iguais de proposito, uma
// maquina real nunca vibra igual nos tres eixos.
static const float kAxisWeight[3] = {0.85f, 0.70f, 1.00f};
// Defasagem por eixo para os eixos nao ficarem identicos
static const float kAxisPhase[3] = {0.0f, 1.1f, 2.3f};
static constexpr float DEG2RAD = 0.01745329f;
static constexpr float RAD2DEG = 57.2957795f;

void Generator::init(const Config &cfg, float sampleRateHz) {
  cfg_ = cfg;
  fs_  = sampleRateHz > 1.0f ? sampleRateHz : 1600.0f;
  dt_  = 1.0f / fs_;
  t_   = 0.0;

  for (int i = 0; i < SOURCES; i++) {
    jitter_[i] = 0.0f;
    curFreq_[i] = cfg_.src[i].freqHz;
    for (int h = 0; h < HARMONICS_MAX; h++) {
      // fases iniciais espalhadas para o sinal nao comecar com um pico gigante
      phase_[i][h] = (float)((i * 7 + h * 3) % 16) * 0.3927f;
    }
  }

  attRoll_ = attPitch_ = attYaw_ = 0.0f;
  resGain_ = cfg_.resonanceOn ? powf(10.0f, cfg_.resonanceGainDb / 20.0f) - 1.0f : 0.0f;
  if (cfg_.resonanceOn) {
    for (int ax = 0; ax < 3; ax++) {
      res_[ax].init(cfg_.resonanceHz, fs_, cfg_.resonanceQ, bf::BQ_BPF, 1.0f);
    }
  }
}

float Generator::rand01() {
  rng_ ^= rng_ << 13;
  rng_ ^= rng_ >> 17;
  rng_ ^= rng_ << 5;
  return (rng_ & 0xFFFFFF) / 16777215.0f;
}

// Aproximacao barata de ruido gaussiano: soma de tres uniformes.
float Generator::noise() {
  return (rand01() + rand01() + rand01() - 1.5f) * 1.1547f;
}

void Generator::next(float acc[3], float gyr[3]) {
  float acc_[3] = {0, 0, 0};

  for (int i = 0; i < SOURCES; i++) {
    const Source &s = cfg_.src[i];
    if (!s.enabled || s.ampMg <= 0.0f) { curFreq_[i] = s.freqHz; continue; }

    // deriva senoidal lenta: e o que faz o notch dinamico ter o que perseguir
    float f = s.freqHz;
    if (s.driftHz != 0.0f && s.driftPeriodS > 0.01f) {
      f += s.driftHz * sinf(2.0f * (float)M_PI * (float)t_ / s.driftPeriodS);
    }
    // jitter: passeio aleatorio de primeira ordem em torno de zero
    if (s.jitterHz > 0.0f) {
      jitter_[i] += (noise() * s.jitterHz * 0.05f - jitter_[i] * 0.002f);
      f += jitter_[i];
    }
    if (f < 0.5f) f = 0.5f;
    curFreq_[i] = f;

    const float decay = s.harmDecayPct / 100.0f;
    const int   nh = constrain((int)s.harmonics, 1, HARMONICS_MAX);
    float amp = s.ampMg * 0.001f;   // mg -> g

    for (int h = 0; h < nh; h++) {
      const float fh = f * (h + 1);
      if (fh >= fs_ * 0.5f) break;               // acima de Nyquist so criaria alias
      phase_[i][h] += 2.0f * (float)M_PI * fh * dt_;
      if (phase_[i][h] > 2.0f * (float)M_PI) phase_[i][h] -= 2.0f * (float)M_PI;

      const float a = amp * powf(decay, (float)h);
      if (s.axis < 3) {
        acc_[s.axis] += a * sinf(phase_[i][h]);
      } else {
        for (int ax = 0; ax < 3; ax++) {
          acc_[ax] += a * kAxisWeight[ax] * sinf(phase_[i][h] + kAxisPhase[ax]);
        }
      }
    }
  }

  const float noiseAmp = cfg_.noiseMg * 0.001f;
  for (int ax = 0; ax < 3; ax++) {
    float v = acc_[ax];
    // ressonancia da estrutura: realca uma faixa, como a fixacao da placa faz
    if (resGain_ > 0.0f) v += resGain_ * res_[ax].applyDF2(v);
    if (noiseAmp > 0.0f) v += noise() * noiseAmp;
    acc[ax] = v;
  }

  // --- movimento do corpo: gravidade projetada nos eixos + taxas angulares ---
  float rateDeg[3] = {0, 0, 0};
  if (cfg_.attOn) {
    const float w = (cfg_.attPeriodS > 0.01f) ? 2.0f * (float)M_PI / cfg_.attPeriodS : 0.0f;
    const float phi   = cfg_.rollAmpDeg  * sinf(w * (float)t_) * DEG2RAD;
    const float theta = cfg_.pitchAmpDeg * sinf(w * (float)t_ * 0.73f + 0.9f) * DEG2RAD;
    attRoll_  = phi * RAD2DEG;
    attPitch_ = theta * RAD2DEG;
    attYaw_  += cfg_.yawRateDps * dt_;
    if (attYaw_ > 180.0f) attYaw_ -= 360.0f;

    if (cfg_.gravity) {
      // vetor gravidade escrito no referencial do corpo
      acc[0] += -sinf(theta);
      acc[1] +=  sinf(phi) * cosf(theta);
      acc[2] +=  cosf(phi) * cosf(theta);
    }
    // derivada analitica dos angulos = velocidade angular medida pelo giro
    rateDeg[0] = cfg_.rollAmpDeg  * w * cosf(w * (float)t_) ;
    rateDeg[1] = cfg_.pitchAmpDeg * w * 0.73f * cosf(w * (float)t_ * 0.73f + 0.9f);
    rateDeg[2] = cfg_.yawRateDps;
  } else if (cfg_.gravity) {
    acc[2] += 1.0f;
  }

  if (gyr) {
    // o giroscopio tambem sente a vibracao, so que bem mais fraca
    for (int i = 0; i < 3; i++) gyr[i] = rateDeg[i] + noise() * 0.4f;
  }

  t_ += dt_;
}

}  // namespace sim
