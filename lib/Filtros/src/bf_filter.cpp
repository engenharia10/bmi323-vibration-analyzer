#include "bf_filter.h"
#include <math.h>

namespace bf {

float pt1FilterGain(float fCut, float dT) {
  const float omega = 2.0f * (float)M_PI * fCut * dT;
  return omega / (omega + 1.0f);
}

// Q = f0 / (f2 - f1), com f2 = f0^2 / f1
float filterGetNotchQ(float centerFreq, float cutoffFreq) {
  if (cutoffFreq <= 0.0f || cutoffFreq >= centerFreq) return 0.0f;
  return centerFreq * cutoffFreq / (centerFreq * centerFreq - cutoffFreq * cutoffFreq);
}

void Biquad::update(float filterFreq, float sampleRateHz, float q, BiquadType type, float w) {
  const float nyquist = sampleRateHz * 0.5f;
  if (filterFreq > nyquist * 0.98f) filterFreq = nyquist * 0.98f;
  if (filterFreq < 0.1f) filterFreq = 0.1f;
  if (q < 0.05f) q = 0.05f;

  const float omega = 2.0f * (float)M_PI * filterFreq / sampleRateHz;
  const float sn = sinf(omega);
  const float cs = cosf(omega);
  const float alpha = sn / (2.0f * q);

  switch (type) {
    case BQ_LPF:
      // Secao Butterworth de 2a ordem (Q = 1/sqrt(2)) - TI slaa447
      b1 = 1 - cs;
      b0 = b1 * 0.5f;
      b2 = b0;
      a1 = -2 * cs;
      a2 = 1 - alpha;
      break;
    case BQ_NOTCH:
      b0 = 1;
      b1 = -2 * cs;
      b2 = 1;
      a1 = b1;
      a2 = 1 - alpha;
      break;
    case BQ_BPF:
      b0 = alpha;
      b1 = 0;
      b2 = -alpha;
      a1 = -2 * cs;
      a2 = 1 - alpha;
      break;
  }

  const float a0 = 1 + alpha;
  b0 /= a0;
  b1 /= a0;
  b2 /= a0;
  a1 /= a0;
  a2 /= a0;

  weight = w;
}

void Biquad::init(float filterFreq, float sampleRateHz, float q, BiquadType type, float w) {
  update(filterFreq, sampleRateHz, q, type, w);
  reset();
}

void Lowpass::init(LowpassType t, float cutoffHz, float sampleRateHz, bool dynamicCutoff) {
  type = t;
  biquadDF1 = dynamicCutoff;
  enabled = (cutoffHz > 0.0f) && (cutoffHz < sampleRateHz * 0.5f);
  if (!enabled) return;

  const float dT = 1.0f / sampleRateHz;
  switch (type) {
    case FILTER_PT1:
      pt1.init(pt1FilterGain(cutoffHz, dT));
      break;
    case FILTER_PT2:
      // desloca o corte para satisfazer a condicao de -3 dB
      pt2.init(pt1FilterGain(cutoffHz * CUTOFF_CORRECTION_PT2, dT));
      break;
    case FILTER_PT3:
      pt3.init(pt1FilterGain(cutoffHz * CUTOFF_CORRECTION_PT3, dT));
      break;
    case FILTER_BIQUAD:
      bq.initLPF(cutoffHz, sampleRateHz);
      break;
  }
}

}  // namespace bf
