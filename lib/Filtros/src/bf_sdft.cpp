#include "bf_sdft.h"
#include <math.h>

namespace bf {

static float     rPowerN = 0;
static bool      isInitialized = false;
static complex_t twiddle[SDFT_BIN_COUNT];

void Sdft::init(int start, int end, int batches) {
  if (!isInitialized) {
    rPowerN = powf(SDFT_R, SDFT_SAMPLE_SIZE);
    const float c = 2.0f * (float)M_PI / (float)SDFT_SAMPLE_SIZE;
    for (int i = 0; i < SDFT_BIN_COUNT; i++) {
      const float phi = c * i;
      twiddle[i] = SDFT_R * complex_t(cosf(phi), sinf(phi));
    }
    isInitialized = true;
  }

  idx = 0;
  startBin = constrain(start, 0, SDFT_BIN_COUNT - 1);
  endBin   = constrain(end, startBin, SDFT_BIN_COUNT - 1);
  numBatches = max(batches, 1);
  batchSize  = (endBin - startBin + 1) / numBatches;

  for (int i = 0; i < SDFT_SAMPLE_SIZE; i++) samples[i] = 0.0f;
  for (int i = 0; i < SDFT_BIN_COUNT; i++)   data[i] = complex_t(0.0f, 0.0f);
}

void Sdft::push(float sample) {
  const float delta = sample - rPowerN * samples[idx];

  samples[idx] = sample;
  idx = (idx + 1) % SDFT_SAMPLE_SIZE;

  for (int i = startBin; i <= endBin; i++) {
    data[i] = twiddle[i] * (data[i] + delta);
  }
  updateEdges(delta, 0);
}

void Sdft::pushBatch(float sample, int batchIdx) {
  const int batchStart = batchSize * batchIdx + startBin;
  int batchEnd = batchStart;

  const float delta = sample - rPowerN * samples[idx];

  if (batchIdx == numBatches - 1) {
    samples[idx] = sample;
    idx = (idx + 1) % SDFT_SAMPLE_SIZE;
    batchEnd += endBin - batchStart + 1;
  } else {
    batchEnd += batchSize;
  }

  for (int i = batchStart; i < batchEnd && i < SDFT_BIN_COUNT; i++) {
    data[i] = twiddle[i] * (data[i] + delta);
  }
  updateEdges(delta, batchIdx);
}

void Sdft::magSq(float *output) const {
  for (int i = startBin; i <= endBin; i++) {
    const float re = data[i].real(), im = data[i].imag();
    output[i] = re * re + im * im;
  }
}

// Janela de Hann no dominio da frequencia:
//   X[k] = -0.25*X[k-1] + 0.5*X[k] - 0.25*X[k+1]
// (o fator 2 e omitido, como no original, para poupar uma multiplicacao)
void Sdft::winSq(float *output) const {
  complex_t val;

  if (startBin == 0) {
    val = data[startBin] - data[startBin + 1];
  } else {
    val = data[startBin] - 0.5f * (data[startBin - 1] + data[startBin + 1]);
  }
  output[startBin] = val.real() * val.real() + val.imag() * val.imag();

  for (int i = startBin + 1; i < endBin; i++) {
    val = data[i] - 0.5f * (data[i - 1] + data[i + 1]);
    output[i] = val.real() * val.real() + val.imag() * val.imag();
  }

  if (endBin == SDFT_BIN_COUNT - 1) {
    val = data[endBin] - data[endBin - 1];
  } else {
    val = data[endBin] - 0.5f * (data[endBin - 1] + data[endBin + 1]);
  }
  output[endBin] = val.real() * val.real() + val.imag() * val.imag();
}

// Necessario para janelar corretamente nas bordas da faixa ativa
void Sdft::updateEdges(float value, int batchIdx) {
  if (startBin > 0 && batchIdx == 0) {
    const int i = startBin - 1;
    data[i] = twiddle[i] * (data[i] + value);
  }
  if (endBin < SDFT_BIN_COUNT - 1 && batchIdx == numBatches - 1) {
    const int i = endBin + 1;
    data[i] = twiddle[i] * (data[i] + value);
  }
}

}  // namespace bf
