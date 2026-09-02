#pragma once
#include <Arduino.h>
#include <complex>

// ---------------------------------------------------------------------------
// Porte da SDFT (Sliding Discrete Fourier Transform) do Betaflight
// (src/main/common/sdft.c). Custo O(N) por amostra para N bins, o que permite
// manter um espectro sempre atualizado sem rodar uma FFT inteira.
//
// Unica mudanca em relacao ao original: std::complex<float> no lugar do
// "float complex" do C99.
// ---------------------------------------------------------------------------

namespace bf {

constexpr int SDFT_SAMPLE_SIZE = 72;
constexpr int SDFT_BIN_COUNT   = SDFT_SAMPLE_SIZE / 2;   // 36
constexpr float SDFT_R = 0.9999f;  // fator de amortecimento (estabilidade, r < 1)

using complex_t = std::complex<float>;

struct Sdft {
  int   idx = 0;                         // indice do buffer circular
  int   startBin = 1;
  int   endBin = SDFT_BIN_COUNT - 1;
  int   batchSize = 1;
  int   numBatches = 1;
  float samples[SDFT_SAMPLE_SIZE] = {0}; // buffer circular
  complex_t data[SDFT_BIN_COUNT];        // espectro complexo

  void init(int startBin, int endBin, int numBatches);
  void push(float sample);                        // atualiza todos os bins
  void pushBatch(float sample, int batchIdx);     // atualiza uma fatia dos bins
  void magSq(float *output) const;                // |X|^2
  void winSq(float *output) const;                // |X|^2 com janela Hann

 private:
  void updateEdges(float value, int batchIdx);
};

}  // namespace bf
