#pragma once
#include <Arduino.h>
#include "config.h"

// ---------------------------------------------------------------------------
// FFT radix-2 in-place + janelamento + deteccao de picos com interpolacao
// parabolica. Tudo em float (a FPU do ESP32-S3 da conta de sobra).
// ---------------------------------------------------------------------------

namespace dsp {

enum Window : uint8_t { WIN_RECT = 0, WIN_HANN = 1, WIN_HAMMING = 2, WIN_BLACKMAN = 3, WIN_FLATTOP = 4 };

const char *windowName(Window w);

// Preenche win[0..n-1] e devolve o ganho coerente (media da janela),
// usado para normalizar a amplitude do espectro.
float buildWindow(Window type, float *win, int n);

// FFT complexa in-place. n deve ser potencia de 2.
void fft(float *re, float *im, int n);

// Espectro de amplitude de um sinal real ja janelado.
//   src: n amostras (a janela e aplicada internamente)
//   mag: n/2 bins de amplitude na mesma unidade de src
void magnitudeSpectrum(const float *src, const float *win, float coherentGain,
                       float *re, float *im, float *mag, int n);

struct Peak {
  float freq;   // Hz, com interpolacao parabolica
  float amp;    // amplitude do pico
  int   bin;
};

// Encontra ate maxPeaks maximos locais acima de floorAmp, ordenados por amplitude.
int findPeaks(const float *mag, int bins, float binHz, float floorAmp,
              Peak *out, int maxPeaks, int minBinSpacing);

// RMS AC (remove a media) de um trecho.
float rmsAC(const float *src, int n);

}  // namespace dsp
