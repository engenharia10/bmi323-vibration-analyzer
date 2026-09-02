#include "dsp.h"
#include <math.h>

namespace dsp {

const char *windowName(Window w) {
  switch (w) {
    case WIN_RECT:     return "rect";
    case WIN_HANN:     return "hann";
    case WIN_HAMMING:  return "hamming";
    case WIN_BLACKMAN: return "blackman";
    case WIN_FLATTOP:  return "flattop";
  }
  return "hann";
}

float buildWindow(Window type, float *win, int n) {
  double sum = 0.0;
  for (int i = 0; i < n; i++) {
    double r = (double)i / (double)(n - 1);
    double v;
    switch (type) {
      case WIN_RECT:     v = 1.0; break;
      case WIN_HAMMING:  v = 0.54 - 0.46 * cos(TWO_PI * r); break;
      case WIN_BLACKMAN: v = 0.42 - 0.5 * cos(TWO_PI * r) + 0.08 * cos(2 * TWO_PI * r); break;
      case WIN_FLATTOP:
        v = 0.21557895 - 0.41663158 * cos(TWO_PI * r) + 0.277263158 * cos(2 * TWO_PI * r) -
            0.083578947 * cos(3 * TWO_PI * r) + 0.006947368 * cos(4 * TWO_PI * r);
        break;
      case WIN_HANN:
      default:           v = 0.5 - 0.5 * cos(TWO_PI * r); break;
    }
    win[i] = (float)v;
    sum += v;
  }
  return (float)(sum / n);  // ganho coerente
}

void fft(float *re, float *im, int n) {
  // Reordenacao bit-reversa
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i];       im[i] = im[j]; im[j] = t;
    }
  }
  // Borboletas
  for (int len = 2; len <= n; len <<= 1) {
    double ang = -TWO_PI / len;
    float wr = (float)cos(ang), wi = (float)sin(ang);
    for (int i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = i + k + len / 2;
        float xr = re[b] * cr - im[b] * ci;
        float xi = re[b] * ci + im[b] * cr;
        re[b] = re[a] - xr; im[b] = im[a] - xi;
        re[a] += xr;        im[a] += xi;
        float nr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = nr;
      }
    }
  }
}

void magnitudeSpectrum(const float *src, const float *win, float coherentGain,
                       float *re, float *im, float *mag, int n) {
  for (int i = 0; i < n; i++) {
    re[i] = src[i] * win[i];
    im[i] = 0.0f;
  }
  fft(re, im, n);

  // Amplitude de uma senoide = 2*|X[k]| / (N * ganho_coerente)
  const float k = 2.0f / (n * (coherentGain > 1e-6f ? coherentGain : 1.0f));
  const int bins = n / 2;
  mag[0] = sqrtf(re[0] * re[0] + im[0] * im[0]) / (n * coherentGain);  // DC sem o fator 2
  for (int i = 1; i < bins; i++) {
    mag[i] = k * sqrtf(re[i] * re[i] + im[i] * im[i]);
  }
}

int findPeaks(const float *mag, int bins, float binHz, float floorAmp,
              Peak *out, int maxPeaks, int minBinSpacing) {
  int found = 0;
  // Ignora o bin 0/1 (DC e vazamento) para nao mascarar a vibracao real.
  for (int i = 2; i < bins - 1; i++) {
    float m = mag[i];
    if (m < floorAmp) continue;
    if (m <= mag[i - 1] || m < mag[i + 1]) continue;

    // Interpolacao parabolica no dominio log para refinar a frequencia.
    float a = logf(fmaxf(mag[i - 1], 1e-12f));
    float b = logf(fmaxf(mag[i], 1e-12f));
    float c = logf(fmaxf(mag[i + 1], 1e-12f));
    float d = a - 2.0f * b + c;
    float delta = (fabsf(d) > 1e-9f) ? 0.5f * (a - c) / d : 0.0f;
    if (delta > 0.5f) delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;

    Peak p;
    p.bin  = i;
    p.freq = (i + delta) * binHz;
    p.amp  = m;

    // Insere mantendo a lista ordenada por amplitude decrescente.
    int pos = found;
    while (pos > 0 && out[pos - 1].amp < p.amp) pos--;
    if (pos >= maxPeaks) continue;
    if (found < maxPeaks) found++;
    for (int k = found - 1; k > pos; k--) out[k] = out[k - 1];
    out[pos] = p;
  }

  // Remove picos muito proximos entre si (mesma raia espectral).
  for (int i = 0; i < found; i++) {
    for (int j = i + 1; j < found;) {
      if (abs(out[j].bin - out[i].bin) < minBinSpacing) {
        for (int k = j; k < found - 1; k++) out[k] = out[k + 1];
        found--;
      } else {
        j++;
      }
    }
  }
  return found;
}

float rmsAC(const float *src, int n) {
  double mean = 0.0;
  for (int i = 0; i < n; i++) mean += src[i];
  mean /= n;
  double acc = 0.0;
  for (int i = 0; i < n; i++) {
    double d = src[i] - mean;
    acc += d * d;
  }
  return (float)sqrt(acc / n);
}

}  // namespace dsp
