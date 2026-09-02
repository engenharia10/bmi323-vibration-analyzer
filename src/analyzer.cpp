#include "analyzer.h"
#include <bmi323.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace vib {

// ------------------------------------------------------------- estado global

static SensorCfg       g_sensor;
static AnalysisCfg     g_analysis;
static filtros::Config g_filter;
static sim::Config     g_sim;

// Uma cadeia para o acelerometro e outra para o giroscopio: cada uma com seu
// proprio notch dinamico, como no Betaflight (que filtra so o giroscopio).
static filtros::Chain  g_chainAcc;
static filtros::Chain  g_chainGyr;

static volatile bool g_reconfigSensor  = false;
static volatile bool g_reconfigFilters = false;
static volatile bool g_reconfigSim     = false;

// Buffer circular dos 6 canais. O cru fica em contagens nativas do sensor
// (int16): ocupa metade da memoria e nao perde nada, porque e exatamente a
// resolucao que o BMI323 entrega. O filtrado precisa de float.
//
// Estes buffers vao para a PSRAM quando ela existe. Sao percorridos em
// rajadas sequenciais (escrita por amostra, leitura em blocos pela FFT),
// que e o padrao de acesso em que a PSRAM se sai bem. O scratch da FFT
// fica na RAM interna de proposito: ali o acesso e aleatorio e quente.
typedef int16_t RawRow[CHANNELS];
typedef float   FiltRow[CHANNELS];
static RawRow  *g_raw  = nullptr;
static FiltRow *g_filt = nullptr;
static bool     g_psram = false;
static size_t   g_bufBytes = 0;
static float   g_scale[CHANNELS]    = {1, 1, 1, 1, 1, 1};   // contagem -> g / graus/s
static float   g_invScale[CHANNELS] = {1, 1, 1, 1, 1, 1};
static volatile uint32_t g_write = 0;   // contador monotonico de amostras
static volatile uint32_t g_overruns = 0;

static Results g_res;
static SemaphoreHandle_t g_resMtx;
static SemaphoreHandle_t g_cfgMtx;

static float g_measuredHz = 0;
static float g_tempC = 0;
static float g_load = 0;

// Scratch da FFT (fora da pilha das tasks)
static float s_win[FFT_MAX];
static float s_re[FFT_MAX];
static float s_im[FFT_MAX];
static float s_mag[FFT_MAX / 2];
typedef float SpecRow[FFT_MAX / 2];
static SpecRow *s_avg = nullptr;
static float s_avgFilt[FFT_MAX / 2];
static uint8_t s_avgValidMask = 0;
static bool  s_avgFiltValid = false;
static int   s_winSize = 0;
static dsp::Window s_winType = dsp::WIN_HANN;
static float s_coherentGain = 0.5f;

// Prefere PSRAM; se nao houver, usa a RAM interna e segue o jogo.
static void *bigAlloc(size_t n) {
  void *p = nullptr;
  if (psramFound()) p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  if (p) g_psram = true;
  else   p = malloc(n);
  if (p) { memset(p, 0, n); g_bufBytes += n; }
  return p;
}

static void refreshScales() {
  const float a = 1.0f / bmi::imu.lsbPerG();
  const float g = 1.0f / bmi::imu.lsbPerDps();
  for (int ch = 0; ch < 3; ch++) { g_scale[ch] = a; g_scale[3 + ch] = g; }
  for (int ch = 0; ch < CHANNELS; ch++) g_invScale[ch] = 1.0f / g_scale[ch];
}

// Canal CH_ACC_MAG = modulo do acelerometro; os demais sao diretos.
static inline float rawValue(const int16_t v[CHANNELS], uint8_t ch) {
  if (ch < CHANNELS) return v[ch] * g_scale[ch];
  const float x = v[0] * g_scale[0], y = v[1] * g_scale[1], z = v[2] * g_scale[2];
  return sqrtf(x * x + y * y + z * z);
}
static inline float filtValue(const float v[CHANNELS], uint8_t ch) {
  if (ch < CHANNELS) return v[ch];
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// --------------------------------------------------------------- aquisicao

static inline int16_t toCounts(float v, float inv) {
  float c = v * inv;
  if (c > 32767.0f) c = 32767.0f;
  if (c < -32768.0f) c = -32768.0f;
  return (int16_t)lrintf(c);
}

static inline void pushSample(const float acc[3], const float gyr[3], bool haveGyro) {
  uint32_t idx = g_write % RING_SAMPLES;
  float accF[3], gyrF[3];
  for (int ax = 0; ax < 3; ax++) {
    accF[ax] = g_chainAcc.apply(ax, acc[ax]);
    gyrF[ax] = g_chainGyr.apply(ax, gyr[ax]);
    g_raw[idx][ax]      = toCounts(acc[ax], g_invScale[ax]);
    g_filt[idx][ax]     = accF[ax];
    g_raw[idx][3 + ax]  = toCounts(gyr[ax], g_invScale[3 + ax]);
    g_filt[idx][3 + ax] = gyrF[ax];
  }
  g_chainAcc.update();   // maquina de estados do notch dinamico, 1x por amostra
  g_chainGyr.update();

  // Atitude a partir do sinal escolhido pelo usuario. O giroscopio filtrado
  // e quase sempre o melhor: tira a vibracao antes de integrar. Ja o
  // acelerometro so pode vir filtrado se o DC-block estiver desligado -
  // senao a gravidade, que e exatamente a referencia procurada, some.
  const uint8_t src = att::est.config().source;
  const bool gyrFilt = haveGyro && src >= att::SRC_GYRO;
  const bool accFilt = (src == att::SRC_BOTH) && !g_filter.dcBlock;
  att::est.setSourceInfo(accFilt, gyrFilt);
  att::est.update(accFilt ? accF : acc, haveGyro ? (gyrFilt ? gyrF : gyr) : nullptr);

  g_write++;
}

static void acqTask(void *) {
  static int16_t frames[96 * 6];
  uint32_t lastCount = 0;
  uint32_t lastTick = millis();
  uint32_t lastTemp = 0;
  uint32_t lastSimUs = micros();
  float    simAccum = 0;

  for (;;) {
    if (g_reconfigSensor) {
      xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
      SensorCfg s = g_sensor;
      filtros::Config f = g_filter;
      g_reconfigSensor = false;
      xSemaphoreGive(g_cfgMtx);
      bmi::imu.configureAccel(s.odrCode, s.rangeCode, s.avgCode, s.bwCode, s.gyroOn, s.gyrRange);
      refreshScales();
      g_write = 0;   // fundo de escala mudou: o historico anterior nao vale mais
      g_chainAcc.init(f, bmi::imu.odr());
      g_chainGyr.init(f, bmi::imu.odr());
      sim::gen.init(g_sim, bmi::imu.odr());
      att::est.begin(bmi::imu.odr());
      s_avgValidMask = 0;
      s_avgFiltValid = false;
    }
    if (g_reconfigFilters) {
      xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
      filtros::Config f = g_filter;
      g_reconfigFilters = false;
      xSemaphoreGive(g_cfgMtx);
      g_chainAcc.init(f, bmi::imu.odr());
      g_chainGyr.init(f, bmi::imu.odr());
    }
    if (g_reconfigSim) {
      xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
      sim::Config sc = g_sim;
      g_reconfigSim = false;
      xSemaphoreGive(g_cfgMtx);
      sim::gen.init(sc, bmi::imu.odr() > 0 ? bmi::imu.odr() : 1600.0f);
      simAccum = 0;
      lastSimUs = micros();
    }

    if (g_sim.enabled) {
      // Gera em tempo real: a quantidade de amostras sai do tempo decorrido,
      // para o notch dinamico e a FFT verem a mesma taxa do sensor.
      const float fs = bmi::imu.odr() > 0 ? bmi::imu.odr() : 1600.0f;
      uint32_t nowUs = micros();
      simAccum += (nowUs - lastSimUs) * 1e-6f * fs;
      lastSimUs = nowUs;
      int n = (int)simAccum;
      simAccum -= n;
      if (n > 96) { n = 96; simAccum = 0; }   // atrasou: descarta em vez de acumular

      for (int i = 0; i < n; i++) {
        float acc[3], gyr[3];
        sim::gen.next(acc, gyr);
        pushSample(acc, gyr, true);
      }
    } else {
      const int fw = bmi::imu.frameWords();
      int n = bmi::imu.fifoReadFrames(frames, 96);
      if (n > 0) {
        const float aScale = 1.0f / bmi::imu.lsbPerG();
        const float gScale = 1.0f / bmi::imu.lsbPerDps();
        for (int i = 0; i < n; i++) {
          const int16_t *fr = frames + i * fw;
          float acc[3], gyr[3] = {0, 0, 0};
          for (int ax = 0; ax < 3; ax++) acc[ax] = fr[ax] * aScale;
          if (fw >= 6) for (int ax = 0; ax < 3; ax++) gyr[ax] = fr[3 + ax] * gScale;
          pushSample(acc, gyr, fw >= 6);
        }
      }
    }

    uint32_t now = millis();
    if (now - lastTick >= 1000) {
      uint32_t c = g_write;
      g_measuredHz = (float)(c - lastCount) * 1000.0f / (float)(now - lastTick);
      lastCount = c;
      lastTick = now;
    }
    if (now - lastTemp >= 2000) {
      lastTemp = now;
      g_tempC = bmi::imu.present() ? bmi::imu.temperatureC() : 30.0f;
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ------------------------------------------------------------------- DSP

// Copia N amostras de um canal terminando na escrita mais recente, sem travar
// a aquisicao. Repete se o produtor deu a volta no buffer durante a leitura.
static bool snapshot(int n, uint8_t ch, float *rawOut, float *filtOut) {
  for (int attempt = 0; attempt < 3; attempt++) {
    uint32_t end = g_write;
    if (end < (uint32_t)n) return false;
    uint32_t start = end - n;
    for (int i = 0; i < n; i++) {
      uint32_t idx = (start + i) % RING_SAMPLES;
      rawOut[i] = rawValue(g_raw[idx], ch);
      if (filtOut) filtOut[i] = filtValue(g_filt[idx], ch);
    }
    if (g_write - start <= (uint32_t)RING_SAMPLES) return true;
    g_overruns++;
  }
  return false;
}

// Ultima amostra que entrou no buffer, sem conversao nenhuma: e o valor que
// veio do FIFO do BMI323 no ultimo quadro lido.
static void lastRawFrame(int16_t out[CHANNELS]) {
  const uint32_t w = g_write;
  if (w == 0) { for (int c = 0; c < CHANNELS; c++) out[c] = 0; return; }
  const int16_t *v = g_raw[(w - 1) % RING_SAMPLES];
  for (int c = 0; c < CHANNELS; c++) out[c] = v[c];
}

// Media e RMS AC dos 6 eixos numa janela curta do sinal CRU. Roda para
// todos os canais, mesmo os escondidos no espectro: a faixa de leitura da
// interface mostra os 6 sempre.
static void rawSummary(float dc[CHANNELS], float rms[CHANNELS]) {
  const int N = 256;
  uint32_t end = g_write;
  if (end < (uint32_t)N) { for (int c = 0; c < CHANNELS; c++) dc[c] = rms[c] = 0; return; }
  const uint32_t start = end - N;

  double sum[CHANNELS] = {0}, sq[CHANNELS] = {0};
  for (int i = 0; i < N; i++) {
    const int16_t *v = g_raw[(start + i) % RING_SAMPLES];
    for (int c = 0; c < CHANNELS; c++) {
      const double x = v[c] * g_scale[c];
      sum[c] += x;
      sq[c]  += x * x;
    }
  }
  for (int c = 0; c < CHANNELS; c++) {
    const double m = sum[c] / N;
    dc[c] = (float)m;
    const double var = sq[c] / N - m * m;
    rms[c] = (float)sqrt(var > 0 ? var : 0);
  }
}

// Maior raia do espectro, com interpolacao parabolica para a frequencia
// nao ficar presa na grade dos bins.
static void dominant(const float *mag, int bins, float binHz, float &f, float &a) {
  int best = 2;
  for (int i = 3; i < bins - 1; i++) if (mag[i] > mag[best]) best = i;
  a = mag[best];
  const float y0 = logf(fmaxf(mag[best - 1], 1e-12f));
  const float y1 = logf(fmaxf(mag[best], 1e-12f));
  const float y2 = logf(fmaxf(mag[best + 1], 1e-12f));
  const float d = y0 - 2.0f * y1 + y2;
  float delta = (fabsf(d) > 1e-9f) ? 0.5f * (y0 - y2) / d : 0.0f;
  delta = constrain(delta, -0.5f, 0.5f);
  f = (best + delta) * binHz;
}

static void dspTask(void *) {
  static float buf[FFT_MAX];
  static float bufFilt[FFT_MAX];

  for (;;) {
    xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
    AnalysisCfg a = g_analysis;
    xSemaphoreGive(g_cfgMtx);

    const int n = a.fftSize;
    const int bins = n / 2;
    const float fs = bmi::imu.odr() > 0 ? bmi::imu.odr() : 1600.0f;
    const float binHz = fs / n;
    const float alpha = constrain(a.avgAlpha, 0.05f, 1.0f);

    if (s_winSize != n || s_winType != a.window) {
      s_coherentGain = dsp::buildWindow(a.window, s_win, n);
      s_winSize = n;
      s_winType = a.window;
      s_avgValidMask = 0;
      s_avgFiltValid = false;
    }

    // O canal em detalhe entra na conta mesmo se o usuario escondeu a curva:
    // picos, RMS e osciloscopio saem dele.
    uint8_t mask = a.chanMask;
    if (a.axis < CHANNELS) mask |= (1 << a.axis);

    bool any = false;
    float dcCh[CHANNELS] = {0}, rmsCh[CHANNELS] = {0};
    rawSummary(dcCh, rmsCh);
    int16_t lsbCh[CHANNELS] = {0};
    lastRawFrame(lsbCh);

    float domF[CHANNELS] = {0}, domA[CHANNELS] = {0};

    // Os 6 canais entram na conta mesmo escondidos: a frequencia dominante
    // por eixo e justamente o que orienta o ajuste dos filtros. A mascara
    // decide apenas o que vai para a rede.
    for (int ch = 0; ch < CHANNELS; ch++) {
      if (!snapshot(n, ch, buf, nullptr)) continue;
      any = true;

      dsp::magnitudeSpectrum(buf, s_win, s_coherentGain, s_re, s_im, s_mag, n);
      if (!(s_avgValidMask & (1 << ch))) {
        memcpy(s_avg[ch], s_mag, bins * sizeof(float));
        s_avgValidMask |= (1 << ch);
      } else {
        for (int i = 0; i < bins; i++) s_avg[ch][i] += alpha * (s_mag[i] - s_avg[ch][i]);
      }
      dominant(s_avg[ch], bins, binHz, domF[ch], domA[ch]);
    }

    // Canal selecionado: precisa tambem do sinal filtrado, para a comparacao
    float pkpk = 0, rmsRaw = 0, rmsFilt = 0;
    int np = 0;
    dsp::Peak peaks[6];
    memset(peaks, 0, sizeof(peaks));

    if (any && snapshot(n, a.axis, buf, bufFilt)) {
      dsp::magnitudeSpectrum(bufFilt, s_win, s_coherentGain, s_re, s_im, s_mag, n);
      if (!s_avgFiltValid) {
        memcpy(s_avgFilt, s_mag, bins * sizeof(float));
        s_avgFiltValid = true;
      } else {
        for (int i = 0; i < bins; i++) s_avgFilt[i] += alpha * (s_mag[i] - s_avgFilt[i]);
      }

      // O canal em detalhe pode ser o modulo (CH_ACC_MAG), que nao esta em
      // s_avg; nesse caso o espectro de referencia sai daqui.
      const float *ref;
      if (a.axis < CHANNELS) {
        ref = s_avg[a.axis];
      } else {
        dsp::magnitudeSpectrum(buf, s_win, s_coherentGain, s_re, s_im, s_mag, n);
        ref = s_mag;
      }

      // Piso de deteccao: 3x a amplitude media do espectro.
      float sum = 0;
      for (int i = 2; i < bins; i++) sum += ref[i];
      const float floorAmp = (sum / (bins - 2)) * 3.0f;
      np = dsp::findPeaks(ref, bins, binHz, floorAmp, peaks, 6, max(2, (int)(3.0f / binHz)));

      // Alimenta o filtro RPM em modo automatico com o pico dominante
      if (np > 0) {
        g_chainAcc.setDetectedFundamental(peaks[0].freq);
        g_chainGyr.setDetectedFundamental(peaks[0].freq);
      }

      float mn = buf[0], mx = buf[0];
      for (int i = 1; i < n; i++) {
        if (buf[i] < mn) mn = buf[i];
        if (buf[i] > mx) mx = buf[i];
      }
      pkpk = mx - mn;
      rmsRaw  = dsp::rmsAC(buf, n);
      rmsFilt = dsp::rmsAC(bufFilt, n);
    }

    if (any) {
      const bool gyroCh = (a.axis >= 3 && a.axis < CHANNELS);
      const filtros::Chain &shown = gyroCh ? g_chainGyr : g_chainAcc;

      xSemaphoreTake(g_resMtx, portMAX_DELAY);
      g_res.bins  = bins;
      g_res.binHz = binHz;
      g_res.fs    = fs;
      g_res.chanMask = mask;
      for (int ch = 0; ch < CHANNELS; ch++) {
        if (mask & (1 << ch)) memcpy(g_res.spec[ch], s_avg[ch], bins * sizeof(float));
      }
      memcpy(g_res.specFilt, s_avgFilt, bins * sizeof(float));
      memcpy(g_res.dcCh, dcCh, sizeof(dcCh));
      memcpy(g_res.rmsCh, rmsCh, sizeof(rmsCh));
      memcpy(g_res.lsbCh, lsbCh, sizeof(lsbCh));
      memcpy(g_res.domFreq, domF, sizeof(domF));
      memcpy(g_res.domAmp, domA, sizeof(domA));
      g_res.nPeaks = np;
      memcpy(g_res.peaks, peaks, sizeof(peaks));
      g_res.rmsRaw  = rmsRaw;
      g_res.rmsFilt = rmsFilt;
      g_res.peakToPeakRaw = pkpk;
      g_res.tempC = g_tempC;
      g_res.measuredHz = g_measuredHz;
      g_res.overruns = g_overruns;
      g_res.dynCount = shown.dynCount();
      for (int p = 0; p < filtros::DYN_NOTCH_COUNT_MAX; p++) {
        g_res.dynFreq[p] = shown.dynCenterFreq(a.axis % 3, p);
      }
      g_res.rpmHarmonics = shown.rpmHarmonics();
      for (int h = 0; h < filtros::RPM_HARMONICS_MAX; h++) {
        g_res.rpmFreq[h]   = shown.rpmFreq(h);
        g_res.rpmWeight[h] = shown.rpmWeight(h);
      }
      g_res.lpf1Cutoff = shown.lpf1CutoffNow();
      g_res.attitude   = att::est.state();
      g_res.simulated  = g_sim.enabled;
      for (int i = 0; i < sim::SOURCES; i++) g_res.simFreq[i] = sim::gen.currentFreq(i);
      g_res.seq++;
      xSemaphoreGive(g_resMtx);
    }

    vTaskDelay(pdMS_TO_TICKS(SPECTRUM_PERIOD_MS / 2));
  }
}

// ------------------------------------------------------------------- API

bool begin() {
  g_resMtx = xSemaphoreCreateMutex();
  g_cfgMtx = xSemaphoreCreateMutex();

  g_raw  = (RawRow *)bigAlloc(sizeof(RawRow) * RING_SAMPLES);
  g_filt = (FiltRow *)bigAlloc(sizeof(FiltRow) * RING_SAMPLES);
  s_avg  = (SpecRow *)bigAlloc(sizeof(SpecRow) * CHANNELS);
  if (!g_raw || !g_filt || !s_avg) {
    Serial.println("[mem] sem memoria para os buffers do analisador");
    return false;
  }
  Serial.printf("[mem] %u kB de buffers em %s\n",
                (unsigned)(g_bufBytes / 1024), g_psram ? "PSRAM" : "RAM interna");

  bmi::BusConfig bus;
  bus.useSpi  = BMI_USE_SPI;
  bus.sck     = PIN_BMI_SCK;
  bus.miso    = PIN_BMI_MISO;
  bus.mosi    = PIN_BMI_MOSI;
  bus.cs      = PIN_BMI_CS;
  bus.spiHz   = BMI_SPI_HZ;
  bus.sda     = PIN_BMI_SDA;
  bus.scl     = PIN_BMI_SCL;
  bus.i2cAddr = BMI_I2C_ADDR;
  bus.i2cHz   = BMI_I2C_HZ;

  bool ok = bmi::imu.begin(bus);
  if (ok) {
    bmi::imu.configureAccel(g_sensor.odrCode, g_sensor.rangeCode,
                            g_sensor.avgCode, g_sensor.bwCode,
                            g_sensor.gyroOn, g_sensor.gyrRange);
  }
  refreshScales();
  g_chainAcc.init(g_filter, bmi::imu.odr());
  g_chainGyr.init(g_filter, bmi::imu.odr());
  sim::gen.init(g_sim, bmi::imu.odr());
  att::est.begin(bmi::imu.odr());

  xTaskCreatePinnedToCore(acqTask, "acq", 4096, nullptr, 6, nullptr, TASK_ACQ_CORE);
  xTaskCreatePinnedToCore(dspTask, "dsp", 6144, nullptr, 3, nullptr, TASK_DSP_CORE);
  return ok;
}

void applySensor(const SensorCfg &s) {
  xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
  g_sensor = s;
  g_reconfigSensor = true;
  xSemaphoreGive(g_cfgMtx);
}

void applyAnalysis(const AnalysisCfg &a) {
  xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
  g_analysis = a;
  g_analysis.fftSize = constrain(a.fftSize, FFT_MIN, FFT_MAX);
  xSemaphoreGive(g_cfgMtx);
  s_avgValidMask = 0;
  s_avgFiltValid = false;
}

void applySim(const sim::Config &s) {
  xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
  g_sim = s;
  g_reconfigSim = true;
  xSemaphoreGive(g_cfgMtx);
  s_avgValidMask = 0;
  s_avgFiltValid = false;
}

void applyFilters(const filtros::Config &f) {
  xSemaphoreTake(g_cfgMtx, portMAX_DELAY);
  g_filter = f;
  g_reconfigFilters = true;
  xSemaphoreGive(g_cfgMtx);
}

SensorCfg       sensorCfg()   { return g_sensor; }
AnalysisCfg     analysisCfg() { return g_analysis; }
filtros::Config filterCfg()   { return g_filter; }
sim::Config     simCfg()      { return g_sim; }
bool            simActive()   { return g_sim.enabled; }

bool withResults(uint32_t lastSeq, const std::function<void(const Results &)> &fn) {
  bool fresh = false;
  xSemaphoreTake(g_resMtx, portMAX_DELAY);
  if (g_res.seq != lastSeq && g_res.bins > 0) {
    fn(g_res);
    fresh = true;
  }
  xSemaphoreGive(g_resMtx);
  return fresh;
}

int copyScope(float *raw, float *filt, int maxPoints, float *dtOut) {
  uint8_t ch = g_analysis.axis;
  uint32_t end = g_write;
  uint32_t span = min((uint32_t)(RING_SAMPLES / 2), end);
  if (span < 8) return 0;

  int step = max(1, (int)(span / maxPoints));
  int pts  = min((uint32_t)maxPoints, span / step);
  uint32_t start = end - (uint32_t)(pts * step);

  for (int i = 0; i < pts; i++) {
    uint32_t idx = (start + (uint32_t)i * step) % RING_SAMPLES;
    raw[i]  = rawValue(g_raw[idx], ch);
    filt[i] = filtValue(g_filt[idx], ch);
  }
  float fs = bmi::imu.odr() > 0 ? bmi::imu.odr() : 1600.0f;
  if (dtOut) *dtOut = step / fs;
  return pts;
}

// Um bloco por canal habilitado, ja decimado para caber na tela. filtered =
// false devolve a leitura CRUA do sensor (contagem x escala, sem filtro);
// true devolve o que saiu da cadeia.
static int copyScopeMulti(float *dst, int maxPoints, uint8_t mask, float *dtOut, bool filtered) {
  uint32_t end = g_write;
  uint32_t span = min((uint32_t)(RING_SAMPLES / 2), end);
  if (span < 8) return 0;

  int step = max(1, (int)(span / maxPoints));
  int pts  = min((uint32_t)maxPoints, span / step);
  uint32_t start = end - (uint32_t)(pts * step);

  float *out = dst;
  for (int ch = 0; ch < CHANNELS; ch++) {
    if (!(mask & (1 << ch))) continue;
    for (int i = 0; i < pts; i++) {
      uint32_t idx = (start + (uint32_t)i * step) % RING_SAMPLES;
      out[i] = filtered ? g_filt[idx][ch] : g_raw[idx][ch] * g_scale[ch];
    }
    out += pts;
  }
  float fs = bmi::imu.odr() > 0 ? bmi::imu.odr() : 1600.0f;
  if (dtOut) *dtOut = step / fs;
  return pts;
}

int copyScopeFiltered(float *dst, int maxPoints, uint8_t mask, float *dtOut) {
  return copyScopeMulti(dst, maxPoints, mask, dtOut, true);
}

int copyScopeRaw(float *dst, int maxPoints, uint8_t mask, float *dtOut) {
  return copyScopeMulti(dst, maxPoints, mask, dtOut, false);
}

size_t csvChunk(uint32_t &row, uint8_t *buf, size_t maxLen) {
  static uint32_t s_start = 0, s_total = 0;
  static float    s_dt = 0;

  if (row == 0) {
    uint32_t end = g_write;
    s_total = min((uint32_t)RING_SAMPLES, end);
    s_start = end - s_total;
    s_dt = 1.0f / (bmi::imu.odr() > 0 ? bmi::imu.odr() : 1600.0f);
  }
  if (row >= s_total) return 0;

  size_t used = 0;
  char line[208];
  while (row < s_total) {
    uint32_t idx = (s_start + row) % RING_SAMPLES;
    const int16_t *r = g_raw[idx];
    const float   *f = g_filt[idx];
    int len = snprintf(line, sizeof(line),
                       "%.6f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f\n",
                       row * s_dt,
                       r[0] * g_scale[0], r[1] * g_scale[1], r[2] * g_scale[2],
                       r[3] * g_scale[3], r[4] * g_scale[4], r[5] * g_scale[5],
                       f[0], f[1], f[2], f[3], f[4], f[5]);
    if (len <= 0 || used + (size_t)len > maxLen) break;
    memcpy(buf + used, line, len);
    used += len;
    row++;
  }
  return used;
}

void setLoad(float load01) {
  g_load = constrain(load01, 0.0f, 1.0f);
  g_chainAcc.setLoad(g_load);
  g_chainGyr.setLoad(g_load);
}
float load() { return g_load; }

void attCalibrate() { att::est.startCalibration(); }
void attZeroYaw()   { att::est.zeroYaw(); }
void attConfigure(const att::Config &c) { att::est.configure(c); }
att::Config attConfig() { return att::est.config(); }

float sampleRate() { return bmi::imu.odr(); }
bool  sensorOk()   { return bmi::imu.present(); }
bool  usingPsram() { return g_psram; }
size_t bufferBytes() { return g_bufBytes; }

}  // namespace vib
