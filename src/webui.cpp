#include "webui.h"
#include "analyzer.h"
#include <bmi323.h>
#include "settings.h"
#include "config.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

namespace webui {

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");
static String g_ip = "0.0.0.0";
static bool   g_ap = true;

// ------------------------------------------------------- protocolo binario
// Cabecalho de 16 bytes seguido dos vetores float32.
struct __attribute__((packed)) PktHeader {
  uint8_t  magic;    // 0xA5
  uint8_t  type;     // 1 = espectro, 2 = osciloscopio, 3 = onda filtrada, 4 = onda crua
  uint8_t  axis;     // canal em detalhe: 0..5 = aX..gZ, 6 = modulo
  uint8_t  flags;    // espectro: bits 0..5 = canais crus, bit 7 = serie filtrada
  uint16_t count;    // bins ou pontos por serie
  uint16_t series;   // quantidade de series que seguem o cabecalho
  float    stepX;    // Hz por bin (espectro) ou segundos por ponto (scope)
  float    fs;
};

// pior caso: 6 canais crus + 1 filtrado. Vai para a PSRAM quando existe.
static constexpr size_t TX_BUF_SIZE =
    sizeof(PktHeader) + (CHANNELS + 1) * (FFT_MAX / 2) * sizeof(float);
static uint8_t *g_txBuf = nullptr;
static uint32_t g_lastSeq = 0;
static uint32_t g_frames  = 0;

// ---------------------------------------------------------------- helpers

// Origem dos parametros de uma chamada de API: pode ser uma requisicao HTTP ou
// uma query string vinda do enlace serial. Sem isso a validacao de cada campo
// teria que existir duas vezes.
class Params {
 public:
  explicit Params(AsyncWebServerRequest *r) : req_(r) {}
  explicit Params(const String &query) : query_(query) {}

  bool has(const char *k) const {
    if (req_) return req_->hasParam(k);
    return find(k) >= 0;
  }
  String get(const char *k) const {
    if (req_) return req_->hasParam(k) ? req_->getParam(k)->value() : String();
    int i = find(k);
    if (i < 0) return String();
    i += strlen(k) + 1;
    int e = query_.indexOf('&', i);
    return e < 0 ? query_.substring(i) : query_.substring(i, e);
  }

 private:
  // acha "k=" respeitando limites de campo, para "n1h" nao casar dentro de "n1hx"
  int find(const char *k) const {
    const String pat = String(k) + "=";
    int i = query_.indexOf(pat);
    while (i >= 0) {
      if (i == 0 || query_.charAt(i - 1) == '&') return i;
      i = query_.indexOf(pat, i + 1);
    }
    return -1;
  }

  AsyncWebServerRequest *req_ = nullptr;
  String query_;
};

static float paramF(const Params &p, const char *k, float def) {
  return p.has(k) ? p.get(k).toFloat() : def;
}
static int paramI(const Params &p, const char *k, int def) {
  return p.has(k) ? p.get(k).toInt() : def;
}
static bool paramB(const Params &p, const char *k, bool def) {
  if (!p.has(k)) return def;
  String v = p.get(k);
  return v == "1" || v == "true" || v == "on";
}

static uint8_t rangeCodeFromG(int g) {
  switch (g) {
    case 2:  return 0;
    case 4:  return 1;
    case 8:  return 2;
    default: return 3;
  }
}

// Fator para a unidade que a interface espera: mg no acelerometro,
// graus/s no giroscopio.
static inline float chanScale(int ch) { return ch < 3 ? 1000.0f : 1.0f; }

static String configJson() {
  vib::SensorCfg   s = vib::sensorCfg();
  vib::AnalysisCfg a = vib::analysisCfg();
  filtros::Config  f = vib::filterCfg();

  String j = "{";
  j += "\"odr\":" + String(bmi::odrHz(s.odrCode), 2);
  j += ",\"odrCode\":" + String(s.odrCode);
  j += ",\"rangeG\":" + String((int)bmi::rangeG(s.rangeCode));
  j += ",\"avg\":" + String(s.avgCode);
  j += ",\"bw\":" + String(s.bwCode);
  j += ",\"gyro\":" + String(s.gyroOn ? 1 : 0);
  j += ",\"gyrRange\":" + String((int)bmi::gyrRangeDps(s.gyrRange));
  // fatores de conversao do fundo de escala atual: a interface precisa deles
  // para mostrar a contagem crua do chip junto do valor em g / graus por segundo
  j += ",\"lsbG\":" + String(bmi::rangeLsbPerG(s.rangeCode), 1);
  j += ",\"lsbDps\":" + String(bmi::gyrLsbPerDps(s.gyrRange), 2);
  j += ",\"fft\":" + String(a.fftSize);
  j += ",\"window\":" + String((int)a.window);
  j += ",\"alpha\":" + String(a.avgAlpha, 3);
  j += ",\"axis\":" + String(a.axis);
  j += ",\"chan\":" + String(a.chanMask);
  // ---- cadeia estilo Betaflight ----
  j += ",\"dc\":" + String(f.dcBlock ? 1 : 0);
  j += ",\"dcf\":" + String(f.dcBlockHz, 2);
  j += ",\"mult\":" + String(f.lpfMultiplier);
  j += ",\"lpf1Type\":" + String((int)f.lpf1Type);
  j += ",\"lpf1Hz\":" + String(f.lpf1Hz, 1);
  j += ",\"lpf1Dyn\":" + String(f.lpf1Dyn ? 1 : 0);
  j += ",\"lpf1DynMin\":" + String(f.lpf1DynMin, 0);
  j += ",\"lpf1DynMax\":" + String(f.lpf1DynMax, 0);
  j += ",\"lpf1DynExpo\":" + String(f.lpf1DynExpo);
  j += ",\"lpf2Type\":" + String((int)f.lpf2Type);
  j += ",\"lpf2Hz\":" + String(f.lpf2Hz, 1);
  j += ",\"n1h\":" + String(f.notch1Hz, 1);
  j += ",\"n1c\":" + String(f.notch1Cutoff, 1);
  j += ",\"n2h\":" + String(f.notch2Hz, 1);
  j += ",\"n2c\":" + String(f.notch2Cutoff, 1);
  j += ",\"dnCount\":" + String(f.dynNotchCount);
  j += ",\"dnQ\":" + String(f.dynNotchQ);
  j += ",\"dnMin\":" + String(f.dynNotchMinHz, 0);
  j += ",\"dnMax\":" + String(f.dynNotchMaxHz, 0);
  j += ",\"rpmSrc\":" + String((int)f.rpmSource);
  j += ",\"rpmBase\":" + String(f.rpmBaseHz, 1);
  j += ",\"rpmHarm\":" + String(f.rpmHarmonics);
  j += ",\"rpmQ\":" + String(f.rpmQ);
  j += ",\"rpmMin\":" + String(f.rpmMinHz, 0);
  j += ",\"rpmFade\":" + String(f.rpmFadeRangeHz, 0);
  j += ",\"rpmLpf\":" + String(f.rpmLpfHz, 0);
  j += ",\"rpmW\":[" + String(f.rpmWeights[0]) + "," + String(f.rpmWeights[1]) + "," + String(f.rpmWeights[2]) + "]";
  j += ",\"load\":" + String(vib::load(), 3);

  att::Config ac = vib::attConfig();
  j += ",\"attLpf\":" + String(ac.gravLpfHz, 2);
  j += ",\"attKp\":" + String(ac.kp, 2);
  j += ",\"attKi\":" + String(ac.ki, 3);
  j += ",\"attTol\":" + String(ac.tolG, 2);
  j += ",\"attSrc\":" + String((int)ac.source);

  // ---- simulador ----
  sim::Config sc = vib::simCfg();
  j += ",\"simOn\":" + String(sc.enabled ? 1 : 0);
  j += ",\"simNoise\":" + String(sc.noiseMg, 2);
  j += ",\"simGrav\":" + String(sc.gravity ? 1 : 0);
  j += ",\"simResOn\":" + String(sc.resonanceOn ? 1 : 0);
  j += ",\"simResHz\":" + String(sc.resonanceHz, 0);
  j += ",\"simResQ\":" + String(sc.resonanceQ, 1);
  j += ",\"simResG\":" + String(sc.resonanceGainDb, 0);
  j += ",\"simSrc\":[";
  for (int i = 0; i < sim::SOURCES; i++) {
    const sim::Source &o = sc.src[i];
    if (i) j += ",";
    j += "{\"en\":" + String(o.enabled ? 1 : 0);
    j += ",\"f\":" + String(o.freqHz, 2);
    j += ",\"a\":" + String(o.ampMg, 1);
    j += ",\"h\":" + String(o.harmonics);
    j += ",\"d\":" + String(o.harmDecayPct);
    j += ",\"dr\":" + String(o.driftHz, 1);
    j += ",\"dp\":" + String(o.driftPeriodS, 1);
    j += ",\"jt\":" + String(o.jitterHz, 2);
    j += ",\"ax\":" + String(o.axis) + "}";
  }
  j += "]";
  j += ",\"simAtt\":" + String(sc.attOn ? 1 : 0);
  j += ",\"simRoll\":" + String(sc.rollAmpDeg, 1);
  j += ",\"simPitch\":" + String(sc.pitchAmpDeg, 1);
  j += ",\"simAttP\":" + String(sc.attPeriodS, 1);
  j += ",\"simYaw\":" + String(sc.yawRateDps, 1);
  j += "}";
  return j;
}

static String statusJson(const vib::Results &r) {
  String j = "{\"t\":\"status\"";
  j += ",\"sensor\":" + String(vib::sensorOk() ? 1 : 0);
  j += ",\"fs\":" + String(r.fs, 1);
  j += ",\"measured\":" + String(r.measuredHz, 1);
  j += ",\"rmsRaw\":" + String(r.rmsRaw * 1000.0f, 2);
  j += ",\"rmsFilt\":" + String(r.rmsFilt * 1000.0f, 2);
  j += ",\"pkpk\":" + String(r.peakToPeakRaw * 1000.0f, 1);
  j += ",\"temp\":" + String(r.tempC, 1);
  j += ",\"heap\":" + String((uint32_t)ESP.getFreeHeap());
  j += ",\"psram\":" + String(vib::usingPsram() ? 1 : 0);
  j += ",\"psfree\":" + String((uint32_t)ESP.getFreePsram());
  j += ",\"drops\":" + String(bmi::imu.droppedFrames());
  j += ",\"dyn\":[";
  for (int i = 0; i < r.dynCount; i++) {
    if (i) j += ",";
    j += String(r.dynFreq[i], 1);
  }
  j += "]";
  j += ",\"rpm\":[";
  for (int i = 0; i < r.rpmHarmonics; i++) {
    if (i) j += ",";
    j += "{\"f\":" + String(r.rpmFreq[i], 1) + ",\"w\":" + String(r.rpmWeight[i], 2) + "}";
  }
  j += "]";
  j += ",\"lpf1Cut\":" + String(r.lpf1Cutoff, 1);
  j += ",\"domCh\":[";
  for (int ch = 0; ch < CHANNELS; ch++) {
    if (ch) j += ",";
    j += "{\"f\":" + String(r.domFreq[ch], 2) +
         ",\"a\":" + String(r.domAmp[ch] * chanScale(ch), 3) + "}";
  }
  j += "]";
  j += ",\"dcCh\":[";
  for (int ch = 0; ch < CHANNELS; ch++) {
    if (ch) j += ",";
    j += String(r.dcCh[ch], ch < 3 ? 4 : 2);
  }
  j += "]";
  j += ",\"rmsCh\":[";
  for (int ch = 0; ch < CHANNELS; ch++) {
    if (ch) j += ",";
    j += String(r.rmsCh[ch] * (ch < 3 ? 1000.0f : 1.0f), 2);
  }
  j += "]";
  // leitura direta: contagem de 16 bits do ultimo quadro do FIFO
  j += ",\"lsbCh\":[";
  for (int ch = 0; ch < CHANNELS; ch++) {
    if (ch) j += ",";
    j += String(r.lsbCh[ch]);
  }
  j += "]";
  j += ",\"att\":{\"r\":" + String(r.attitude.roll, 2);
  j += ",\"p\":" + String(r.attitude.pitch, 2);
  j += ",\"y\":" + String(r.attitude.yaw, 2);
  j += ",\"gx\":" + String(r.attitude.gx, 1);
  j += ",\"gy\":" + String(r.attitude.gy, 1);
  j += ",\"gz\":" + String(r.attitude.gz, 1);
  j += ",\"g\":" + String(r.attitude.accMag, 3);
  j += ",\"gf\":" + String(r.attitude.accMagF, 3);
  j += ",\"trust\":" + String(r.attitude.trust, 3);
  j += ",\"vib\":" + String(r.attitude.vibG * 1000.0f, 1);
  j += ",\"gv\":[" + String(r.attitude.grav[0], 4) + "," +
       String(r.attitude.grav[1], 4) + "," + String(r.attitude.grav[2], 4) + "]";
  j += ",\"bias\":[" + String(r.attitude.bias[0], 3) + "," +
       String(r.attitude.bias[1], 3) + "," + String(r.attitude.bias[2], 3) + "]";
  j += ",\"af\":" + String(r.attitude.accFiltered ? 1 : 0);
  j += ",\"gftd\":" + String(r.attitude.gyrFiltered ? 1 : 0);
  j += ",\"cal\":" + String(r.attitude.calibrating ? 1 : 0);
  j += ",\"ok\":" + String(r.attitude.valid ? 1 : 0) + "}";
  j += ",\"sim\":" + String(r.simulated ? 1 : 0);
  if (r.simulated) {
    j += ",\"simF\":[";
    for (int i = 0; i < sim::SOURCES; i++) {
      if (i) j += ",";
      j += String(r.simFreq[i], 1);
    }
    j += "]";
  }
  j += ",\"peaks\":[";
  for (int i = 0; i < r.nPeaks; i++) {
    if (i) j += ",";
    j += "{\"f\":" + String(r.peaks[i].freq, 2) + ",\"a\":" + String(r.peaks[i].amp * 1000.0f, 3) + "}";
  }
  j += "]}";
  return j;
}

// ---------------------------------------------------------------- envio WS

// Monta o pacote de espectro no buffer. Usado pelo WebSocket e pelo enlace
// serial - o formato binario e exatamente o mesmo nos dois.
size_t buildSpectrumPacket(uint8_t *buf, size_t cap, uint32_t &lastSeq, String *statusOut) {
  size_t len = 0;
  vib::withResults(lastSeq, [&](const vib::Results &r) {
    lastSeq = r.seq;
    const int bins = r.bins;
    const uint8_t mask = r.chanMask;
    const uint8_t axis = vib::analysisCfg().axis;

    int nSeries = 0;
    for (int ch = 0; ch < CHANNELS; ch++) if (mask & (1 << ch)) nSeries++;
    nSeries++;   // a serie filtrada do canal em detalhe vai sempre junto

    const size_t need = sizeof(PktHeader) + (size_t)nSeries * bins * sizeof(float);
    if (need > cap) return;

    PktHeader *h = (PktHeader *)buf;
    h->magic  = 0xA5;
    h->type   = 1;
    h->axis   = axis;
    h->flags  = mask | 0x80;
    h->count  = bins;
    h->series = nSeries;
    h->stepX  = r.binHz;
    h->fs     = r.fs;

    float *p = (float *)(buf + sizeof(PktHeader));
    for (int ch = 0; ch < CHANNELS; ch++) {
      if (!(mask & (1 << ch))) continue;
      const float k = chanScale(ch);
      for (int i = 0; i < bins; i++) *p++ = r.spec[ch][i] * k;
    }
    const float kf = chanScale(axis < CHANNELS ? axis : 0);
    for (int i = 0; i < bins; i++) *p++ = r.specFilt[i] * kf;

    len = need;
    if (statusOut) *statusOut = statusJson(r);
  });
  return len;
}

static void pushSpectrum() {
  if (ws.count() == 0 || !g_txBuf) return;
  String status;
  size_t len = buildSpectrumPacket(g_txBuf, TX_BUF_SIZE, g_lastSeq, &status);
  if (!len) return;
  ws.binaryAll(g_txBuf, len);
  ws.textAll(status);
  g_frames++;
}

size_t buildScopePacket(uint8_t *buf, size_t cap) {
  static float raw[SCOPE_POINTS];
  static float filt[SCOPE_POINTS];
  float dt = 0;
  int pts = vib::copyScope(raw, filt, SCOPE_POINTS, &dt);
  if (pts <= 0) return 0;
  const size_t need = sizeof(PktHeader) + 2u * pts * sizeof(float);
  if (need > cap) return 0;

  PktHeader *h = (PktHeader *)buf;
  h->magic  = 0xA5;
  h->type   = 2;
  h->axis   = vib::analysisCfg().axis;
  h->flags  = 0;
  h->count  = pts;
  h->series = 2;
  h->stepX  = dt;
  h->fs     = vib::sampleRate();

  const float k = chanScale(h->axis < CHANNELS ? h->axis : 0);
  float *p = (float *)(buf + sizeof(PktHeader));
  for (int i = 0; i < pts; i++) p[i] = raw[i] * k;
  for (int i = 0; i < pts; i++) p[pts + i] = filt[i] * k;
  return need;
}

static void pushScope() {
  if (ws.count() == 0 || !g_txBuf) return;
  size_t len = buildScopePacket(g_txBuf, TX_BUF_SIZE);
  if (len) ws.binaryAll(g_txBuf, len);
}

// Um bloco por canal habilitado, sinal CRU (type 4) ou FILTRADO (type 3). A
// interface desenha cada canal na sua propria faixa, entao mg e graus/s
// convivem sem briga de escala.
size_t buildMultiScopePacket(uint8_t *buf, size_t cap, bool filtered) {
  const uint8_t mask = vib::analysisCfg().chanMask;
  if (!mask) return 0;

  static float tmp[CHANNELS * SCOPE_POINTS];
  float dt = 0;
  int pts = filtered ? vib::copyScopeFiltered(tmp, SCOPE_POINTS, mask, &dt)
                     : vib::copyScopeRaw(tmp, SCOPE_POINTS, mask, &dt);
  if (pts <= 0) return 0;

  int nSeries = 0;
  for (int ch = 0; ch < CHANNELS; ch++) if (mask & (1 << ch)) nSeries++;
  const size_t need = sizeof(PktHeader) + (size_t)nSeries * pts * sizeof(float);
  if (need > cap) return 0;

  PktHeader *h = (PktHeader *)buf;
  h->magic  = 0xA5;
  h->type   = filtered ? 3 : 4;
  h->axis   = vib::analysisCfg().axis;
  h->flags  = mask;
  h->count  = pts;
  h->series = nSeries;
  h->stepX  = dt;
  h->fs     = vib::sampleRate();

  float *p = (float *)(buf + sizeof(PktHeader));
  const float *src = tmp;
  for (int ch = 0; ch < CHANNELS; ch++) {
    if (!(mask & (1 << ch))) continue;
    const float k = chanScale(ch);
    for (int i = 0; i < pts; i++) *p++ = src[i] * k;
    src += pts;
  }
  return need;
}

static void pushMultiScope(bool filtered) {
  if (ws.count() == 0 || !g_txBuf) return;
  size_t len = buildMultiScopePacket(g_txBuf, TX_BUF_SIZE, filtered);
  if (len) ws.binaryAll(g_txBuf, len);
}

// ---------------------------------------------------------- API compartilhada
// Mesmas funcoes servem o HTTP e o enlace serial.

static String apiSensor(const Params &p) {
    vib::SensorCfg s = vib::sensorCfg();
    if (p.has("odr"))   s.odrCode   = bmi::odrCodeFor(paramF(p, "odr", 1600));
    if (p.has("range")) s.rangeCode = rangeCodeFromG(paramI(p, "range", 16));
    if (p.has("avg"))   s.avgCode   = constrain(paramI(p, "avg", 0), 0, 6);
    if (p.has("bw"))    s.bwCode    = paramI(p, "bw", 0) ? 1 : 0;
    if (p.has("gyro"))  s.gyroOn    = paramB(p, "gyro", s.gyroOn);
    if (p.has("gyrRange")) {
      int dps = paramI(p, "gyrRange", 500);
      s.gyrRange = dps <= 125 ? 0 : dps <= 250 ? 1 : dps <= 500 ? 2 : dps <= 1000 ? 3 : 4;
    }
    vib::applySensor(s);
  return configJson();
}

static String apiAnalysis(const Params &p) {
    vib::AnalysisCfg a = vib::analysisCfg();
    if (p.has("fft")) {
      int n = paramI(p, "fft", a.fftSize);
      int p = FFT_MIN;
      while (p < n && p < FFT_MAX) p <<= 1;
      a.fftSize = p;
    }
    if (p.has("window")) a.window = (dsp::Window)constrain(paramI(p, "window", 1), 0, 4);
    if (p.has("alpha"))  a.avgAlpha = constrain(paramF(p, "alpha", 0.35f), 0.05f, 1.0f);
    if (p.has("axis"))   a.axis = constrain(paramI(p, "axis", 0), 0, CH_ACC_MAG);
    if (p.has("chan"))   a.chanMask = constrain(paramI(p, "chan", 0x07), 0, 0x3F);
    vib::applyAnalysis(a);
  return configJson();
}

static String apiFilters(const Params &p) {
    filtros::Config f = vib::filterCfg();
    const float nyq = vib::sampleRate() * 0.5f;

    f.dcBlock       = paramB(p, "dc", f.dcBlock);
    f.dcBlockHz     = constrain(paramF(p, "dcf", f.dcBlockHz), 0.2f, 50.0f);
    f.lpfMultiplier = constrain(paramI(p, "mult", f.lpfMultiplier), 25, 200);

    f.lpf1Type    = (bf::LowpassType)constrain(paramI(p, "lpf1Type", (int)f.lpf1Type), 0, 3);
    f.lpf1Hz      = constrain(paramF(p, "lpf1Hz", f.lpf1Hz), 0.0f, nyq);
    f.lpf1Dyn     = paramB(p, "lpf1Dyn", f.lpf1Dyn);
    f.lpf1DynMin  = constrain(paramF(p, "lpf1DynMin", f.lpf1DynMin), 0.0f, nyq);
    f.lpf1DynMax  = constrain(paramF(p, "lpf1DynMax", f.lpf1DynMax), 0.0f, nyq);
    f.lpf1DynExpo = constrain(paramI(p, "lpf1DynExpo", f.lpf1DynExpo), 0, 10);
    f.lpf2Type    = (bf::LowpassType)constrain(paramI(p, "lpf2Type", (int)f.lpf2Type), 0, 3);
    f.lpf2Hz      = constrain(paramF(p, "lpf2Hz", f.lpf2Hz), 0.0f, nyq);

    f.notch1Hz     = constrain(paramF(p, "n1h", f.notch1Hz), 0.0f, nyq);
    f.notch1Cutoff = constrain(paramF(p, "n1c", f.notch1Cutoff), 0.0f, nyq);
    f.notch2Hz     = constrain(paramF(p, "n2h", f.notch2Hz), 0.0f, nyq);
    f.notch2Cutoff = constrain(paramF(p, "n2c", f.notch2Cutoff), 0.0f, nyq);

    f.dynNotchCount = constrain(paramI(p, "dnCount", f.dynNotchCount), 0, filtros::DYN_NOTCH_COUNT_MAX);
    f.dynNotchQ     = constrain(paramI(p, "dnQ", f.dynNotchQ), 1, 1000);
    f.dynNotchMinHz = constrain(paramF(p, "dnMin", f.dynNotchMinHz), 20.0f, 250.0f);
    f.dynNotchMaxHz = constrain(paramF(p, "dnMax", f.dynNotchMaxHz), 100.0f, nyq);

    f.rpmSource      = (filtros::RpmSource)constrain(paramI(p, "rpmSrc", (int)f.rpmSource), 0, 2);
    f.rpmBaseHz      = constrain(paramF(p, "rpmBase", f.rpmBaseHz), 1.0f, nyq);
    f.rpmHarmonics   = constrain(paramI(p, "rpmHarm", f.rpmHarmonics), 0, filtros::RPM_HARMONICS_MAX);
    f.rpmQ           = constrain(paramI(p, "rpmQ", f.rpmQ), 250, 3000);
    f.rpmMinHz       = constrain(paramF(p, "rpmMin", f.rpmMinHz), 5.0f, 200.0f);
    f.rpmFadeRangeHz = constrain(paramF(p, "rpmFade", f.rpmFadeRangeHz), 0.0f, 1000.0f);
    f.rpmLpfHz       = constrain(paramF(p, "rpmLpf", f.rpmLpfHz), 0.0f, 500.0f);
    for (int h = 0; h < filtros::RPM_HARMONICS_MAX; h++) {
      char k[8];
      snprintf(k, sizeof(k), "rpmW%d", h);
      f.rpmWeights[h] = constrain(paramI(p, k, f.rpmWeights[h]), 0, 100);
    }

    if (p.has("load")) vib::setLoad(paramF(p, "load", 0.0f));

    vib::applyFilters(f);
  return configJson();
}

static String apiSim(const Params &p) {
    sim::Config sc = vib::simCfg();
    const float nyq = vib::sampleRate() * 0.5f;

    sc.enabled     = paramB(p, "simOn", sc.enabled);
    sc.noiseMg     = constrain(paramF(p, "simNoise", sc.noiseMg), 0.0f, 200.0f);
    sc.gravity     = paramB(p, "simGrav", sc.gravity);
    sc.resonanceOn = paramB(p, "simResOn", sc.resonanceOn);
    sc.resonanceHz = constrain(paramF(p, "simResHz", sc.resonanceHz), 5.0f, nyq);
    sc.resonanceQ  = constrain(paramF(p, "simResQ", sc.resonanceQ), 0.5f, 40.0f);
    sc.resonanceGainDb = constrain(paramF(p, "simResG", sc.resonanceGainDb), 0.0f, 30.0f);

    char k[10];
    for (int i = 0; i < sim::SOURCES; i++) {
      sim::Source &o = sc.src[i];
      snprintf(k, sizeof(k), "s%den", i);  o.enabled      = paramB(p, k, o.enabled);
      snprintf(k, sizeof(k), "s%df", i);   o.freqHz       = constrain(paramF(p, k, o.freqHz), 0.5f, nyq);
      snprintf(k, sizeof(k), "s%da", i);   o.ampMg        = constrain(paramF(p, k, o.ampMg), 0.0f, 8000.0f);
      snprintf(k, sizeof(k), "s%dh", i);   o.harmonics    = constrain(paramI(p, k, o.harmonics), 1, sim::HARMONICS_MAX);
      snprintf(k, sizeof(k), "s%dd", i);   o.harmDecayPct = constrain(paramI(p, k, o.harmDecayPct), 0, 100);
      snprintf(k, sizeof(k), "s%ddr", i);  o.driftHz      = constrain(paramF(p, k, o.driftHz), 0.0f, 200.0f);
      snprintf(k, sizeof(k), "s%ddp", i);  o.driftPeriodS = constrain(paramF(p, k, o.driftPeriodS), 0.5f, 120.0f);
      snprintf(k, sizeof(k), "s%djt", i);  o.jitterHz     = constrain(paramF(p, k, o.jitterHz), 0.0f, 50.0f);
      snprintf(k, sizeof(k), "s%dax", i);  o.axis         = constrain(paramI(p, k, o.axis), 0, 3);
    }

    sc.attOn       = paramB(p, "simAtt", sc.attOn);
    sc.rollAmpDeg  = constrain(paramF(p, "simRoll", sc.rollAmpDeg), 0.0f, 80.0f);
    sc.pitchAmpDeg = constrain(paramF(p, "simPitch", sc.pitchAmpDeg), 0.0f, 80.0f);
    sc.attPeriodS  = constrain(paramF(p, "simAttP", sc.attPeriodS), 0.5f, 60.0f);
    sc.yawRateDps  = constrain(paramF(p, "simYaw", sc.yawRateDps), -360.0f, 360.0f);

    vib::applySim(sc);
  return configJson();
}

static String apiAttitude(const Params &p) {
    if (paramI(p, "cal", 0))  vib::attCalibrate();
    if (paramI(p, "zero", 0)) vib::attZeroYaw();

    att::Config c = vib::attConfig();
    c.gravLpfHz = paramF(p, "attLpf", c.gravLpfHz);
    c.kp        = paramF(p, "attKp", c.kp);
    c.ki        = paramF(p, "attKi", c.ki);
    c.tolG      = paramF(p, "attTol", c.tolG);
    c.source    = constrain(paramI(p, "attSrc", (int)c.source), 0, 2);
    vib::attConfigure(c);
  return configJson();
}

String handleApi(const String &path, const String &query) {
  Params p(query);
  if (path == "sensor")   return apiSensor(p);
  if (path == "analysis") return apiAnalysis(p);
  if (path == "filters")  return apiFilters(p);
  if (path == "sim")      return apiSim(p);
  if (path == "attitude") return apiAttitude(p);
  if (path == "save")     { settings::save(); return "{\"ok\":1}"; }
  if (path == "defaults") { settings::resetDefaults(); return configJson(); }
  return configJson();
}

String apiConfigJson() { return configJson(); }
String apiStatusJson(const vib::Results &r) { return statusJson(r); }

// ------------------------------------------------------------------- rotas

static void setupRoutes() {
  server.addHandler(&ws);

  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send(200, "application/json", configJson());
  });

  server.on("/api/sensor", HTTP_GET, [](AsyncWebServerRequest *r) {
    Params p(r);
    r->send(200, "application/json", apiSensor(p));
  });

  server.on("/api/analysis", HTTP_GET, [](AsyncWebServerRequest *r) {
    Params p(r);
    r->send(200, "application/json", apiAnalysis(p));
  });

  server.on("/api/filters", HTTP_GET, [](AsyncWebServerRequest *r) {
    Params p(r);
    r->send(200, "application/json", apiFilters(p));
  });

  server.on("/api/sim", HTTP_GET, [](AsyncWebServerRequest *r) {
    Params p(r);
    r->send(200, "application/json", apiSim(p));
  });

  server.on("/api/attitude", HTTP_GET, [](AsyncWebServerRequest *r) {
    Params p(r);
    r->send(200, "application/json", apiAttitude(p));
  });

  server.on("/api/save", HTTP_GET, [](AsyncWebServerRequest *r) {
    settings::save();
    r->send(200, "application/json", "{\"ok\":1}");
  });

  server.on("/api/defaults", HTTP_GET, [](AsyncWebServerRequest *r) {
    settings::resetDefaults();
    r->send(200, "application/json", configJson());
  });

  server.on("/api/wifi", HTTP_GET, [](AsyncWebServerRequest *r) {
    String ssid = r->hasParam("ssid") ? r->getParam("ssid")->value() : "";
    String pass = r->hasParam("pass") ? r->getParam("pass")->value() : "";
    settings::saveWifi(ssid.c_str(), pass.c_str());
    r->send(200, "application/json", "{\"ok\":1,\"reboot\":1}");
    delay(200);
    ESP.restart();
  });

  // Download do buffer circular inteiro em CSV, gerado por streaming.
  server.on("/api/csv", HTTP_GET, [](AsyncWebServerRequest *r) {
    auto row = std::make_shared<uint32_t>(0);
    auto header = std::make_shared<bool>(false);

    AsyncWebServerResponse *resp = r->beginChunkedResponse(
        "text/csv", [row, header](uint8_t *buf, size_t maxLen, size_t) -> size_t {
          size_t used = 0;
          if (!*header) {
            const char *h = "t_s,ax_g,ay_g,az_g,fx_g,fy_g,fz_g\n";
            used = strlen(h);
            memcpy(buf, h, used);
            *header = true;
          }
          used += vib::csvChunk(*row, buf + used, maxLen - used);
          return used;
        });
    resp->addHeader("Content-Disposition", "attachment; filename=bmi323_vib.csv");
    r->send(resp);
  });

  // no-cache: o navegador revalida a cada carga. Sem isso, gravar o sistema de
  // arquivos nao muda nada na tela - o Chrome continua servindo o app.js e o
  // index.html velhos, e a placa parece nao ter sido gravada.
  server.serveStatic("/", LittleFS, "/")
      .setDefaultFile("index.html")
      .setCacheControl("no-cache");

  server.onNotFound([](AsyncWebServerRequest *r) {
    r->send(404, "text/plain", "nao encontrado");
  });
}

// -------------------------------------------------------------------- WiFi

static void connectWifi() {
  settings::WifiCfg &w = settings::wifi();
  if (strlen(w.ssid) > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(w.ssid, w.pass);
    Serial.printf("[wifi] conectando em %s", w.ssid);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_STA_TIMEOUT_MS) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      g_ip = WiFi.localIP().toString();
      g_ap = false;
      Serial.printf("[wifi] STA ok: http://%s/\n", g_ip.c_str());
      return;
    }
    Serial.println("[wifi] falhou, subindo access point");
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASS);
  g_ip = WiFi.softAPIP().toString();
  g_ap = true;
  Serial.printf("[wifi] AP \"%s\" / senha \"%s\" -> http://%s/\n", AP_SSID, AP_PASS, g_ip.c_str());
}

// --------------------------------------------------------------------- API

void begin() {
  if (psramFound()) g_txBuf = (uint8_t *)heap_caps_malloc(TX_BUF_SIZE, MALLOC_CAP_SPIRAM);
  if (!g_txBuf) g_txBuf = (uint8_t *)malloc(TX_BUF_SIZE);

  connectWifi();

  // O wrapper do Arduino procura a particao pelo label "spiffs". Tenta tambem
  // o label "littlefs", para nao depender do nome escolhido na tabela.
  if (!LittleFS.begin(true) && !LittleFS.begin(true, "/littlefs", 10, "littlefs")) {
    Serial.println("[fs] LittleFS nao montou: confira a particao de dados");
  } else {
    Serial.printf("[fs] LittleFS ok - %u de %u kB usados\n",
                  (unsigned)(LittleFS.usedBytes() / 1024),
                  (unsigned)(LittleFS.totalBytes() / 1024));
  }
  if (MDNS.begin(MDNS_HOST)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[mdns] http://%s.local/\n", MDNS_HOST);
  }

  ws.onEvent([](AsyncWebSocket *, AsyncWebSocketClient *c, AwsEventType type,
                void *, uint8_t *, size_t) {
    if (type == WS_EVT_CONNECT) {
      Serial.printf("[ws] cliente %u conectou de %s (%u no total)\n",
                    c->id(), c->remoteIP().toString().c_str(), ws.count());
      c->text(String("{\"t\":\"config\",\"cfg\":") + configJson() + "}");
    } else if (type == WS_EVT_DISCONNECT) {
      Serial.printf("[ws] cliente saiu (%u restantes)\n", ws.count());
    }
  });

  setupRoutes();
  server.begin();
  Serial.println("[http] servidor no ar");
}

void loop() {
  static uint32_t tSpec = 0, tScope = 0, tFilt = 0, tRaw = 0, tClean = 0;
  uint32_t now = millis();

  if (now - tSpec >= SPECTRUM_PERIOD_MS)  { tSpec = now; pushSpectrum(); }
  if (now - tScope >= SCOPE_PERIOD_MS)    { tScope = now; pushScope(); }
  // a leitura crua e o painel principal, entao vai na taxa cheia do scope
  if (now - tRaw >= SCOPE_PERIOD_MS)      { tRaw = now; pushMultiScope(false); }
  // metade da taxa: sao ate 6 series de uma vez
  if (now - tFilt >= SCOPE_PERIOD_MS * 2) { tFilt = now; pushMultiScope(true); }
  if (now - tClean >= 2000)              { tClean = now; ws.cleanupClients(); }
}

uint8_t *txBuffer() { return g_txBuf; }
size_t   txBufferSize() { return TX_BUF_SIZE; }

const char *ipString() { return g_ip.c_str(); }
bool isAccessPoint()   { return g_ap; }
int      wsClients()   { return ws.count(); }
uint32_t framesSent()  { return g_frames; }

}  // namespace webui
