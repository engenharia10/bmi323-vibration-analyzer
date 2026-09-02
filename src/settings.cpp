#include "settings.h"
#include <Preferences.h>

namespace settings {

static Preferences prefs;
static WifiCfg g_wifi;

void begin() {
  prefs.begin("bmivib", false);

  vib::SensorCfg s;
  s.odrCode   = prefs.getUChar("odr", s.odrCode);
  s.rangeCode = prefs.getUChar("range", s.rangeCode);
  s.avgCode   = prefs.getUChar("avg", s.avgCode);
  s.bwCode    = prefs.getUChar("bw", s.bwCode);
  s.gyroOn    = prefs.getBool("gyro", s.gyroOn);
  s.gyrRange  = prefs.getUChar("gyrr", s.gyrRange);

  vib::AnalysisCfg a;
  a.fftSize  = prefs.getInt("fft", a.fftSize);
  a.window   = (dsp::Window)prefs.getUChar("win", (uint8_t)a.window);
  a.avgAlpha = prefs.getFloat("alpha", a.avgAlpha);
  a.axis     = prefs.getUChar("axis", a.axis);
  a.chanMask = prefs.getUChar("chan", a.chanMask);

  filtros::Config f;
  f.dcBlock   = prefs.getBool("dc", f.dcBlock);
  f.dcBlockHz = prefs.getFloat("dcf", f.dcBlockHz);
  f.lpf1Type  = (bf::LowpassType)prefs.getUChar("l1t", (uint8_t)f.lpf1Type);
  f.lpf1Hz    = prefs.getFloat("l1f", f.lpf1Hz);
  f.lpf2Type  = (bf::LowpassType)prefs.getUChar("l2t", (uint8_t)f.lpf2Type);
  f.lpf2Hz    = prefs.getFloat("l2f", f.lpf2Hz);
  f.notch1Hz     = prefs.getFloat("n1h", f.notch1Hz);
  f.notch1Cutoff = prefs.getFloat("n1c", f.notch1Cutoff);
  f.notch2Hz     = prefs.getFloat("n2h", f.notch2Hz);
  f.notch2Cutoff = prefs.getFloat("n2c", f.notch2Cutoff);
  f.dynNotchCount = prefs.getUChar("dnc", f.dynNotchCount);
  f.dynNotchQ     = prefs.getUShort("dnq", f.dynNotchQ);
  f.dynNotchMinHz = prefs.getFloat("dnmin", f.dynNotchMinHz);
  f.dynNotchMaxHz = prefs.getFloat("dnmax", f.dynNotchMaxHz);
  f.lpfMultiplier = prefs.getUShort("mult", f.lpfMultiplier);
  f.lpf1Dyn       = prefs.getBool("l1dyn", f.lpf1Dyn);
  f.lpf1DynMin    = prefs.getFloat("l1dmin", f.lpf1DynMin);
  f.lpf1DynMax    = prefs.getFloat("l1dmax", f.lpf1DynMax);
  f.lpf1DynExpo   = prefs.getUChar("l1dexp", f.lpf1DynExpo);
  f.rpmSource      = (filtros::RpmSource)prefs.getUChar("rsrc", (uint8_t)f.rpmSource);
  f.rpmBaseHz      = prefs.getFloat("rbase", f.rpmBaseHz);
  f.rpmHarmonics   = prefs.getUChar("rharm", f.rpmHarmonics);
  f.rpmQ           = prefs.getUShort("rq", f.rpmQ);
  f.rpmMinHz       = prefs.getFloat("rmin", f.rpmMinHz);
  f.rpmFadeRangeHz = prefs.getFloat("rfade", f.rpmFadeRangeHz);
  f.rpmLpfHz       = prefs.getFloat("rlpf", f.rpmLpfHz);
  for (int h = 0; h < filtros::RPM_HARMONICS_MAX; h++) {
    char k[8];
    snprintf(k, sizeof(k), "rw%d", h);
    f.rpmWeights[h] = prefs.getUChar(k, f.rpmWeights[h]);
  }

  sim::Config sc;
  sc.enabled     = prefs.getBool("simon", sc.enabled);
  sc.noiseMg     = prefs.getFloat("simn", sc.noiseMg);
  sc.gravity     = prefs.getBool("simg", sc.gravity);
  sc.resonanceOn = prefs.getBool("simro", sc.resonanceOn);
  sc.resonanceHz = prefs.getFloat("simrf", sc.resonanceHz);
  sc.resonanceQ  = prefs.getFloat("simrq", sc.resonanceQ);
  sc.resonanceGainDb = prefs.getFloat("simrg", sc.resonanceGainDb);
  sc.attOn       = prefs.getBool("sima", sc.attOn);
  sc.rollAmpDeg  = prefs.getFloat("simar", sc.rollAmpDeg);
  sc.pitchAmpDeg = prefs.getFloat("simap", sc.pitchAmpDeg);
  sc.attPeriodS  = prefs.getFloat("simat", sc.attPeriodS);
  sc.yawRateDps  = prefs.getFloat("simay", sc.yawRateDps);
  {
    char k[10];
    for (int i = 0; i < sim::SOURCES; i++) {
      sim::Source &o = sc.src[i];
      snprintf(k, sizeof(k), "s%de", i);  o.enabled      = prefs.getBool(k, o.enabled);
      snprintf(k, sizeof(k), "s%df", i);  o.freqHz       = prefs.getFloat(k, o.freqHz);
      snprintf(k, sizeof(k), "s%da", i);  o.ampMg        = prefs.getFloat(k, o.ampMg);
      snprintf(k, sizeof(k), "s%dh", i);  o.harmonics    = prefs.getUChar(k, o.harmonics);
      snprintf(k, sizeof(k), "s%dd", i);  o.harmDecayPct = prefs.getUChar(k, o.harmDecayPct);
      snprintf(k, sizeof(k), "s%dr", i);  o.driftHz      = prefs.getFloat(k, o.driftHz);
      snprintf(k, sizeof(k), "s%dp", i);  o.driftPeriodS = prefs.getFloat(k, o.driftPeriodS);
      snprintf(k, sizeof(k), "s%dj", i);  o.jitterHz     = prefs.getFloat(k, o.jitterHz);
      snprintf(k, sizeof(k), "s%dx", i);  o.axis         = prefs.getUChar(k, o.axis);
    }
  }
  vib::applySim(sc);

  prefs.getBytes("ssid", g_wifi.ssid, sizeof(g_wifi.ssid));
  prefs.getBytes("pass", g_wifi.pass, sizeof(g_wifi.pass));
  g_wifi.ssid[32] = 0;
  g_wifi.pass[64] = 0;

  att::Config ac;
  ac.gravLpfHz = prefs.getFloat("atlpf", ac.gravLpfHz);
  ac.kp        = prefs.getFloat("atkp", ac.kp);
  ac.ki        = prefs.getFloat("atki", ac.ki);
  ac.tolG      = prefs.getFloat("attol", ac.tolG);
  ac.source    = prefs.getUChar("atsrc", ac.source);
  vib::attConfigure(ac);

  vib::applySensor(s);
  vib::applyAnalysis(a);
  vib::applyFilters(f);
}

void save() {
  vib::SensorCfg   s = vib::sensorCfg();
  vib::AnalysisCfg a = vib::analysisCfg();
  filtros::Config  f = vib::filterCfg();

  prefs.putUChar("odr", s.odrCode);
  prefs.putUChar("range", s.rangeCode);
  prefs.putUChar("avg", s.avgCode);
  prefs.putUChar("bw", s.bwCode);
  prefs.putBool("gyro", s.gyroOn);
  prefs.putUChar("gyrr", s.gyrRange);

  att::Config ac = vib::attConfig();
  prefs.putFloat("atlpf", ac.gravLpfHz);
  prefs.putFloat("atkp", ac.kp);
  prefs.putFloat("atki", ac.ki);
  prefs.putFloat("attol", ac.tolG);
  prefs.putUChar("atsrc", ac.source);

  prefs.putInt("fft", a.fftSize);
  prefs.putUChar("win", (uint8_t)a.window);
  prefs.putFloat("alpha", a.avgAlpha);
  prefs.putUChar("axis", a.axis);
  prefs.putUChar("chan", a.chanMask);

  prefs.putBool("dc", f.dcBlock);
  prefs.putFloat("dcf", f.dcBlockHz);
  prefs.putUChar("l1t", (uint8_t)f.lpf1Type);
  prefs.putFloat("l1f", f.lpf1Hz);
  prefs.putUChar("l2t", (uint8_t)f.lpf2Type);
  prefs.putFloat("l2f", f.lpf2Hz);
  prefs.putFloat("n1h", f.notch1Hz);
  prefs.putFloat("n1c", f.notch1Cutoff);
  prefs.putFloat("n2h", f.notch2Hz);
  prefs.putFloat("n2c", f.notch2Cutoff);
  prefs.putUChar("dnc", f.dynNotchCount);
  prefs.putUShort("dnq", f.dynNotchQ);
  prefs.putFloat("dnmin", f.dynNotchMinHz);
  prefs.putFloat("dnmax", f.dynNotchMaxHz);
  prefs.putUShort("mult", f.lpfMultiplier);
  prefs.putBool("l1dyn", f.lpf1Dyn);
  prefs.putFloat("l1dmin", f.lpf1DynMin);
  prefs.putFloat("l1dmax", f.lpf1DynMax);
  prefs.putUChar("l1dexp", f.lpf1DynExpo);
  prefs.putUChar("rsrc", (uint8_t)f.rpmSource);
  prefs.putFloat("rbase", f.rpmBaseHz);
  prefs.putUChar("rharm", f.rpmHarmonics);
  prefs.putUShort("rq", f.rpmQ);
  prefs.putFloat("rmin", f.rpmMinHz);
  prefs.putFloat("rfade", f.rpmFadeRangeHz);
  prefs.putFloat("rlpf", f.rpmLpfHz);
  for (int h = 0; h < filtros::RPM_HARMONICS_MAX; h++) {
    char k[8];
    snprintf(k, sizeof(k), "rw%d", h);
    prefs.putUChar(k, f.rpmWeights[h]);
  }

  sim::Config sc = vib::simCfg();
  prefs.putBool("simon", sc.enabled);
  prefs.putFloat("simn", sc.noiseMg);
  prefs.putBool("simg", sc.gravity);
  prefs.putBool("simro", sc.resonanceOn);
  prefs.putFloat("simrf", sc.resonanceHz);
  prefs.putFloat("simrq", sc.resonanceQ);
  prefs.putFloat("simrg", sc.resonanceGainDb);
  prefs.putBool("sima", sc.attOn);
  prefs.putFloat("simar", sc.rollAmpDeg);
  prefs.putFloat("simap", sc.pitchAmpDeg);
  prefs.putFloat("simat", sc.attPeriodS);
  prefs.putFloat("simay", sc.yawRateDps);
  {
    char k[10];
    for (int i = 0; i < sim::SOURCES; i++) {
      const sim::Source &o = sc.src[i];
      snprintf(k, sizeof(k), "s%de", i);  prefs.putBool(k, o.enabled);
      snprintf(k, sizeof(k), "s%df", i);  prefs.putFloat(k, o.freqHz);
      snprintf(k, sizeof(k), "s%da", i);  prefs.putFloat(k, o.ampMg);
      snprintf(k, sizeof(k), "s%dh", i);  prefs.putUChar(k, o.harmonics);
      snprintf(k, sizeof(k), "s%dd", i);  prefs.putUChar(k, o.harmDecayPct);
      snprintf(k, sizeof(k), "s%dr", i);  prefs.putFloat(k, o.driftHz);
      snprintf(k, sizeof(k), "s%dp", i);  prefs.putFloat(k, o.driftPeriodS);
      snprintf(k, sizeof(k), "s%dj", i);  prefs.putFloat(k, o.jitterHz);
      snprintf(k, sizeof(k), "s%dx", i);  prefs.putUChar(k, o.axis);
    }
  }
}

void resetDefaults() {
  prefs.clear();
  vib::applySensor(vib::SensorCfg{});
  vib::applyAnalysis(vib::AnalysisCfg{});
  vib::applyFilters(filtros::Config{});
  vib::applySim(sim::Config{});
}

WifiCfg &wifi() { return g_wifi; }

void saveWifi(const char *ssid, const char *pass) {
  strlcpy(g_wifi.ssid, ssid ? ssid : "", sizeof(g_wifi.ssid));
  strlcpy(g_wifi.pass, pass ? pass : "", sizeof(g_wifi.pass));
  prefs.putBytes("ssid", g_wifi.ssid, sizeof(g_wifi.ssid));
  prefs.putBytes("pass", g_wifi.pass, sizeof(g_wifi.pass));
}

}  // namespace settings
