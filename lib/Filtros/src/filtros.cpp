#include "filtros.h"
#include <math.h>

namespace filtros {

// ------------------------------------------------------------------ init

void Chain::init(const Config &cfg, float sampleRateHz) {
  cfg_ = cfg;
  fs_  = sampleRateHz > 1.0f ? sampleRateHz : 1600.0f;
  const float dT = 1.0f / fs_;

  dcOn_ = cfg_.dcBlock && cfg_.dcBlockHz > 0.0f;
  lpf1CutoffNow_ = cfg_.lpf1Dyn ? lpfCut(cfg_.lpf1DynMin) : lpfCut(cfg_.lpf1Hz);

  for (int ax = 0; ax < AXES; ax++) {
    // DC-block: PT1 usado como passa-alta (entrada menos a saida do passa-baixa)
    dc_[ax].init(bf::pt1FilterGain(cfg_.dcBlockHz, dT));
    lpf2_[ax].init(cfg_.lpf2Type, lpfCut(cfg_.lpf2Hz), fs_);
    lpf1_[ax].init(cfg_.lpf1Type, lpf1CutoffNow_, fs_, cfg_.lpf1Dyn);
  }

  // Notches estaticos: o Betaflight define o notch por centro + corte inferior
  // e deriva o Q com filterGetNotchQ(). Desligado quando hz = 0 ou quando o
  // corte nao e menor que o centro.
  const float q1 = bf::filterGetNotchQ(cfg_.notch1Hz, cfg_.notch1Cutoff);
  const float q2 = bf::filterGetNotchQ(cfg_.notch2Hz, cfg_.notch2Cutoff);
  notch1On_ = cfg_.notch1Hz > 0.0f && q1 > 0.0f;
  notch2On_ = cfg_.notch2Hz > 0.0f && q2 > 0.0f;
  for (int ax = 0; ax < AXES; ax++) {
    if (notch1On_) notch1_[ax].init(cfg_.notch1Hz, fs_, q1, bf::BQ_NOTCH, 1.0f);
    if (notch2On_) notch2_[ax].init(cfg_.notch2Hz, fs_, q2, bf::BQ_NOTCH, 1.0f);
  }

  rpmInit();
  dynNotchInit();
}

void Chain::rpmInit() {
  rpmNumHarmonics_ = 0;
  for (int h = 0; h < RPM_HARMONICS_MAX; h++) { rpmFreqHz_[h] = 0; rpmWeightOut_[h] = 0; }

  if (cfg_.rpmSource == RPM_OFF || cfg_.rpmHarmonics == 0) return;

  rpmNumHarmonics_ = min<int>(cfg_.rpmHarmonics, RPM_HARMONICS_MAX);
  rpmQf_    = cfg_.rpmQ / 100.0f;
  rpmMaxHz_ = 0.48f * fs_;   // nao chega ate Nyquist, evita oscilacao
  if (cfg_.rpmSource == RPM_MANUAL) rpmFundamentalRaw_ = cfg_.rpmBaseHz;

  // rpm_filter_lpf_hz: PT1 sobre a propria fundamental. Sem ele um degrau na
  // frequencia (troca de pico entre dois quadros da FFT) reescreve os
  // coeficientes de uma vez e o notch salta. Parte ja no valor corrente para
  // nao rampar desde zero a cada re-init.
  rpmFreqLpf_.init(cfg_.rpmLpfHz > 0.0f ? bf::pt1FilterGain(cfg_.rpmLpfHz, 1.0f / fs_) : 1.0f);
  rpmFreqLpf_.state = rpmFundamentalRaw_;
  rpmFundamental_   = rpmFundamentalRaw_;

  for (int ax = 0; ax < AXES; ax++) {
    for (int h = 0; h < rpmNumHarmonics_; h++) {
      // weight 0 = notch inativo ate a primeira atualizacao
      rpmNotch_[ax][h].init(cfg_.rpmMinHz * (h + 1), fs_, rpmQf_, bf::BQ_NOTCH, 0.0f);
    }
  }
  rpmUpdate();
}

void Chain::dynNotchInit() {
  const float looprateHz = fs_;
  const float nyquistHz  = looprateHz * 0.5f;

  state_ = {};
  sampleIndex_ = 0;
  for (int ax = 0; ax < AXES; ax++) sampleAccumulator_[ax] = sampleAvg_[ax] = 0.0f;

  if (looprateHz < DYN_NOTCH_MIN_SAMPLE_RATE || cfg_.dynNotchCount == 0) {
    dynCount_ = 0;
    return;
  }

  dynQ_     = cfg_.dynNotchQ / 100.0f;
  dynMinHz_ = cfg_.dynNotchMinHz;
  dynMaxHz_ = max(dynMinHz_, cfg_.dynNotchMaxHz);
  dynMaxHz_ = min(dynMaxHz_, nyquistHz);
  dynCount_ = min<int>(cfg_.dynNotchCount, DYN_NOTCH_COUNT_MAX);

  // Acumula e faz a media de varias amostras para que a SDFT rode a ~2x maxHz
  sampleCount_    = max(1, (int)(nyquistHz / dynMaxHz_));
  sampleCountRcp_ = 1.0f / sampleCount_;

  sdftSampleRateHz_ = looprateHz / sampleCount_;
  sdftResolutionHz_ = sdftSampleRateHz_ / bf::SDFT_SAMPLE_SIZE;
  sdftStartBin_ = max(1, (int)lrintf(dynMinHz_ / sdftResolutionHz_));
  sdftEndBin_   = min(bf::SDFT_BIN_COUNT - 1, (int)lrintf(dynMaxHz_ / sdftResolutionHz_));
  if (sdftEndBin_ <= sdftStartBin_) { dynCount_ = 0; return; }

  pt1LooptimeS_ = DYN_NOTCH_CALC_TICKS / looprateHz;

  for (int ax = 0; ax < AXES; ax++) sdft_[ax].init(sdftStartBin_, sdftEndBin_, sampleCount_);

  for (int ax = 0; ax < AXES; ax++) {
    for (int p = 0; p < dynCount_; p++) {
      // Espalhar os centros pela faixa faz os notches grudarem nos picos mais rapido
      centerFreq_[ax][p] = (p + 0.5f) * (dynMaxHz_ - dynMinHz_) / (float)dynCount_ + dynMinHz_;
      dynNotch_[ax][p].init(centerFreq_[ax][p], fs_, dynQ_, bf::BQ_NOTCH, 1.0f);
    }
  }
}

// ----------------------------------------------------------------- apply

float Chain::apply(int axis, float input) {
  float v = input;

  // DC-block (fora do Betaflight): passa-alta de 1a ordem
  if (dcOn_) v = v - dc_[axis].apply(v);

  // Ordem do gyro_filter_impl.c
  v = lpf2_[axis].apply(v);

  for (int h = 0; h < rpmNumHarmonics_; h++) {
    v = rpmNotch_[axis][h].applyDF1Weighted(v);   // rpmFilterApply()
  }

  if (notch1On_) v = notch1_[axis].applyDF1(v);
  if (notch2On_) v = notch2_[axis].applyDF1(v);
  v = lpf1_[axis].apply(v);

  if (dynCount_ > 0) {
    sampleAccumulator_[axis] += v;                // dynNotchPush()
    for (int p = 0; p < dynCount_; p++) {
      v = dynNotch_[axis][p].applyDF1(v);         // dynNotchFilter()
    }
  }
  return v;
}

// ---------------------------------------------------------------- update

void Chain::update() {
  rpmUpdate();
  lpf1Update();

  if (dynCount_ <= 0) return;

  if (sampleIndex_ == sampleCount_) {
    sampleIndex_ = 0;
    for (int ax = 0; ax < AXES; ax++) {
      sampleAvg_[ax] = sampleAccumulator_[ax] * sampleCountRcp_;
      sampleAccumulator_[ax] = 0.0f;
    }
    // 12 ticks cobrem 4 passos x 3 eixos
    state_.tick = DYN_NOTCH_CALC_TICKS;
  }

  for (int ax = 0; ax < AXES; ax++) sdft_[ax].pushBatch(sampleAvg_[ax], sampleIndex_);
  sampleIndex_++;

  if (state_.tick > 0) {
    dynNotchProcess();
    --state_.tick;
  }
}

void Chain::setLoad(float load01) { load_ = constrain(load01, 0.0f, 1.0f); }

void Chain::setDetectedFundamental(float hz) {
  if (cfg_.rpmSource == RPM_AUTO_PEAK && hz > 0.0f) rpmFundamentalRaw_ = hz;
}

// LPF1 dinamico: mesma curva de dynLpfCutoffFreq() (src/main/flight/pid.c),
// com o "acelerador" substituido pelo nivel de carga informado pela aplicacao.
void Chain::lpf1Update() {
  if (!cfg_.lpf1Dyn) return;
  // atualizar a cada 32 amostras ja e muito mais rapido que a dinamica do sinal
  if ((++lpf1UpdateTick_ & 31) != 0) return;

  const float expof = cfg_.lpf1DynExpo / 10.0f;
  const float curve = load_ * (1 - load_) * expof + load_;
  const float lo = lpfCut(cfg_.lpf1DynMin), hi = lpfCut(cfg_.lpf1DynMax);
  const float cutoff = (hi - lo) * curve + lo;

  if (fabsf(cutoff - lpf1CutoffNow_) < 0.5f) return;
  lpf1CutoffNow_ = cutoff;

  const float dT = 1.0f / fs_;
  for (int ax = 0; ax < AXES; ax++) {
    switch (cfg_.lpf1Type) {
      case bf::FILTER_PT1: lpf1_[ax].pt1.k = bf::pt1FilterGain(cutoff, dT); break;
      case bf::FILTER_PT2: lpf1_[ax].pt2.k = bf::pt1FilterGain(cutoff * bf::CUTOFF_CORRECTION_PT2, dT); break;
      case bf::FILTER_PT3: lpf1_[ax].pt3.k = bf::pt1FilterGain(cutoff * bf::CUTOFF_CORRECTION_PT3, dT); break;
      case bf::FILTER_BIQUAD: lpf1_[ax].bq.update(cutoff, fs_, bf::BIQUAD_Q, bf::BQ_LPF, 1.0f); break;
    }
  }
}

// Filtro RPM: notches na fundamental e harmonicas, com peso que desaparece
// perto de minHz. Logica identica a de rpmFilterUpdate().
void Chain::rpmUpdate() {
  if (rpmNumHarmonics_ <= 0 || rpmFundamentalRaw_ <= 0.0f) return;

  // Suaviza a fundamental antes de virar coeficiente (rpmFilterUpdate() usa
  // getMotorFrequencyHz(), que no Betaflight ja sai filtrado por este PT1).
  rpmFundamental_ = rpmFreqLpf_.apply(rpmFundamentalRaw_);

  for (int h = 0; h < rpmNumHarmonics_; h++) {
    const float w0 = cfg_.rpmWeights[h] / 100.0f;
    if (w0 <= 0.0f) { rpmWeightOut_[h] = 0.0f; continue; }

    const float freqHz = constrain((h + 1) * rpmFundamental_, cfg_.rpmMinHz, rpmMaxHz_);
    const float marginHz = freqHz - cfg_.rpmMinHz;
    float weight = 1.0f;
    // desliga o notch progressivamente quando ele se aproxima de minHz
    if (cfg_.rpmFadeRangeHz > 0.0f && marginHz < cfg_.rpmFadeRangeHz) {
      weight *= marginHz / cfg_.rpmFadeRangeHz;
    }
    weight *= w0;

    if (fabsf(freqHz - rpmFreqHz_[h]) < 0.05f && fabsf(weight - rpmWeightOut_[h]) < 0.005f) continue;
    rpmFreqHz_[h]    = freqHz;
    rpmWeightOut_[h] = weight;

    // atualiza o notch do eixo 0 e copia os coeficientes para os demais
    bf::Biquad &tmpl = rpmNotch_[0][h];
    tmpl.update(freqHz, fs_, rpmQf_, bf::BQ_NOTCH, weight);
    for (int ax = 1; ax < AXES; ax++) {
      bf::Biquad &d = rpmNotch_[ax][h];
      d.b0 = tmpl.b0; d.b1 = tmpl.b1; d.b2 = tmpl.b2;
      d.a1 = tmpl.a1; d.a2 = tmpl.a2; d.weight = tmpl.weight;
    }
  }
}

void Chain::dynNotchProcess() {
  switch (state_.step) {
    case STEP_WINDOW: {
      sdft_[state_.axis].winSq(sdftData_);
      // Potencia total na faixa, base para estimar o piso de ruido
      sdftNoiseThreshold_ = 0.0f;
      for (int bin = sdftStartBin_; bin <= sdftEndBin_; bin++) sdftNoiseThreshold_ += sdftData_[bin];
      break;
    }

    case STEP_DETECT_PEAKS: {
      for (int p = 0; p < dynCount_; p++) { peaks_[p].bin = 0; peaks_[p].value = 0.0f; }

      // Procura os N maiores picos do espectro
      for (int bin = sdftStartBin_ + 1; bin < sdftEndBin_; bin++) {
        if (sdftData_[bin] > sdftData_[bin - 1] && sdftData_[bin] > sdftData_[bin + 1]) {
          for (int p = 0; p < dynCount_; p++) {
            if (sdftData_[bin] > peaks_[p].value) {
              for (int k = dynCount_ - 1; k > p; k--) peaks_[k] = peaks_[k - 1];
              peaks_[p].bin = bin;
              peaks_[p].value = sdftData_[bin];
              break;
            }
          }
          bin++;  // se este bin e pico, o proximo nao pode ser
        }
      }

      // Reordena por bin crescente, deixando os picos vazios (bin = 0) no fim
      for (int p = dynCount_ - 1; p > 0; p--) {
        for (int k = 0; k < p; k++) {
          if (peaks_[k].bin > peaks_[k + 1].bin && peaks_[k + 1].bin != 0) {
            Peak tmp = peaks_[k];
            peaks_[k] = peaks_[k + 1];
            peaks_[k + 1] = tmp;
          }
        }
      }
      break;
    }

    case STEP_CALC_FREQUENCIES: {
      // Piso de ruido = densidade espectral media da faixa, excluindo os picos
      int peakCount = 0;
      for (int p = 0; p < dynCount_; p++) {
        if (peaks_[p].bin != 0) {
          sdftNoiseThreshold_ -= 0.75f * sdftData_[peaks_[p].bin - 1];
          sdftNoiseThreshold_ -= sdftData_[peaks_[p].bin];
          sdftNoiseThreshold_ -= 0.75f * sdftData_[peaks_[p].bin + 1];
          peakCount++;
        }
      }
      sdftNoiseThreshold_ /= (sdftEndBin_ - sdftStartBin_ - peakCount + 1);
      // Limiar 2x o piso evita o rastreamento seguir ruido
      sdftNoiseThreshold_ *= 2.0f;

      for (int p = 0; p < dynCount_; p++) {
        if (peaks_[p].bin != 0 && peaks_[p].value > sdftNoiseThreshold_) {
          float meanBin = peaks_[p].bin;

          // Parabola sobre y0, y1, y2 -> posicao real do pico
          const float y0 = sdftData_[peaks_[p].bin - 1];
          const float y1 = sdftData_[peaks_[p].bin];
          const float y2 = sdftData_[peaks_[p].bin + 1];
          const float denom = 2.0f * (y0 - 2 * y1 + y2);
          if (denom != 0.0f) meanBin += (y0 - y2) / denom;

          const float centerFreq = constrain(meanBin * sdftResolutionHz_, dynMinHz_, dynMaxHz_);

          // PT1: aproxima rapido de picos fortes, afasta devagar (ate 10x)
          const float cutoffMult = constrain(peaks_[p].value / sdftNoiseThreshold_, 1.0f, 10.0f);
          const float gain = bf::pt1FilterGain(DYN_NOTCH_SMOOTH_HZ * cutoffMult, pt1LooptimeS_);

          centerFreq_[state_.axis][p] += gain * (centerFreq - centerFreq_[state_.axis][p]);
        }
      }
      break;
    }

    case STEP_UPDATE_FILTERS: {
      for (int p = 0; p < dynCount_; p++) {
        if (peaks_[p].bin != 0 && peaks_[p].value > sdftNoiseThreshold_) {
          dynNotch_[state_.axis][p].update(centerFreq_[state_.axis][p], fs_, dynQ_, bf::BQ_NOTCH, 1.0f);
        }
      }
      state_.axis = (state_.axis + 1) % AXES;
      break;
    }
  }

  state_.step = (state_.step + 1) % STEP_COUNT;
}

}  // namespace filtros
