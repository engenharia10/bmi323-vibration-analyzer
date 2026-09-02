#pragma once
#include <Arduino.h>
#include <functional>
#include "config.h"
#include "dsp.h"
#include <filtros.h>
#include <simulador.h>
#include "attitude.h"

// ---------------------------------------------------------------------------
// Nucleo do analisador: task de aquisicao (FIFO do BMI323) + task de DSP
// (FFT do sinal cru e do sinal filtrado) + buffer circular para osciloscopio
// e exportacao CSV.
// ---------------------------------------------------------------------------

namespace vib {

struct SensorCfg {
  // 0x0B = 800 Hz, 0x0C = 1600 Hz (ver bmi::odrHz)
  uint8_t odrCode   = (BMI_DEFAULT_ODR_HZ >= 1600) ? 0x0C : 0x0B;
  uint8_t rangeCode = 0x03;  // +/-16 g
  uint8_t avgCode   = 0x00;  // sem media
  uint8_t bwCode    = 0x00;  // BW = ODR/2
  bool    gyroOn    = true;  // necessario para pitch/roll/yaw
  uint8_t gyrRange  = 0x02;  // +/-500 graus/s
};

struct AnalysisCfg {
  int         fftSize = 1024;
  dsp::Window window  = dsp::WIN_HANN;
  float       avgAlpha = 0.35f;  // media exponencial do espectro (1 = sem media)
  // canal analisado em detalhe (filtrado, osciloscopio, picos, RMS):
  // 0..5 = aX aY aZ gX gY gZ, 6 = modulo do acelerometro
  uint8_t     axis     = 0;
  uint8_t     chanMask = 0x07;   // canais com espectro calculado (bit 0 = aX)
};

struct Results {
  uint32_t seq = 0;
  int   bins   = 0;
  float binHz  = 0;
  float fs     = 0;
  uint8_t chanMask = 0;                    // canais realmente preenchidos
  float spec[CHANNELS][FFT_MAX / 2];       // espectro cru de cada canal
  float specFilt[FFT_MAX / 2];             // canal selecionado, apos os filtros
  int   nPeaks = 0;
  dsp::Peak peaks[6];
  float rmsRaw = 0, rmsFilt = 0;
  // Leitura direta dos 6 eixos, sempre os 6 (independe da mascara do
  // espectro): nivel medio e RMS AC de uma janela curta do sinal CRU.
  float dcCh[CHANNELS] = {0};              // media (g / graus/s)
  float rmsCh[CHANNELS] = {0};             // RMS AC de cada canal
  // Ultimo quadro que saiu do FIFO, exatamente como o chip devolveu: contagem
  // de 16 bits, sem escala, sem media e sem filtro. E a leitura em si.
  int16_t lsbCh[CHANNELS] = {0};
  // Frequencia de oscilacao dominante de cada eixo. E o numero que diz
  // onde por o notch, entao vale para os 6 mesmo com a curva escondida.
  float domFreq[CHANNELS] = {0};
  float domAmp[CHANNELS] = {0};
  float peakToPeakRaw = 0;
  float tempC = 0;
  float measuredHz = 0;
  uint32_t overruns = 0;
  int   dynCount = 0;                                 // notches dinamicos ativos
  float dynFreq[filtros::DYN_NOTCH_COUNT_MAX] = {0};  // centros rastreados, eixo atual
  int   rpmHarmonics = 0;                             // harmonicas do filtro RPM
  float rpmFreq[filtros::RPM_HARMONICS_MAX] = {0};
  float rpmWeight[filtros::RPM_HARMONICS_MAX] = {0};
  float lpf1Cutoff = 0;                               // corte efetivo do LPF1
  att::State attitude;                                // pitch / roll / yaw
  bool  simulated = false;                            // origem: simulador
  float simFreq[sim::SOURCES] = {0};                  // freq instantanea de cada fonte
};

bool begin();
void applySensor(const SensorCfg &s);
void applyAnalysis(const AnalysisCfg &a);
void applyFilters(const filtros::Config &f);
void applySim(const sim::Config &s);

SensorCfg   sensorCfg();
AnalysisCfg analysisCfg();
filtros::Config filterCfg();
sim::Config     simCfg();
bool            simActive();

// Executa fn com o ultimo resultado, sob o mutex. Evita uma segunda copia de
// ~14 kB so para serializar. Retorna false se nada novo desde lastSeq.
bool withResults(uint32_t lastSeq, const std::function<void(const Results &)> &fn);

// Ultimos SCOPE_POINTS pontos (cru e filtrado) do canal selecionado, ja
// decimados para caber na tela.
int copyScope(float *raw, float *filt, int maxPoints, float *dtOut);

// Sinal FILTRADO dos canais indicados em mask, um bloco de maxPoints por
// canal, na ordem crescente de canal. Retorna quantos pontos por canal.
int copyScopeFiltered(float *dst, int maxPoints, uint8_t mask, float *dtOut);

// Mesma organizacao, mas o sinal CRU: a leitura que saiu do FIFO do sensor,
// so convertida de contagem para g / graus por segundo. Nao passa por filtro.
int copyScopeRaw(float *dst, int maxPoints, uint8_t mask, float *dtOut);

// Gera o buffer circular em CSV, em pedacos. Chame com row = 0 na primeira
// vez; a funcao incrementa row e retorna 0 quando terminou.
size_t csvChunk(uint32_t &row, uint8_t *buf, size_t maxLen);

float sampleRate();
bool  sensorOk();

// Onde os buffers grandes foram parar: true = PSRAM, false = RAM interna.
bool  usingPsram();
size_t bufferBytes();

// Onde os buffers grandes foram parar: true = PSRAM, false = RAM interna.
bool  usingPsram();
size_t bufferBytes();

// Nivel 0..1 que dirige o LPF1 dinamico (equivale ao acelerador no Betaflight)
void  setLoad(float load01);
float load();

void  attCalibrate();   // recalibra o offset do giroscopio (deixe parado)
void  attZeroYaw();
void  attConfigure(const att::Config &c);
att::Config attConfig();

}  // namespace vib
