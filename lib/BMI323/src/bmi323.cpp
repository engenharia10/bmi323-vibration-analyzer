#include "bmi323.h"
#include <SPI.h>
#include <Wire.h>

namespace bmi {

BMI323 imu;

static SPISettings spiCfg(10000000UL, MSBFIRST, SPI_MODE0);

// acc_odr: 0x1 = 0.78125 Hz, dobrando a cada codigo ate 0xE = 6.4 kHz
static const float kOdrTable[16] = {
    0.0f,   0.78125f, 1.5625f, 3.125f, 6.25f,   12.5f,   25.0f,   50.0f,
    100.0f, 200.0f,   400.0f,  800.0f, 1600.0f, 3200.0f, 6400.0f, 0.0f};

float odrHz(uint8_t code) { return (code < 16) ? kOdrTable[code] : 0.0f; }

uint8_t odrCodeFor(float hz) {
  uint8_t best = 0x0C;  // 1.6 kHz
  float bestErr = 1e9f;
  for (uint8_t c = 1; c <= 0x0E; c++) {
    float e = fabsf(kOdrTable[c] - hz);
    if (e < bestErr) { bestErr = e; best = c; }
  }
  return best;
}

float rangeLsbPerG(uint8_t code) {
  static const float t[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
  return t[code & 3];
}
float rangeG(uint8_t code) {
  static const float t[4] = {2.0f, 4.0f, 8.0f, 16.0f};
  return t[code & 3];
}

// gyr_range: 0=+/-125 dps ... 4=+/-2000 dps (datasheet p. 90)
float gyrLsbPerDps(uint8_t code) {
  static const float t[5] = {262.144f, 131.072f, 65.536f, 32.768f, 16.4f};
  return t[code > 4 ? 4 : code];
}
float gyrRangeDps(uint8_t code) {
  static const float t[5] = {125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f};
  return t[code > 4 ? 4 : code];
}

// ---------------------------------------------------------------- transporte

void BMI323::busBegin() {
  if (bus_.useSpi) {
    spiCfg = SPISettings(bus_.spiHz, MSBFIRST, SPI_MODE0);
    pinMode(bus_.cs, OUTPUT);
    digitalWrite(bus_.cs, HIGH);
    SPI.begin(bus_.sck, bus_.miso, bus_.mosi, bus_.cs);
  } else {
    Wire.begin(bus_.sda, bus_.scl, bus_.i2cHz);
    Wire.setTimeOut(20);
  }
}

bool BMI323::readBurst(uint8_t reg, uint16_t *dst, size_t words) {
  if (!words) return true;

  if (bus_.useSpi) {
    // 1 byte dummy apos o endereco (datasheet, secao 4).
    static uint8_t rx[2 * 3 * 96];
    const size_t kMaxWords = sizeof(rx) / 2;
    while (words) {
      size_t n = words > kMaxWords ? kMaxWords : words;
      SPI.beginTransaction(spiCfg);
      digitalWrite(bus_.cs, LOW);
      SPI.transfer(reg | 0x80);
      SPI.transfer(0x00);
      SPI.transferBytes(nullptr, rx, n * 2);
      digitalWrite(bus_.cs, HIGH);
      SPI.endTransaction();
      for (size_t i = 0; i < n; i++) {
        dst[i] = (uint16_t)rx[i * 2] | ((uint16_t)rx[i * 2 + 1] << 8);
      }
      dst += n;
      words -= n;
      if (reg != REG_FIFO_DATA) reg += n;  // FIFO_DATA nao auto-incrementa
    }
    return true;
  } else {
    // I2C: 2 bytes dummy antes do payload; Wire limita o tamanho da rajada.
    const size_t kMaxWords = 28;
    while (words) {
      size_t n = words > kMaxWords ? kMaxWords : words;
      Wire.beginTransmission(bus_.i2cAddr);
      Wire.write(reg);
      if (Wire.endTransmission(false) != 0) { busErr_++; return false; }
      size_t got = Wire.requestFrom((int)bus_.i2cAddr, (int)(n * 2 + 2));
      if (got < n * 2 + 2) { busErr_++; return false; }
      Wire.read(); Wire.read();  // dummy
      for (size_t i = 0; i < n; i++) {
        uint8_t lo = Wire.read();
        uint8_t hi = Wire.read();
        dst[i] = (uint16_t)lo | ((uint16_t)hi << 8);
      }
      dst += n;
      words -= n;
      if (reg != REG_FIFO_DATA) reg += n;
    }
    return true;
  }
}

uint16_t BMI323::readReg(uint8_t reg) {
  uint16_t v = 0;
  readBurst(reg, &v, 1);
  return v;
}

void BMI323::writeReg(uint8_t reg, uint16_t val) {
  if (bus_.useSpi) {
    SPI.beginTransaction(spiCfg);
    digitalWrite(bus_.cs, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer((uint8_t)(val & 0xFF));
    SPI.transfer((uint8_t)(val >> 8));
    digitalWrite(bus_.cs, HIGH);
    SPI.endTransaction();
  } else {
    Wire.beginTransmission(bus_.i2cAddr);
    Wire.write(reg);
    Wire.write((uint8_t)(val & 0xFF));
    Wire.write((uint8_t)(val >> 8));
    Wire.endTransmission();
  }
}

// -------------------------------------------------------------------- setup

bool BMI323::begin(const BusConfig &bus) {
  bus_ = bus;
  busBegin();
  delay(5);

  // A primeira leitura dummy comuta a interface de I3C/I2C para SPI.
  if (bus_.useSpi) { (void)readReg(REG_CHIP_ID); delay(2); }

  writeReg(REG_CMD, CMD_SOFTRESET);
  delay(10);

  if (bus_.useSpi) { (void)readReg(REG_CHIP_ID); delay(2); }

  uint16_t id = readReg(REG_CHIP_ID) & 0xFF;

  // No I2C o endereco depende do nivel do pino SDO. Em vez de exigir que o
  // usuario descubra, tenta o outro antes de desistir.
  if (id != CHIP_ID_BMI323 && !bus_.useSpi) {
    const uint8_t alt = (bus_.i2cAddr == 0x68) ? 0x69 : 0x68;
    log_w("BMI323 nao respondeu em 0x%02X, tentando 0x%02X", bus_.i2cAddr, alt);
    bus_.i2cAddr = alt;
    writeReg(REG_CMD, CMD_SOFTRESET);
    delay(10);
    id = readReg(REG_CHIP_ID) & 0xFF;
  }

  if (id != CHIP_ID_BMI323) {
    log_e("BMI323 nao encontrado (chip_id=0x%02X, esperado 0x43)", id);
    ok_ = false;
    return false;
  }

  uint16_t status = readReg(REG_STATUS);
  errReg_ = readReg(REG_ERR_REG);
  if (!(status & 0x0001)) log_w("por_detected nao sinalizado (status=0x%04X)", status);
  if (errReg_ & 0x0001)   log_e("fatal_err ativo (err_reg=0x%04X)", errReg_);

  ok_ = true;
  return true;
}

bool BMI323::configureAccel(uint8_t odrCode, uint8_t rangeCode, uint8_t avgCode, uint8_t bwCode,
                            bool gyroOn, uint8_t gyrRangeCode) {
  if (!ok_) return false;

  // Desliga o FIFO antes de mexer na configuracao (secao 5.7.2).
  writeReg(REG_FIFO_CONF, 0);

  gyroOn_ = gyroOn;
  if (gyroOn_) {
    // Mesmo ODR do acelerometro para os quadros do FIFO ficarem alinhados
    uint16_t gconf = ((uint16_t)MODE_PERF << CONF_MODE_SHIFT) |
                     ((uint16_t)(gyrRangeCode & 0x7) << CONF_RANGE_SHIFT) |
                     ((uint16_t)(odrCode & 0xF) << CONF_ODR_SHIFT);
    writeReg(REG_GYR_CONF, gconf);
    lsbPerDps_ = gyrLsbPerDps(gyrRangeCode);
  } else {
    writeReg(REG_GYR_CONF, 0);
  }

  uint16_t conf = ((uint16_t)MODE_PERF << CONF_MODE_SHIFT) |
                  ((uint16_t)(avgCode & 0x7) << CONF_AVG_SHIFT) |
                  ((uint16_t)(bwCode & 0x1) << CONF_BW_SHIFT) |
                  ((uint16_t)(rangeCode & 0x7) << CONF_RANGE_SHIFT) |
                  ((uint16_t)(odrCode & 0xF) << CONF_ODR_SHIFT);
  writeReg(REG_ACC_CONF, conf);
  delay(5);

  uint16_t rb = readReg(REG_ACC_CONF);
  if (rb != conf) log_w("ACC_CONF escrito=0x%04X lido=0x%04X", conf, rb);

  lsbPerG_ = rangeLsbPerG(rangeCode);
  odr_     = odrHz(odrCode);

  writeReg(REG_FIFO_CONF, FIFO_ACC_EN | (gyroOn_ ? FIFO_GYR_EN : 0));
  fifoFlush();
  dropped_ = 0;
  return true;
}

void BMI323::fifoFlush() { writeReg(REG_FIFO_CTRL, 0x0001); }

uint16_t BMI323::fifoFillWords() { return readReg(REG_FIFO_FILL_LVL) & 0x07FF; }

int BMI323::fifoReadFrames(int16_t *dst, int maxFrames) {
  if (!ok_) return 0;
  const int fw = frameWords();

  // Um quadro NUNCA pode ser partido entre duas transacoes: quando o BMI323 e
  // lido pela metade ele reenvia o quadro inteiro na leitura seguinte
  // (datasheet, secao 5.7.2 "Buffer Frame Reads"). O resultado seria palavras
  // duplicadas, leitura desalinhada e mais quadros do que o sensor produziu.
  //
  // Por isso o tamanho da rajada e sempre um numero inteiro de quadros que
  // cabe em uma transacao do barramento: 28 words no I2C (limite do buffer do
  // Wire) e 288 no SPI (buffer interno do readBurst).
  const int burstWords = bus_.useSpi ? 288 : 28;
  const int chunk = burstWords / fw;

  uint16_t words = fifoFillWords();
  int frames = words / fw;
  if (frames <= 0) return 0;
  if (frames > maxFrames) frames = maxFrames;

  static uint16_t buf[3 * 96];
  int written = 0;
  int remaining = frames;
  while (remaining > 0) {
    int n = remaining > chunk ? chunk : remaining;
    // Leitura falhou: para aqui e devolve so o que ja veio inteiro. Inventar
    // amostras (zeros) envenenaria FFT, RMS e atitude de uma vez so.
    if (!readBurst(REG_FIFO_DATA, buf, n * fw)) break;
    for (int f = 0; f < n; f++) {
      const uint16_t *src = buf + f * fw;
      int16_t x = (int16_t)src[0];
      if (x == DUMMY_ACC_W1 || x == DUMMY_INVALID) { dropped_++; continue; }
      for (int w = 0; w < fw; w++) dst[written * fw + w] = (int16_t)src[w];
      written++;
    }
    remaining -= n;
  }
  return written;
}

void BMI323::scanI2C(int8_t sda, int8_t scl, uint32_t hz, Print &out) {
  Wire.begin(sda, scl, hz);
  out.printf("[i2c] varrendo SDA=%d SCL=%d a %u kHz\n", sda, scl, (unsigned)(hz / 1000));
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      const char *who = "";
      switch (a) {
        case 0x68: case 0x69: who = "  <- BMI323"; break;
        case 0x5D: case 0x14: who = "  <- touch GT911"; break;
        case 0x24:            who = "  <- expansor CH422G"; break;
      }
      out.printf("[i2c]   0x%02X%s\n", a, who);
      found++;
    }
  }
  if (!found) out.println("[i2c]   nada respondeu - confira alimentacao e pull-ups");
}

float BMI323::temperatureC() {
  int16_t raw = (int16_t)readReg(REG_TEMP_DATA);
  return (float)raw / 512.0f + 23.0f;
}

}  // namespace bmi
