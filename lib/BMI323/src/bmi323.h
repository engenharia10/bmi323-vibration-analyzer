#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Driver do Bosch BMI323 (registradores de 16 bits, enderecos de 8 bits).
// Referencias: datasheet BST-BMI323-DS000, secoes 4 (quick start), 5.7 (FIFO)
// e 6.1 (mapa de registradores).
//
// Barramento escolhido em tempo de execucao pela BusConfig, entao a lib nao
// depende de nenhum header do projeto.
//
//   bmi::BusConfig bus;
//   bus.useSpi = true; bus.sck = 12; bus.miso = 13; bus.mosi = 11; bus.cs = 10;
//   bmi::imu.begin(bus);
//   bmi::imu.configureAccel(0x0C, 0x03, 0, 0);   // 1600 Hz, +/-16 g
// ---------------------------------------------------------------------------

namespace bmi {

// Mapa de registradores (Table 36)
enum : uint8_t {
  REG_CHIP_ID        = 0x00,
  REG_ERR_REG        = 0x01,
  REG_STATUS         = 0x02,
  REG_ACC_DATA_X     = 0x03,
  REG_GYR_DATA_X     = 0x06,
  REG_TEMP_DATA      = 0x09,
  REG_SENSOR_TIME_0  = 0x0A,
  REG_SAT_FLAGS      = 0x0C,
  REG_FIFO_FILL_LVL  = 0x15,
  REG_FIFO_DATA      = 0x16,
  REG_ACC_CONF       = 0x20,
  REG_GYR_CONF       = 0x21,
  REG_FIFO_WATERMARK = 0x35,
  REG_FIFO_CONF      = 0x36,
  REG_FIFO_CTRL      = 0x37,
  REG_CMD            = 0x7E,
};

constexpr uint8_t  CHIP_ID_BMI323 = 0x43;
constexpr uint16_t CMD_SOFTRESET  = 0xDEAF;

// Campos de ACC_CONF / GYR_CONF (datasheet p. 88-90)
constexpr uint16_t CONF_ODR_SHIFT   = 0;   // 4 bits
constexpr uint16_t CONF_RANGE_SHIFT = 4;   // 3 bits
constexpr uint16_t CONF_BW_SHIFT    = 7;   // 1 bit  (0 = ODR/2, 1 = ODR/4)
constexpr uint16_t CONF_AVG_SHIFT   = 8;   // 3 bits
constexpr uint16_t CONF_MODE_SHIFT  = 12;  // 3 bits

enum AccMode : uint16_t { MODE_OFF = 0, MODE_DUTY = 3, MODE_LOWPOWER = 4, MODE_PERF = 7 };

// FIFO_CONF
constexpr uint16_t FIFO_STOP_ON_FULL = 1 << 0;
constexpr uint16_t FIFO_TIME_EN      = 1 << 8;
constexpr uint16_t FIFO_ACC_EN       = 1 << 9;
constexpr uint16_t FIFO_GYR_EN       = 1 << 10;
constexpr uint16_t FIFO_TEMP_EN      = 1 << 11;

// Assinaturas de quadros invalidos inseridos pelo sensor (Table 18)
constexpr int16_t DUMMY_ACC_W1  = (int16_t)0x7F01;
constexpr int16_t DUMMY_INVALID = (int16_t)0x8000;

// Tabelas de conversao
float   odrHz(uint8_t code);          // codigo acc_odr -> Hz
uint8_t odrCodeFor(float hz);         // Hz -> codigo mais proximo
float   rangeLsbPerG(uint8_t code);   // codigo acc_range -> LSB/g
float   rangeG(uint8_t code);         // codigo acc_range -> fundo de escala em g
float   gyrLsbPerDps(uint8_t code);   // codigo gyr_range -> LSB/(graus/s)
float   gyrRangeDps(uint8_t code);    // codigo gyr_range -> fundo de escala em graus/s

struct BusConfig {
  bool     useSpi  = true;
  // SPI (4 fios)
  int8_t   sck     = 12;
  int8_t   miso    = 13;
  int8_t   mosi    = 11;
  int8_t   cs      = 10;
  uint32_t spiHz   = 10000000UL;   // datasheet: ate 10 MHz
  // I2C
  int8_t   sda     = 8;
  int8_t   scl     = 9;
  uint8_t  i2cAddr = 0x68;         // 0x69 se SDO em VDDIO
  uint32_t i2cHz   = 1000000UL;
};

class BMI323 {
 public:
  bool begin(const BusConfig &bus);
  bool present() const { return ok_; }

  uint16_t readReg(uint8_t reg);
  void     writeReg(uint8_t reg, uint16_t val);
  // Retorna false se o barramento falhou. Nesse caso dst NAO e alterado:
  // preencher com zero injetaria amostras falsas no analisador.
  bool     readBurst(uint8_t reg, uint16_t *dst, size_t words);

  // Configura os sensores em alta performance e (re)inicia o FIFO em modo
  // streaming. Com o giroscopio ligado o quadro passa de 3 para 6 words.
  bool configureAccel(uint8_t odrCode, uint8_t rangeCode, uint8_t avgCode, uint8_t bwCode,
                      bool gyroOn = false, uint8_t gyrRangeCode = 4);

  void     fifoFlush();
  uint16_t fifoFillWords();
  // Words por quadro do FIFO: 3 (so acc) ou 6 (acc + gyr)
  int      frameWords() const { return gyroOn_ ? 6 : 3; }
  bool     gyroEnabled() const { return gyroOn_; }
  // Le ate maxFrames quadros de frameWords() words. Retorna quadros validos.
  int      fifoReadFrames(int16_t *dst, int maxFrames);

  float    temperatureC();
  float    lsbPerG() const { return lsbPerG_; }
  float    lsbPerDps() const { return lsbPerDps_; }
  float    odr()     const { return odr_; }
  uint16_t lastError() const { return errReg_; }
  uint8_t  address() const { return bus_.i2cAddr; }
  // Varre o barramento I2C e imprime o que responder. So faz sentido em I2C.
  static void scanI2C(int8_t sda, int8_t scl, uint32_t hz, Print &out);
  uint32_t droppedFrames() const { return dropped_; }
  uint32_t busErrors() const { return busErr_; }

 private:
  BusConfig bus_;
  bool      ok_      = false;
  float     lsbPerG_ = 4096.0f;
  float     lsbPerDps_ = 16.4f;
  bool      gyroOn_  = false;
  float     odr_     = 1600.0f;
  uint16_t  errReg_  = 0;
  uint32_t  dropped_ = 0;
  uint32_t  busErr_  = 0;

  void busBegin();
};

extern BMI323 imu;

}  // namespace bmi
