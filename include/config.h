#pragma once
// ---------------------------------------------------------------------------
// Configuracao de hardware e limites do analisador.
// Altere os pinos aqui conforme a sua ligacao fisica.
// ---------------------------------------------------------------------------

// ---- Interface com o BMI323 ------------------------------------------------
#ifndef BMI_USE_SPI
#define BMI_USE_SPI 1
#endif

// SPI (4 fios). Recomendado: suporta ODR alto sem gargalo de barramento.
#define PIN_BMI_SCK   12
#define PIN_BMI_MISO  13
#define PIN_BMI_MOSI  11
#define PIN_BMI_CS    10
#define BMI_SPI_HZ    10000000UL   // datasheet: ate 10 MHz

// I2C. Na Waveshare ESP32-S3-Touch-LCD-7 estes sao os pinos do header P4,
// o mesmo barramento do touch GT911 e do expansor CH422G.
#define PIN_BMI_SDA   8
#define PIN_BMI_SCL   9
#define BMI_I2C_ADDR  0x68         // 0x69 se SDO em VDDIO; detectado sozinho
#ifndef BMI_I2C_HZ_OVERRIDE
#define BMI_I2C_HZ    1000000UL    // Fm+; caia para 400000 se dividir o bus
#else
#define BMI_I2C_HZ    BMI_I2C_HZ_OVERRIDE
#endif

#define PIN_BMI_INT1  14           // opcional, nao usado (leitura e por polling do FIFO)

// Velocidade da serial. Precisa ser alta: com o streaming ligado o enlace
// carrega os mesmos pacotes do WebSocket (~70 kB/s no pior caso).
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 2000000
#endif

// ---- Rede ------------------------------------------------------------------
#define AP_SSID       "BMI323-Vib"
#define AP_PASS       "vibracao123"    // >= 8 caracteres
#define MDNS_HOST     "bmi323"         // http://bmi323.local
#define WIFI_STA_TIMEOUT_MS 12000

// ODR padrao. No SPI da para ir a 6.4 kHz; no I2C o barramento satura antes,
// ainda mais dividido com outros dispositivos.
#ifndef BMI_DEFAULT_ODR_HZ
#if BMI_USE_SPI
#define BMI_DEFAULT_ODR_HZ 1600
#else
#define BMI_DEFAULT_ODR_HZ 800
#endif
#endif

// ---- Aquisicao / DSP -------------------------------------------------------
#define CHANNELS         6         // aX aY aZ gX gY gZ
#define CH_ACC_MAG       6         // pseudo-canal: modulo do acelerometro
#define FFT_MAX          1024      // potencia de 2; maximo suportado
#define FFT_MIN          256
#define RING_SAMPLES     2048      // buffer circular de amostras (>= 2*FFT_MAX)
#define SCOPE_POINTS     256       // pontos enviados ao osciloscopio da UI

#define TASK_ACQ_CORE    0
#define TASK_DSP_CORE    1

#define SPECTRUM_PERIOD_MS 200     // taxa de envio do espectro
#define SCOPE_PERIOD_MS    100     // taxa de envio da forma de onda
#define STATUS_PERIOD_MS   1000
