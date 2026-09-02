// ---------------------------------------------------------------------------
// BMI323 Vibration Analyzer - ESP32-S3
//
// Le o acelerometro BMI323 pelo FIFO em alta taxa, calcula a FFT no proprio
// ESP32 e publica espectro, forma de onda e picos por WebSocket para a
// interface web em data/ (LittleFS).
// ---------------------------------------------------------------------------
#include <Arduino.h>
#include "config.h"
#include <bmi323.h>
#include "analyzer.h"
#include "settings.h"
#include "webui.h"
#include "seriallink.h"

void setup() {
  // setTxBufferSize so vale ANTES do begin: com o buffer padrao de 256 bytes
  // nenhum pacote de espectro caberia e todo quadro seria descartado.
  slink::begin();
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.println("=== BMI323 Vibration Analyzer ===");
#if BMI_USE_SPI
  Serial.printf("Interface: SPI  SCK=%d MISO=%d MOSI=%d CS=%d\n",
                PIN_BMI_SCK, PIN_BMI_MISO, PIN_BMI_MOSI, PIN_BMI_CS);
#else
  Serial.printf("Interface: I2C  SDA=%d SCL=%d  %u kHz\n",
                PIN_BMI_SDA, PIN_BMI_SCL, (unsigned)(BMI_I2C_HZ / 1000));
  // Varredura antes de tudo: se o sensor nao aparecer aqui, o problema e
  // eletrico (alimentacao, pull-up, fio), nao de configuracao.
  bmi::BMI323::scanI2C(PIN_BMI_SDA, PIN_BMI_SCL, BMI_I2C_HZ, Serial);
#endif

  if (!vib::begin()) {
    Serial.println("[imu] BMI323 nao respondeu. Verifique alimentacao e ligacoes.");
    Serial.println("[imu] A interface web sobe assim mesmo (sem dados).");
  } else {
    Serial.printf("[imu] ok - ODR %.0f Hz, fundo de escala +/-%.0f g", 
                  bmi::imu.odr(), bmi::rangeG(vib::sensorCfg().rangeCode));
#if !BMI_USE_SPI
    Serial.printf(", endereco 0x%02X", bmi::imu.address());
#endif
    Serial.println();
  }

  settings::begin();
  webui::begin();

  Serial.printf("Abra http://%s/  (ou http://%s.local/)\n", webui::ipString(), MDNS_HOST);
}

// Linha de diagnostico no serial: confirma que o FIFO esta entregando amostras
// na taxa certa e que a cadeia de filtros e a atitude estao vivas, sem precisar
// abrir a interface web. Util no bring-up e quando algo para de funcionar.
static void heartbeat() {
  static uint32_t last = 0;
  static uint32_t seq = 0;
  if (millis() - last < 5000) return;
  last = millis();

  vib::withResults(seq, [](const vib::Results &r) {
    seq = r.seq;
    const att::State &a = r.attitude;
    Serial.printf("[st] %.0f/%.0f Hz  rms %.1f->%.1f mg  |a| %.3f g (%.0f%%)  "
                  "roll %.1f pitch %.1f yaw %.1f",
                  r.fs, r.measuredHz, r.rmsRaw * 1000.0f, r.rmsFilt * 1000.0f,
                  a.accMagF, a.trust * 100.0f, a.roll, a.pitch, a.yaw);
    if (r.nPeaks > 0) {
      Serial.printf("  pico %.1f Hz / %.1f mg", r.peaks[0].freq, r.peaks[0].amp * 1000.0f);
    }
    if (bmi::imu.droppedFrames()) Serial.printf("  descartados %u", bmi::imu.droppedFrames());
    Serial.printf("  ws %d/%u", webui::wsClients(), webui::framesSent());
    if (bmi::imu.busErrors()) Serial.printf("  erros de barramento %u", bmi::imu.busErrors());
    // Taxa real muito abaixo da configurada = o barramento nao da conta.
    // O espectro fica com a escala de frequencia errada, entao vale o aviso.
    if (r.measuredHz > 1.0f && r.measuredHz < r.fs * 0.9f) {
      Serial.printf("  <<< BARRAMENTO SATURADO: baixe o ODR");
    }
    Serial.println();
  });
}

// Comandos curtos pelo serial, para nao precisar entrar na rede da placa
// so para trocar um parametro durante o bring-up.
static void serialCommands() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { line += ch; continue; }
    line.trim();
    if (!line.length()) continue;

    if (line.startsWith("odr ")) {
      vib::SensorCfg c = vib::sensorCfg();
      c.odrCode = bmi::odrCodeFor(line.substring(4).toFloat());
      vib::applySensor(c);
      Serial.printf("[cmd] ODR -> %.0f Hz\n", bmi::odrHz(c.odrCode));
    } else if (line.startsWith("range ")) {
      vib::SensorCfg c = vib::sensorCfg();
      int g = line.substring(6).toInt();
      c.rangeCode = g <= 2 ? 0 : g <= 4 ? 1 : g <= 8 ? 2 : 3;
      vib::applySensor(c);
      Serial.printf("[cmd] escala -> +/-%.0f g\n", bmi::rangeG(c.rangeCode));
    } else if (line.startsWith("bw ")) {
      vib::SensorCfg c = vib::sensorCfg();
      c.bwCode = line.substring(3).toInt() ? 1 : 0;
      vib::applySensor(c);
      Serial.printf("[cmd] largura de banda -> ODR/%d\n", c.bwCode ? 4 : 2);
    } else if (line == "gyro on" || line == "gyro off") {
      vib::SensorCfg c = vib::sensorCfg();
      c.gyroOn = line.endsWith("on");
      vib::applySensor(c);
      Serial.printf("[cmd] giroscopio -> %s\n", c.gyroOn ? "ligado" : "desligado");
    } else if (line == "peaks") {
      vib::withResults(0, [](const vib::Results &r) {
        // A amplitude de um canal do giroscopio ja esta em graus/s: multiplicar
        // por 1000 e rotular "mg", como antes, inflava o valor em 1000x.
        const uint8_t ch = vib::analysisCfg().axis;
        const bool gyro = (ch >= 3 && ch < CHANNELS);
        const float k = gyro ? 1.0f : 1000.0f;
        const char *un = gyro ? "graus/s" : "mg";
        Serial.printf("[cmd] %d picos no canal %u (%.2f Hz/bin):\n", r.nPeaks, ch, r.binHz);
        for (int i = 0; i < r.nPeaks; i++) {
          Serial.printf("        %7.2f Hz  %9.3f %s\n", r.peaks[i].freq, r.peaks[i].amp * k, un);
        }
      });
    } else if (line.startsWith("wifi ")) {
      // "wifi <ssid> <senha>" - coloca a placa na rede e reinicia. Evita
      // ter que trocar o PC para o AP da placa so para abrir a interface.
      String rest = line.substring(5);
      int sp = rest.indexOf(' ');
      String ssid = sp < 0 ? rest : rest.substring(0, sp);
      String pass = sp < 0 ? "" : rest.substring(sp + 1);
      settings::saveWifi(ssid.c_str(), pass.c_str());
      Serial.printf("[cmd] wifi -> %s, reiniciando...\n", ssid.c_str());
      delay(200);
      ESP.restart();
    } else if (line == "ap") {
      settings::saveWifi("", "");
      Serial.println("[cmd] voltando para o access point, reiniciando...");
      delay(200);
      ESP.restart();
    } else if (line == "ip") {
      Serial.printf("[cmd] %s  http://%s/  clientes ws: %d  pacotes: %u\n",
                    webui::isAccessPoint() ? "access point" : "estacao",
                    webui::ipString(), webui::wsClients(), webui::framesSent());
    } else if (line.startsWith("stream")) {
      // Liga o fluxo binario para a interface via Web Serial. Fica
      // desligado por padrao para nao poluir o monitor.
      bool on = !line.endsWith("0");
      slink::setStreaming(on);
      if (!on) Serial.println("[cmd] streaming desligado");
    } else if (line.startsWith("api ")) {
      // "api filters dc=1&dcf=2" - mesma API do HTTP, pela serial.
      String rest = line.substring(4);
      int sp = rest.indexOf(' ');
      String path  = sp < 0 ? rest : rest.substring(0, sp);
      String query = sp < 0 ? "" : rest.substring(sp + 1);
      slink::sendText(webui::handleApi(path, query));
    } else if (line == "help") {
      Serial.println("[cmd] odr <Hz> | range <g> | bw <0|1> | gyro on|off | peaks");
      Serial.println("[cmd] wifi <ssid> <senha> | ap | ip");
      Serial.println("[cmd] stream 1|0 | api <rota> <query>");
    } else {
      Serial.printf("[cmd] nao entendi: %s (tente 'help')\n", line.c_str());
    }
    line = "";
  }
}

void loop() {
  webui::loop();
  slink::loop();
  serialCommands();
  // Com o streaming ligado o serial e canal de dados: o heartbeat em texto
  // so atrapalharia o parser do host.
  if (!slink::streaming()) heartbeat();
  delay(2);
}
