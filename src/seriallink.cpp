#include "seriallink.h"
#include "webui.h"
#include "config.h"

namespace slink {

static bool     g_on = false;
static uint32_t g_lastSeq = 0;
static uint32_t g_frames = 0;

// Manda o quadro so se couber inteiro no buffer de transmissao. Bloquear o
// loop esperando a UART esvaziar atrasaria a aquisicao; num enlace de tempo
// real e melhor perder um quadro do que acumular atraso.
static bool sendFrame(const uint8_t *payload, size_t len) {
  // Se a fila mal andou, o host parou de ler: descarta o quadro em vez de
  // bloquear o loop. Num enlace de tempo real perder e melhor que atrasar.
  if (Serial.availableForWrite() < 512) return false;

  uint8_t chk = 0;
  for (size_t i = 0; i < len; i++) chk ^= payload[i];

  const uint8_t hdr[4] = {0xAB, 0xCD, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  Serial.write(hdr, 4);
  Serial.write(payload, len);
  Serial.write(&chk, 1);
  g_frames++;
  return true;
}

void sendText(const String &s) {
  sendFrame((const uint8_t *)s.c_str(), s.length());
}

void begin() {
  // Buffer de saida grande o bastante para um pacote de espectro nao bloquear
  Serial.setTxBufferSize(16384);
}

bool streaming() { return g_on; }

void setStreaming(bool on) {
  g_on = on;
  g_lastSeq = 0;
  if (on) sendText(webui::apiConfigJson());
}

uint32_t framesSent() { return g_frames; }

void loop() {
  if (!g_on) return;

  static uint32_t tSpec = 0, tScope = 0, tFilt = 0, tRaw = 0;
  const uint32_t now = millis();
  uint8_t *buf = webui::txBuffer();
  const size_t cap = webui::txBufferSize();
  if (!buf) return;

  if (now - tSpec >= SPECTRUM_PERIOD_MS) {
    tSpec = now;
    String status;
    size_t len = webui::buildSpectrumPacket(buf, cap, g_lastSeq, &status);
    if (len && sendFrame(buf, len)) sendText(status);
  }
  if (now - tScope >= SCOPE_PERIOD_MS) {
    tScope = now;
    size_t len = webui::buildScopePacket(buf, cap);
    if (len) sendFrame(buf, len);
  }
  if (now - tRaw >= SCOPE_PERIOD_MS) {
    tRaw = now;
    size_t len = webui::buildMultiScopePacket(buf, cap, false);
    if (len) sendFrame(buf, len);
  }
  if (now - tFilt >= SCOPE_PERIOD_MS * 2) {
    tFilt = now;
    size_t len = webui::buildMultiScopePacket(buf, cap, true);
    if (len) sendFrame(buf, len);
  }
}

}  // namespace slink
