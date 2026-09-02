#pragma once
#include <Arduino.h>

// Servidor HTTP + WebSocket que serve a interface em data/ (LittleFS)
// e transmite espectro, forma de onda e status em tempo real.
namespace webui {

void begin();
void loop();          // chame no loop() principal
const char *ipString();
bool isAccessPoint();

// Instrumentacao: quantos navegadores estao ligados e quantos pacotes ja
// sairam. Se o cliente conecta mas o contador nao anda, o problema esta no
// envio; se nem cliente aparece, e rede.
int      wsClients();
uint32_t framesSent();

// API compartilhada com o enlace serial: aplica os parametros de uma query
// string ("dc=1&dcf=2") e devolve o JSON de resposta.
String handleApi(const String &path, const String &query);
String apiConfigJson();

// Montagem dos pacotes binarios. O enlace serial usa exatamente os mesmos,
// entao a interface nao precisa saber por onde os dados chegaram.
size_t buildSpectrumPacket(uint8_t *buf, size_t cap, uint32_t &lastSeq, String *statusOut);
size_t buildScopePacket(uint8_t *buf, size_t cap);
// filtered = false monta a onda CRUA (type 4); true, a filtrada (type 3)
size_t buildMultiScopePacket(uint8_t *buf, size_t cap, bool filtered);
uint8_t *txBuffer();
size_t   txBufferSize();

}  // namespace webui
