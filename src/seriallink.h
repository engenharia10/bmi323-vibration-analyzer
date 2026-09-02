#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Enlace de dados pela USB/serial.
//
// Serve para o caso em que o PC nao esta na mesma rede da placa: em vez de
// trocar de WiFi so para abrir a interface, o navegador fala direto com a
// porta serial (Web Serial API) e recebe exatamente os mesmos pacotes do
// WebSocket.
//
// Formato do quadro:
//
//   0xAB 0xCD  <len:u16 LE>  <payload[len]>  <xor:u8>
//
// O payload e o mesmo do WebSocket: se comeca com 0xA5 e um pacote binario
// (espectro / osciloscopio / onda filtrada); qualquer outro byte inicial
// significa texto UTF-8 (JSON de status ou resposta de API).
//
// O SOF de dois bytes existe para o parser conseguir se realinhar no meio de
// um fluxo que tambem carrega as linhas de log em texto puro.
//
// O streaming comeca DESLIGADO: sem isso o monitor serial viraria uma sopa de
// binario. O host liga mandando "stream 1".
// ---------------------------------------------------------------------------

namespace slink {

void begin();
void loop();

bool streaming();
void setStreaming(bool on);

// Envia uma resposta de texto (JSON) enquadrada, para o host casar com o
// comando que enviou.
void sendText(const String &s);

uint32_t framesSent();

}  // namespace slink
