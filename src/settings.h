#pragma once
#include <Arduino.h>
#include "analyzer.h"

// Persistencia em NVS de toda a configuracao do analisador e do WiFi.
namespace settings {

struct WifiCfg {
  char ssid[33] = "";
  char pass[65] = "";
};

void begin();
void save();
void resetDefaults();

WifiCfg &wifi();
void saveWifi(const char *ssid, const char *pass);

}  // namespace settings
