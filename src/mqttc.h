#pragma once
#include <Arduino.h>
#include "config_store.h"
#include "cv.h"

// Native MQTT client with optional TLS (mqtts) and Home Assistant MQTT
// auto-discovery. Publishes one retained topic per field under the base topic,
// plus discovery configs that create a single HA device (with firmware version
// and a GitHub link). LWT availability marks the device offline when it drops.

void mqttBegin(const Config* cfg, const CvResult* last);
void mqttLoop();           // call every main loop: (re)connect, keepalive, interval publish
void mqttReconfigure();    // re-apply settings after a config change (reconnect)
void mqttPublishNow();     // publish all state immediately (e.g. on count change)
bool mqttConnected();
