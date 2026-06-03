#pragma once
#include <Arduino.h>
#include <time.h>

// Wall-clock time via SNTP/NTP (UTC). Call clockBegin() once WiFi is up; SNTP
// keeps it synced in the background. Used to stamp webhook/stats payloads.

void   clockBegin();    // start the SNTP client (UTC)
bool   clockSynced();   // true once the clock looks valid (post-2023)
time_t clockEpoch();    // UTC seconds since epoch, or 0 if not yet synced
String clockIso();      // "YYYY-MM-DDTHH:MM:SSZ" (UTC), or "" if not yet synced
