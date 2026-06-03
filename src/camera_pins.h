#pragma once
// ---------------------------------------------------------------------------
// OV5640 v1.2 module pin map (plain ESP32 board, NOT AI-Thinker, NOT S3).
//
// Verified from the user's own working ESPHome config:
//   github.com/giovi321/ESP32-cam-OV5640  ->  ESPHome_OV5640.yaml
// and cross-checked against the board's official "IO Configuration" table
// (OV5640 V1.2, ESP32-D0WDQ6, 8MB PSRAM, CH340X, IP5306) -- exact match.
//
// ESPHome data_pins[] are ordered D0..D7 == Y2..Y9, which map directly to
// esp_camera's pin_d0..pin_d7.
//
//   external_clock pin GPIO15 @ 12 MHz   -> XCLK
//   i2c sda GPIO22 / scl GPIO23          -> SCCB (SIOD/SIOC)
//   data_pins [2,14,35,12,27,33,34,39]   -> D0..D7
//   vsync GPIO18 / href GPIO36 / pclk 26
//   reset GPIO5                          -> RESET   (PWDN not wired -> -1)
//
// Note: GPIO0 (BOOT) is intentionally NOT used by the camera here, so it is
// free for the long-press config button (see buttons.cpp).
// ---------------------------------------------------------------------------

#define CAM_PIN_PWDN   -1
#define CAM_PIN_RESET   5
#define CAM_PIN_XCLK   15
#define CAM_PIN_SIOD   22   // SCCB SDA
#define CAM_PIN_SIOC   23   // SCCB SCL

#define CAM_PIN_D0      2   // Y2
#define CAM_PIN_D1     14   // Y3
#define CAM_PIN_D2     35   // Y4
#define CAM_PIN_D3     12   // Y5
#define CAM_PIN_D4     27   // Y6
#define CAM_PIN_D5     33   // Y7
#define CAM_PIN_D6     34   // Y8
#define CAM_PIN_D7     39   // Y9

#define CAM_PIN_VSYNC  18
#define CAM_PIN_HREF   36
#define CAM_PIN_PCLK   26

#define CAM_XCLK_FREQ_HZ 12000000   // OV5640 on this board is happy at 12 MHz

// Free GPIO used for the "enter AP config mode" long-press button.
#define PIN_CONFIG_BUTTON 0         // IO0 / BOOT button (confirmed on this board)

// Onboard status LED (GPIO25 per the board IO table; free, not used by camera).
#define PIN_STATUS_LED 25
// Set to 1 if the LED turns ON when the pin is driven LOW (flip if inverted).
#define LED_ACTIVE_LOW 0
