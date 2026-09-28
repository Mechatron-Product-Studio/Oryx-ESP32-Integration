/**
 * Oryx Signal — standalone ESP32 generator (SD card)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Plays the .txt files exported by Oryx Signal Studio straight from an SD
 * card. One sketch for both export modes: the mode is detected from the file.
 *
 *   digital  → one row per state change: step + one 0/1 column per channel.
 *              Out on 4 GPIOs (CKP, CMP 1, CMP 2, CMP 3).
 *   analog   → one row per chord segment (from, to, channel, levels, handle,
 *              radius, corner radius) plus "RESOLUTION (bits)" metadata. The
 *              curve is rebuilt here with the same geometry as the editor and
 *              sampled into RAM, then played through the two DACs.
 *
 * Two ways to drive it — the SD card is always required:
 *   · OLED + encoder: browse, choose cycles, change RPM on the fly.
 *   · Serial only (115200 baud): type "help". Works with or without the
 *     OLED and the encoder attached; both can be used at the same time.
 * The OLED is detected at boot (I2C 0x3C / 0x3D). Without it every screen is
 * skipped and the serial console does the whole job.
 *
 * Speed is set in RPM (100 – 8000):
 *
 *   step_time (µs) = 60 000 000 / (RPM × total_steps)
 *
 * File metadata (all optional, at the end of the file):
 *   VOLTAGE: 5            analog: scales the DAC output
 *   STEP_TIME (us): 1000  diagnostic only (compared against the RPM tick)
 *   CYCLES: 2             cycles in the signal; if present, config is skipped
 *   RPM: 800              start speed
 *
 * Libraries: U8g2, ESP32Encoder, Button2. Board: classic ESP32 (WROOM /
 * DevKit v1). Also builds as a PlatformIO main.cpp: everything is declared
 * before it is used.
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <Button2.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <esp_timer.h>
#include <vector>
#include <algorithm>
#include <cmath>

// ──────────────────────────────────────────────
//  PIN CONFIGURATION
// ──────────────────────────────────────────────
#define ENCODER_PIN_A   32
#define ENCODER_PIN_B   33
#define ENCODER_BTN_PIN  4   // GPIO 25 is left free: it is DAC1 (analog channel 2)

#define SD_CS_PIN        5   // SD card CS pin

// Signal output pins (one per channel: CKP, CMP, CMP2, CMP3)
#define NUM_CHANNELS     4
#define SIG_PIN_CH0     26   // CKP
#define SIG_PIN_CH1     27   // CMP
#define SIG_PIN_CH2     14   // CMP 2
#define SIG_PIN_CH3     12   // CMP 3

static const gpio_num_t sigPins[NUM_CHANNELS] = {
  (gpio_num_t)SIG_PIN_CH0,
  (gpio_num_t)SIG_PIN_CH1,
  (gpio_num_t)SIG_PIN_CH2,
  (gpio_num_t)SIG_PIN_CH3
};

// Bit mask of all signal pins — used to drive every channel LOW at once.
#define ALL_CH_MASK ((1UL << SIG_PIN_CH0) | (1UL << SIG_PIN_CH1) | \
                     (1UL << SIG_PIN_CH2) | (1UL << SIG_PIN_CH3))

// Analog outputs. The ESP32 only has DACs on GPIO 26 (DAC2) and GPIO 25 (DAC1).
// Analog channel 1 goes out on GPIO 26 — the same connector as CKP, so an
// analog (inductive) CKP comes out where the digital one does. Analog
// channel 2 goes out on GPIO 25. (If the encoder button is ever put back on
// GPIO 25, the second analog channel switches itself off.)
#define ANA_PIN_CH0     26   // DAC2
#define ANA_PIN_CH1     25   // DAC1
#if ENCODER_BTN_PIN == ANA_PIN_CH1
  #define ANA_NUM_CHANNELS 1
#else
  #define ANA_NUM_CHANNELS 2
#endif
static const uint8_t anaPins[2] = { ANA_PIN_CH0, ANA_PIN_CH1 };

// Analog curve sampling: up to ANA_MAX_SPP samples per step, and never more
// than ANA_MAX_SAMPLES per channel (8-bit DAC values, 8 KB per channel).
#define ANA_MAX_SPP      32
#define ANA_MAX_SAMPLES  8192
#define DAC_FULL_SCALE_V 3.3f

// I2C OLED (SSD1306 128x64) – default SDA=21, SCL=22
// If your board uses different pins, change the constructor below.

// ──────────────────────────────────────────────
//  OBJECTS
// ──────────────────────────────────────────────
// SSD1306 128x64 I2C – full buffer for smooth drawing
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

ESP32Encoder encoder;
Button2 button;

#define LOGO_WIDTH  128
#define LOGO_HEIGHT  64
static const unsigned char logo_bitmap[] U8X8_PROGMEM = {

  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0xFF, 0x0F, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFF, 0xFF,
  0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xF8, 0x1F, 0x00, 0x00, 0xF8, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x07, 0x00, 0x00, 0xE0, 0x1F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x01, 0x00,
  0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xFE, 0x01, 0x00, 0x00, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x80, 0xF7,
  0xD7, 0x01, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7F, 0xE0, 0xFF, 0xFF, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xF0, 0xFF, 0xFF, 0x07, 0xFE, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0xFF,
  0xFF, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x3F, 0xF8, 0xFF, 0xFF, 0x07, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF8, 0xFF, 0xFF, 0x01, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0xF8, 0xFF,
  0x07, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x3F, 0xF0, 0xFF, 0x0F, 0x80, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0xFF, 0x1F, 0xC0, 0xFF, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xE0, 0xFF,
  0x3F, 0xE0, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x7F, 0x00, 0xBE, 0x3F, 0xE0, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x7C, 0xE0, 0x7F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x01, 0x00,
  0xFC, 0xC0, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xFC, 0x03, 0x00, 0xF8, 0x81, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x07, 0x00, 0xF0, 0x03, 0x1F, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x1F, 0x00,
  0xE0, 0x03, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0xF0, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0xFF,
  0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0xFC, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x99, 0x99, 0x02, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x1C, 0x08, 0x1F, 0x3C, 0x3E, 0x3E, 0x84, 0x3C, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x7F, 0x8C, 0x3F, 0xFE, 0xFE, 0x7F, 0x8C, 0xFF,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x63, 0x9E, 0x31, 0xC6,
  0x06, 0x63, 0xCE, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0x9E, 0x3F, 0xFE, 0x7F, 0x7F, 0x9F, 0xFF, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xB7, 0x1F, 0x7E, 0x3E, 0x3F, 0xDF, 0x7E,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE7, 0xBF, 0x19, 0x76,
  0x16, 0x83, 0xBF, 0x77, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xBE, 0xF7, 0x39, 0xE6, 0xFF, 0x83, 0xF5, 0xE3, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x14, 0x80, 0x20, 0x00, 0x18, 0x80, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
};

// ──────────────────────────────────────────────
//  ICONS 12x12 pixels (XBM format, stored in bytes)
// ──────────────────────────────────────────────
// Folder icon 12x12
#define ICON_W 12
#define ICON_H 12
static const unsigned char icon_folder[] U8X8_PROGMEM = {
  0x00,0x00, // ............
  0x3C,0x00, // ..####......
  0x7E,0x00, // .######.....
  0xFF,0x0F, // ############
  0x01,0x08, // #..........#
  0x01,0x08, // #..........#
  0x01,0x08, // #..........#
  0x01,0x08, // #..........#
  0x01,0x08, // #..........#
  0x01,0x08, // #..........#
  0xFF,0x0F, // ############
  0x00,0x00, // ............
};

// File / TXT icon 12x12
static const unsigned char icon_file[] U8X8_PROGMEM = {
  0x00,0x00, // ............
  0xFE,0x00, // .#######....
  0x02,0x01, // .#......#...
  0x02,0x03, // .#.......##.
  0x02,0x02, // .#........#.
  0xF2,0x02, // .#..####..#.
  0x02,0x02, // .#........#.
  0xF2,0x02, // .#..####..#.
  0x02,0x02, // .#........#.
  0xF2,0x02, // .#..####..#.
  0xFE,0x03, // .#########..
  0x00,0x00, // ............
};

// ──────────────────────────────────────────────
//  APP STATE
// ──────────────────────────────────────────────
enum AppState {
  STATE_SPLASH,
  STATE_BROWSER,
  STATE_VIEW_FILE,
  STATE_SIG_CONFIG,    // configure cycles before playback
  STATE_PLAYING        // signal generation running (RPM adjustable)
};

AppState appState = STATE_SPLASH;
unsigned long splashStart = 0;
#define SPLASH_DURATION 3000  // ms

// True when an SSD1306 answered on I2C at boot. Without it every draw*()
// returns right away and the serial console is the only interface.
bool oledPresent = false;

// Which kind of file is loaded.
enum SignalMode {
  MODE_DIGITAL,
  MODE_ANALOG
};
SignalMode signalMode = MODE_DIGITAL;

// ──────────────────────────────────────────────
//  FILE BROWSER DATA
// ──────────────────────────────────────────────
struct FileEntry {
  String name;   // display name
  bool   isDir;  // true = directory
};

String              currentPath  = "/";
std::vector<FileEntry> entries;
int                 cursorIndex  = 0;   // selected item
int                 scrollOffset = 0;   // first visible item

// How many items fit on screen (header 9px, items 13px each, screen 64px)
#define HEADER_H   9    // small font (5x7) + 1px divider line
#define ITEM_H     13
#define VISIBLE_ITEMS  ((64 - HEADER_H) / ITEM_H)   // = 4 items

// ──────────────────────────────────────────────
//  FILE VIEWER DATA
// ──────────────────────────────────────────────
std::vector<String> fileLines;
int viewScrollY   = 0;
int totalFileLines = 0;
#define VIEW_LINES_PER_PAGE  6   // approx lines visible at font 6x10

// ──────────────────────────────────────────────
//  SIGNAL GENERATOR DATA (digital)
// ──────────────────────────────────────────────
// Files only contain rows where a channel changes.
// Each "event" stores the channel values and how long (in step-fractions)
// to hold those values before the next change.
struct SignalEvent {
  float   stepF;                  // absolute step position from file (fractional)
  float   durationF;              // duration in steps (next_step - this_step), fractional
  uint8_t val[NUM_CHANNELS];     // channel values (0|1)
};

std::vector<SignalEvent> signalEvents;   // parsed event list
float  signalTotalStepsF = 0;            // total step-duration of one cycle (fractional)
int    signalTotalEvents = 0;            // number of events (transitions)
String signalFileName    = "";
String signalFilePath    = "";

// Channel assignments are fixed:
//   CH0 = CKP (crankshaft position — defines tooth timing)
//   CH1 = CMP 1 (camshaft position sensor 1)
//   CH2 = CMP 2
//   CH3 = CMP 3
String channelNames[NUM_CHANNELS] = {"CKP","CMP 1","CMP 2","CMP 3"};

// ──────────────────────────────────────────────
//  SIGNAL GENERATOR DATA (analog)
// ──────────────────────────────────────────────
// The curve is rebuilt from the file's segments and sampled once, at load
// time, into 8-bit DAC values. anaCount samples span signalTotalStepsF steps.
uint8_t *anaSamples[2] = { NULL, NULL };
uint32_t anaCount      = 0;   // samples per channel
uint8_t  anaChannels   = 0;   // analog outputs in use

// Analog curve geometry types. They live up here, before any function, because
// the Arduino IDE writes a prototype for every function above the first one:
// a function taking one of these types would be declared before the type.
struct APt {
  double x, y;
};

struct APiece {
  APt    to;
  APt    ctrl;
  bool   quad;     // false = straight line to `to`
  double corner;   // radius of the corner where this piece ends (0..1)
};

// One row of the analog file.
struct AnaSeg {
  float   from, to;   // steps
  uint8_t ch;         // 1-based, as in the file
  float   l0, l1;     // start / end level (raw, 0 .. 2^bits-1)
  bool    straight;   // no handle: a plain ramp (or a vertical jump if from == to)
  float   t;          // handle position along the chord, 0..1
  float   h;          // handle level (raw)
  float   r;          // chord radius, 0..1
  float   endR;       // corner radius at the end point, 0..1
};

// Metadata extracted from signal file (optional, 0 if missing)
float   metaVoltage    = 0.0f;   // VOLTAGE:
int     metaCycles     = 0;      // CYCLES: (crankshaft revolutions per file playback)
int     metaRPM        = 0;      // RPM: start speed (0 = keep the current one)
float   metaStepTime   = 0.0f;   // STEP_TIME: per-step microseconds the file was authored with (diagnostic)
int     metaResolution = 0;      // RESOLUTION (bits): analog files only

// RPM-based speed control (100 .. 8000 RPM)
// tickUs (step_time) is derived from the formula:
//   step_time = 60_000_000 / (RPM * total_pulses)
// where total_pulses is the total pulse count of one cycle read from the txt.
volatile uint32_t currentRPM = 800;  // default 800 RPM
#define RPM_MIN       100
#define RPM_MAX       8000
#define RPM_STEP      100            // encoder step size

// Crankshaft revolutions contained in one file playback.
// From CYCLES metadata if present, otherwise user-adjustable.
#define CYCLES_DEFAULT 2
#define CYCLES_MIN     1
#define CYCLES_MAX     8
uint8_t signalCycles = CYCLES_DEFAULT;

// Per-step duration in microseconds (computed from RPM, not user-set directly).
// Kept as float so the sub-microsecond fraction is preserved — truncating it to
// an integer biased every step short, making playback a few µs/pulse too fast.
volatile float tickUs = 1000.0f;

// Playback state
volatile bool     isPlaying        = false;
volatile bool     isPaused         = false;   // short-click toggles this during playback
volatile uint32_t playbackStep     = 0;
volatile uint32_t loopCount        = 0;     // completed signal cycles

// ──────────────────────────────────────────────
//  ENCODER STATE
// ──────────────────────────────────────────────
int64_t lastEncoderCount = 0;

// ──────────────────────────────────────────────
//  HEADER SCROLL ANIMATION
// ──────────────────────────────────────────────
int    headerScrollOffset  = 0;     // current pixel offset (0 = no scroll)
int    headerTextWidth     = 0;     // full text width in pixels
bool   headerNeedsScroll   = false; // true if text wider than screen
unsigned long headerLastStep  = 0;  // millis of last scroll step
unsigned long headerPauseUntil = 0; // millis to wait before scrolling again
#define HEADER_SCROLL_SPEED  40     // ms between each pixel shift
#define HEADER_SCROLL_PAUSE 1500    // ms pause at start/end before scrolling
#define HEADER_SCROLL_GAP   20      // px gap between end and restart of text
#define HEADER_MAX_W        124     // visible pixel width for header text

// ──────────────────────────────────────────────
//  SELECTED-ITEM SCROLL ANIMATION (browser)
// ──────────────────────────────────────────────
int    itemScrollOffset  = 0;
int    itemTextWidth     = 0;
bool   itemNeedsScroll   = false;
unsigned long itemLastStep   = 0;
unsigned long itemPauseUntil = 0;
#define ITEM_TEXT_X    16
#define ITEM_MAX_W     (128 - ITEM_TEXT_X - 4)   // = 108 px (leave room for scrollbar)

// ──────────────────────────────────────────────
//  FORWARD DECLARATIONS
// ──────────────────────────────────────────────
void loadDirectory(const String &path);
void drawBrowser();
void drawFileViewer();
void openSelected();
void goBack();
void handleEncoder();
void onButtonClick(Button2 &btn);
void onButtonLongClick(Button2 &btn);
void drawSplash();
void readFileContent(const String &path);
String truncateString(const String &str, int maxPixels);
void resetHeaderScroll(const String &text);
bool updateHeaderScroll();
void resetSelectedItemScroll();
bool updateItemScroll();
// Signal generator
bool isAnalogFile(const String &path);
bool parseMetaLine(const String &line);
bool parseSignalFile(const String &path);
bool parseAnalogFile(const String &path);
void drawSigConfig();
void drawPlaying();
void startPlayback();
void stopPlayback();
void freeSigBuffers();
void freeAnaBuffers();
void recalcTickFromRPM();
void stepRPM(int ticks);
String modeLabel();
String stripExtension(const String &name);
bool openSignalFile(const String &path, const String &name, bool autoStart);
// OLED detection + serial console
bool detectOled();
void serialPoll();
void handleSerialLine(String line);
void printHelp();
void printStatus();
void listDirectory();
void changeDirectory(const String &arg);
int  findEntry(const String &arg);

// ──────────────────────────────────────────────
//  SETUP
// ──────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("[BOOT] Starting Oryx Signal SD player..."));

  // ── OLED (optional) ──
  oledPresent = detectOled();
  if (oledPresent) {
    u8g2.begin();
    u8g2.setBusClock(400000);     // I2C 400 kHz fast mode – reduce OLED update time
    Serial.println(F("[BOOT] OLED found"));
  } else {
    Serial.println(F("[BOOT] No OLED — serial console only"));
  }
  u8g2.setFont(u8g2_font_6x10_tr);   // text widths are still measured without a screen

  // ── Show splash ──
  appState = STATE_SPLASH;
  splashStart = millis();
  drawSplash();

  // ── SD Card (required) ── retried until a card shows up
  while (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("[SD] Mount FAILED! Insert the card — retrying in 2 s"));
    if (oledPresent) {
      u8g2.clearBuffer();
      u8g2.drawStr(10, 32, "SD Card Error!");
      u8g2.sendBuffer();
    }
    delay(2000);
  }
  Serial.println(F("[SD] Mounted OK"));

  // ── Encoder ──
  // attachFullQuad cuenta 4 pulsos por tick mecánico completo.
  // La acción se dispara sólo cuando se acumulan ±4 pulsos (1 tick real).
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B);
  encoder.setFilter(1023);
  encoder.setCount(0);
  lastEncoderCount = 0;

  // ── Button ──
  button.begin(ENCODER_BTN_PIN, INPUT_PULLUP, true);   // active LOW
  button.setClickHandler(onButtonClick);
  button.setLongClickDetectedHandler(onButtonLongClick);
  button.setLongClickTime(600);

  // ── Load root directory ──
  loadDirectory("/");

  printHelp();
  listDirectory();
}

// ──────────────────────────────────────────────
//  LOOP
// ──────────────────────────────────────────────
void loop() {
  button.loop();
  serialPoll();

  switch (appState) {
    case STATE_SPLASH:
      if (!oledPresent || millis() - splashStart >= SPLASH_DURATION) {
        appState = STATE_BROWSER;
        drawBrowser();
      }
      break;

    case STATE_BROWSER: {
      handleEncoder();
      // Advance both scrolls first, then redraw at most once. If we redrew
      // inside each update, a single iteration could trigger two full I2C
      // sendBuffer calls (~25 ms each at 400 kHz), exceeding the 40 ms
      // animation period and visibly slowing both scrolls.
      bool hAdv = updateHeaderScroll();
      bool iAdv = updateItemScroll();
      if (hAdv || iAdv) drawBrowser();
      break;
    }

    case STATE_VIEW_FILE:
      handleEncoder();
      if (updateHeaderScroll()) drawFileViewer();
      break;

    case STATE_SIG_CONFIG:
      handleEncoder();
      break;

    case STATE_PLAYING:
      // Allow RPM changes mid-generation via encoder
      handleEncoder();
      break;
  }
}

// ──────────────────────────────────────────────
//  SPLASH SCREEN
// ──────────────────────────────────────────────
void drawSplash() {
  if (!oledPresent) return;
  u8g2.clearBuffer();
  u8g2.drawXBMP(0, 0, LOGO_WIDTH, LOGO_HEIGHT, logo_bitmap);
  u8g2.sendBuffer();
}

// ──────────────────────────────────────────────
//  LOAD DIRECTORY CONTENTS
// ──────────────────────────────────────────────
void loadDirectory(const String &path) {
  entries.clear();
  cursorIndex  = 0;
  scrollOffset = 0;

  File root = SD.open(path);
  if (!root || !root.isDirectory()) {
    Serial.printf("[SD] Cannot open dir: %s\n", path.c_str());
    return;
  }

  // Temporary vectors to sort folders first, then files
  std::vector<FileEntry> dirs;
  std::vector<FileEntry> files;

  File file = root.openNextFile();
  while (file) {
    String fname = String(file.name());
    // skip hidden files and system folders
    if (!fname.startsWith(".") && !fname.equalsIgnoreCase("System Volume Information")) {
      if (file.isDirectory()) {
        dirs.push_back({fname, true});
      } else {
        // Show .txt and .bin files
        String lower = fname;
        lower.toLowerCase();
        if (lower.endsWith(".txt") || lower.endsWith(".bin")) {
          files.push_back({fname, false});
        }
      }
    }
    file = root.openNextFile();
  }
  root.close();

  // Sort alphabetically
  std::sort(dirs.begin(), dirs.end(), [](const FileEntry &a, const FileEntry &b) {
    return a.name < b.name;
  });
  std::sort(files.begin(), files.end(), [](const FileEntry &a, const FileEntry &b) {
    return a.name < b.name;
  });

  // Merge: folders first, then files
  for (auto &d : dirs)  entries.push_back(d);
  for (auto &f : files) entries.push_back(f);

  Serial.printf("[DIR] %s → %d items\n", path.c_str(), (int)entries.size());

  // Reset header scroll animation for the new path
  resetHeaderScroll(currentPath);
  // Reset selected-item scroll for the entry now under the cursor (index 0)
  resetSelectedItemScroll();
}

// ──────────────────────────────────────────────
//  TRUNCATE STRING to fit within maxPixels
// ──────────────────────────────────────────────
String truncateString(const String &str, int maxPixels) {
  if ((int)u8g2.getStrWidth(str.c_str()) <= maxPixels) return str;
  String truncated = str;
  while (truncated.length() > 1) {
    truncated.remove(truncated.length() - 1);
    String test = truncated + "..";
    if ((int)u8g2.getStrWidth(test.c_str()) <= maxPixels) {
      return test;
    }
  }
  return "..";
}

// ──────────────────────────────────────────────
//  HEADER SCROLL HELPERS
// ──────────────────────────────────────────────
// Call this whenever the header text changes (new directory or file opened)
void resetHeaderScroll(const String &text) {
  u8g2.setFont(u8g2_font_5x7_tr);
  headerTextWidth    = u8g2.getStrWidth(text.c_str());
  headerNeedsScroll  = (headerTextWidth > HEADER_MAX_W);
  headerScrollOffset = 0;
  headerLastStep     = millis();
  headerPauseUntil   = millis() + HEADER_SCROLL_PAUSE;
}

// Called every loop iteration — advances the scroll offset only.
// Returns true if the offset changed (caller decides when to redraw, so a
// single redraw can absorb multiple scroll updates in the same iteration).
bool updateHeaderScroll() {
  if (!headerNeedsScroll) return false;

  unsigned long now = millis();
  if (now < headerPauseUntil) return false;           // still in pause
  if (now - headerLastStep < HEADER_SCROLL_SPEED) return false;  // too early

  headerLastStep = now;
  headerScrollOffset++;

  // When the text has fully scrolled off + gap, reset and pause
  if (headerScrollOffset >= headerTextWidth + HEADER_SCROLL_GAP) {
    headerScrollOffset = 0;
    headerPauseUntil = now + HEADER_SCROLL_PAUSE;
  }

  return true;
}

// ──────────────────────────────────────────────
//  SELECTED-ITEM SCROLL HELPERS
// ──────────────────────────────────────────────
// Recompute scroll state for the currently selected entry. Call whenever the
// cursor moves or the entries list changes.
void resetSelectedItemScroll() {
  if (entries.empty() || cursorIndex < 0 || cursorIndex >= (int)entries.size()) {
    itemNeedsScroll = false;
    itemScrollOffset = 0;
    return;
  }
  String fullName = entries[cursorIndex].isDir
                    ? entries[cursorIndex].name
                    : stripExtension(entries[cursorIndex].name);
  u8g2.setFont(u8g2_font_6x10_tr);
  itemTextWidth    = u8g2.getStrWidth(fullName.c_str());
  itemNeedsScroll  = (itemTextWidth > ITEM_MAX_W);
  itemScrollOffset = 0;
  itemLastStep     = millis();
  itemPauseUntil   = millis() + HEADER_SCROLL_PAUSE;
}

bool updateItemScroll() {
  if (!itemNeedsScroll) return false;

  unsigned long now = millis();
  if (now < itemPauseUntil) return false;
  if (now - itemLastStep < HEADER_SCROLL_SPEED) return false;

  itemLastStep = now;
  itemScrollOffset++;

  if (itemScrollOffset >= itemTextWidth + HEADER_SCROLL_GAP) {
    itemScrollOffset = 0;
    itemPauseUntil = now + HEADER_SCROLL_PAUSE;
  }

  return true;
}

// ──────────────────────────────────────────────
//  STRIP FILE EXTENSION for display
// ──────────────────────────────────────────────
String stripExtension(const String &name) {
  int dot = name.lastIndexOf('.');
  if (dot > 0) return name.substring(0, dot);
  return name;
}

// ──────────────────────────────────────────────
//  RECALCULATE TICK FROM RPM
// ──────────────────────────────────────────────
//   step_time = 60_000_000 / (RPM * total_pulses)
// total_pulses (signalTotalStepsF) is the length of one pass of the file, in
// steps: the span of its rows (digital) or of its segments (analog). The whole
// waveform spans one crank revolution at the requested RPM, so:
//   one_rev_time   = 60_000_000 / RPM            (microseconds)
//   step_time      = one_rev_time / total_pulses
// NOTE: no "cycles" factor here. The file already contains the full per-cycle
// waveform, so multiplying by a cycle count made playback that many times too
// slow (2 cycles -> ~2x slow, 4 cycles -> ~4x slow).
void recalcTickFromRPM() {
  if (currentRPM == 0) return;
  if (signalTotalStepsF < 1e-3f) return;

  float period = 60000000.0f
                 / ((float)currentRPM * signalTotalStepsF);
  if (period < 0.001f) period = 0.001f;
  tickUs = period;   // keep fractional microseconds — no truncation
}

// One encoder detent = one RPM_STEP, clamped to RPM_MIN .. RPM_MAX.
void stepRPM(int ticks) {
  if (ticks > 0) {
    if (currentRPM <= RPM_MAX - RPM_STEP) {
      currentRPM += RPM_STEP;
    } else {
      currentRPM = RPM_MAX;
    }
  } else if (ticks < 0) {
    if (currentRPM >= RPM_MIN + RPM_STEP) {
      currentRPM -= RPM_STEP;
    } else {
      currentRPM = RPM_MIN;
    }
  }
}

// "Digital 4ch" / "Analog 1ch" — shown under the RPM.
String modeLabel() {
  if (signalMode == MODE_ANALOG) return "Analog " + String(anaChannels) + "ch";
  return "Digital " + String(NUM_CHANNELS) + "ch";
}

// ──────────────────────────────────────────────
//  DRAW BROWSER
// ──────────────────────────────────────────────
void drawBrowser() {
  if (!oledPresent) return;
  u8g2.clearBuffer();

  // ── Header: black background, white text with scrolling path + divider ──
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1);

  if (headerNeedsScroll) {
    u8g2.setClipWindow(0, 0, HEADER_MAX_W + 2, HEADER_H - 1);
    u8g2.drawStr(2 - headerScrollOffset, 7, currentPath.c_str());
    u8g2.drawStr(2 - headerScrollOffset + headerTextWidth + HEADER_SCROLL_GAP, 7, currentPath.c_str());
    u8g2.setMaxClipWindow();
  } else {
    u8g2.drawStr(2, 7, currentPath.c_str());
  }

  u8g2.drawHLine(0, HEADER_H - 1, 128);  // white divider

  if (entries.empty()) {
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(10, 36, "(empty)");
    u8g2.sendBuffer();
    return;
  }

  // ── Draw visible items ──
  u8g2.setFont(u8g2_font_6x10_tr);

  for (int i = 0; i < VISIBLE_ITEMS; i++) {
    int idx = scrollOffset + i;
    if (idx >= (int)entries.size()) break;

    int y = HEADER_H + i * ITEM_H;

    // Highlight selected item
    if (idx == cursorIndex) {
      u8g2.drawBox(0, y, 128, ITEM_H);
      u8g2.setDrawColor(0);
    } else {
      u8g2.setDrawColor(1);
    }

    // Icon
    if (entries[idx].isDir) {
      u8g2.drawXBMP(1, y + 1, ICON_W, ICON_H, icon_folder);
    } else {
      u8g2.drawXBMP(1, y + 1, ICON_W, ICON_H, icon_file);
    }

    // Name — selected row scrolls horizontally if too wide; others truncate.
    String fullName = entries[idx].isDir ? entries[idx].name : stripExtension(entries[idx].name);
    if (idx == cursorIndex && itemNeedsScroll) {
      u8g2.setClipWindow(ITEM_TEXT_X, y, ITEM_TEXT_X + ITEM_MAX_W, y + ITEM_H);
      u8g2.drawStr(ITEM_TEXT_X - itemScrollOffset, y + 10, fullName.c_str());
      u8g2.drawStr(ITEM_TEXT_X - itemScrollOffset + itemTextWidth + HEADER_SCROLL_GAP,
                   y + 10, fullName.c_str());
      u8g2.setMaxClipWindow();
    } else {
      String displayName = truncateString(fullName, ITEM_MAX_W);
      u8g2.drawStr(ITEM_TEXT_X, y + 10, displayName.c_str());
    }

    u8g2.setDrawColor(1);
  }

  // ── Scrollbar ──
  if ((int)entries.size() > VISIBLE_ITEMS) {
    int barH     = max(4, (int)(64 - HEADER_H) * VISIBLE_ITEMS / (int)entries.size());
    int barRange = (64 - HEADER_H) - barH;
    int barY     = HEADER_H + (int)((long)barRange * scrollOffset / max(1, (int)entries.size() - VISIBLE_ITEMS));
    u8g2.drawBox(126, barY, 2, barH);
  }

  u8g2.sendBuffer();
}

// ──────────────────────────────────────────────
//  READ FILE CONTENT (into fileLines vector)
// ──────────────────────────────────────────────
void readFileContent(const String &path) {
  fileLines.clear();
  viewScrollY = 0;

  File f = SD.open(path, FILE_READ);
  if (!f) {
    fileLines.push_back("Error opening file");
    totalFileLines = 1;
    return;
  }

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    fileLines.push_back(line);
  }
  f.close();
  totalFileLines = (int)fileLines.size();
  if (totalFileLines == 0) {
    fileLines.push_back("(empty file)");
    totalFileLines = 1;
  }
  Serial.printf("[FILE] %s → %d lines\n", path.c_str(), totalFileLines);
}

// ──────────────────────────────────────────────
//  DRAW FILE VIEWER
// ──────────────────────────────────────────────
void drawFileViewer() {
  if (!oledPresent) return;
  u8g2.clearBuffer();

  // ── Header: black background, white text with file name + divider ──
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1);
  String fname = entries.empty() ? "file" : stripExtension(entries[cursorIndex].name);

  if (headerNeedsScroll) {
    u8g2.setClipWindow(0, 0, HEADER_MAX_W + 2, HEADER_H - 1);
    u8g2.drawStr(2 - headerScrollOffset, 7, fname.c_str());
    u8g2.drawStr(2 - headerScrollOffset + headerTextWidth + HEADER_SCROLL_GAP, 7, fname.c_str());
    u8g2.setMaxClipWindow();
  } else {
    u8g2.drawStr(2, 7, fname.c_str());
  }

  u8g2.drawHLine(0, HEADER_H - 1, 128);  // white divider

  // ── Content ──
  u8g2.setFont(u8g2_font_5x7_tr);
  int lineH = 9;
  int linesVisible = (64 - HEADER_H) / lineH;

  for (int i = 0; i < linesVisible; i++) {
    int lineIdx = viewScrollY + i;
    if (lineIdx >= totalFileLines) break;
    int y = HEADER_H + i * lineH + 7;
    // Truncate long lines
    String display = truncateString(fileLines[lineIdx], 124);
    u8g2.drawStr(2, y, display.c_str());
  }

  // ── Scrollbar ──
  if (totalFileLines > linesVisible) {
    int barH     = max(4, (int)(64 - HEADER_H) * linesVisible / totalFileLines);
    int barRange = (64 - HEADER_H) - barH;
    int barY     = HEADER_H + barRange * viewScrollY / max(1, totalFileLines - linesVisible);
    u8g2.drawBox(126, barY, 2, barH);
  }

  u8g2.sendBuffer();
}

// ──────────────────────────────────────────────
//  FILE METADATA (shared by both modes)
// ──────────────────────────────────────────────
// The exporter writes metadata at the end of the file, after a blank line:
//   VOLTAGE: 5
//   STEP_TIME (us): 1000
//   CYCLES: 2
//   RPM: 800
//   RESOLUTION (bits): 12        ← analog files only
// Returns true if the line was a metadata line (and consumed).
bool parseMetaLine(const String &line) {
  int colonIdx = line.indexOf(':');
  if (colonIdx < 0) return false;
  String val = line.substring(colonIdx + 1);
  val.trim();

  if (line.startsWith("VOLTAGE")) {
    metaVoltage = val.toFloat();
    Serial.printf("[SIG] Metadata VOLTAGE: %.2f\n", metaVoltage);
    return true;
  }
  if (line.startsWith("STEP_TIME")) {
    // STEP_TIME is the per-step microseconds the file was authored with. The
    // playback tick is computed from RPM, but we read this so we can compare
    // it against our computed tickUs and diagnose timing mismatches.
    metaStepTime = val.toFloat();
    Serial.printf("[SIG] Metadata STEP_TIME: %.4f us\n", metaStepTime);
    return true;
  }
  if (line.startsWith("CYCLES")) {
    metaCycles = val.toInt();
    Serial.printf("[SIG] Metadata CYCLES: %d\n", metaCycles);
    return true;
  }
  if (line.startsWith("RPM")) {
    metaRPM = val.toInt();
    Serial.printf("[SIG] Metadata RPM: %d\n", metaRPM);
    return true;
  }
  if (line.startsWith("RESOLUTION")) {
    metaResolution = val.toInt();
    Serial.printf("[SIG] Metadata RESOLUTION: %d bits\n", metaResolution);
    return true;
  }
  return false;
}

// Analog exports always carry "RESOLUTION (bits)"; digital ones never do.
bool isAnalogFile(const String &path) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  bool analog = false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("RESOLUTION")) { analog = true; break; }
  }
  f.close();
  return analog;
}

// ──────────────────────────────────────────────
//  PARSE SIGNAL FILE (new sparse format with sub-step support)
// ──────────────────────────────────────────────
// File only contains rows where channels change.
// First line is header with channel names.
// Each data row: step_number <TAB> ch0 <TAB> ch1 ...
// Step numbers can be fractional (e.g. 14.333333) for sub-steps.
// Duration of each event = (next_step - this_step) as a float.
// At playback, each duration is multiplied by tickUs to get microseconds.
// This naturally handles any mix of subdivisions (1/3, 1/4, etc.).
// Returns true if successfully parsed.
bool parseSignalFile(const String &path) {
  freeAnaBuffers();
  signalEvents.clear();
  signalTotalStepsF = 0.0f;
  signalTotalEvents = 0;

  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.println(F("[SIG] Cannot open file"));
    return false;
  }

  // Skip header line — channel assignments are fixed (CKP, CMP 1..3)
  if (f.available()) {
    String header = f.readStringUntil('\n');
    header.trim();
    Serial.printf("[SIG] Skipping header: \"%s\"\n", header.c_str());
  }

  // Read all data rows into a temporary list (float step + values)
  struct RawEvent {
    float   stepF;               // original fractional step from file
    uint8_t val[NUM_CHANNELS];
  };
  std::vector<RawEvent> rawEvents;

  // Reset metadata to defaults
  metaVoltage    = 0.0f;
  metaCycles     = 0;
  metaRPM        = 0;
  metaStepTime   = 0.0f;
  metaResolution = 0;

  int skippedLines = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // ── Check for metadata lines ──
    if (parseMetaLine(line)) continue;

    // ── Tokenize by TAB or consecutive spaces ──
    String fields[10];
    int fieldCount = 0;
    int len = line.length();
    int i = 0;
    while (i < len && fieldCount < 10) {
      while (i < len && (line.charAt(i) == ' ' || line.charAt(i) == '\t')) i++;
      if (i >= len) break;
      int start = i;
      if (line.indexOf('\t') >= 0) {
        while (i < len && line.charAt(i) != '\t') i++;
      } else {
        while (i < len && line.charAt(i) != ' ' && line.charAt(i) != '\t') i++;
      }
      fields[fieldCount] = line.substring(start, i);
      fields[fieldCount].trim();
      fieldCount++;
      i++;
    }

    if (fieldCount < 2) {
      skippedLines++;
      if (skippedLines <= 5) {
        Serial.printf("[SIG] SKIP line (only %d fields): \"%s\"\n", fieldCount, line.c_str());
      }
      continue;
    }

    RawEvent ev;
    ev.stepF = fields[0].toFloat();   // parse as float to preserve fractions
    for (int c = 0; c < NUM_CHANNELS; c++) {
      ev.val[c] = ((c + 1) < fieldCount && fields[c + 1].toInt() != 0) ? 1 : 0;
    }
    rawEvents.push_back(ev);
  }
  f.close();

  if (rawEvents.empty()) {
    Serial.println(F("[SIG] No data rows found"));
    return false;
  }

  if (skippedLines > 0) {
    Serial.printf("[SIG] WARNING: %d lines skipped (bad format)\n", skippedLines);
  }

  // ── Build events: duration = float difference between consecutive steps ──
  // Every event holds its values from its own step until the NEXT step. The
  // duration of the final event therefore depends on where the cycle wraps,
  // which the event list alone does not encode.
  //
  // Detect an end-of-cycle marker: a final row whose channel values match the
  // first row's values marks the step at which the waveform repeats. That step
  // is the cycle period, not a state to hold — so it is dropped as an event and
  // its position gives the real last event its true duration. This removes the
  // old "copy the previous duration" guess, which overran the cycle by ~1 step
  // and made playback run ~1% fast.
  int numRaw = (int)rawEvents.size();

  bool hasEndMarker = (numRaw >= 2);
  for (int c = 0; c < NUM_CHANNELS && hasEndMarker; c++) {
    if (rawEvents[numRaw - 1].val[c] != rawEvents[0].val[c]) hasEndMarker = false;
  }

  // Finest grid step — used only as the last-event duration when no wrap marker
  // is present (the period is then unknown and this is the best estimate).
  float gridStep = 1.0f;
  {
    float minDiff = 1e30f;
    for (int i = 1; i < numRaw; i++) {
      float d = rawEvents[i].stepF - rawEvents[i - 1].stepF;
      if (d > 1e-6f && d < minDiff) minDiff = d;
    }
    if (minDiff < 1e29f) gridStep = minDiff;
  }

  // Number of real (playable) events: drop the trailing marker row if present.
  int numEvents = hasEndMarker ? (numRaw - 1) : numRaw;

  for (int i = 0; i < numEvents; i++) {
    SignalEvent se;
    se.stepF = rawEvents[i].stepF;
    for (int c = 0; c < NUM_CHANNELS; c++) {
      se.val[c] = rawEvents[i].val[c];
    }

    if (i < numRaw - 1) {
      // Duration in steps (fractional) = next step position - this step position.
      // For the last real event with a marker, rawEvents[i + 1] IS the marker,
      // so this yields the correct wrap duration.
      se.durationF = rawEvents[i + 1].stepF - rawEvents[i].stepF;
    } else {
      // No end marker: period unknown, assume one grid step until the wrap.
      se.durationF = gridStep;
    }

    if (se.durationF < 1e-6f) se.durationF = 1e-6f;  // safety

    signalEvents.push_back(se);
  }

  Serial.printf("[SIG] End-of-cycle marker %s\n",
                hasEndMarker ? "detected (dropped as event)" : "not found");

  signalTotalEvents = (int)signalEvents.size();

  // Cycle period in steps = span actually covered by the file's rows
  // (last step − first step). This is the number of real step INTERVALS, which
  // is what the per-step time must divide. Summing event durations instead would
  // add the last event's synthetic wrap duration (one extra grid step when there
  // is no end marker), inflating the period by one step and making every pulse
  // ~1/(steps) too short — e.g. 150 vs 149 steps ≈ 5 us/pulse fast at 500 RPM.
  // For a file WITH an end marker, rawEvents.back() is the marker, so this still
  // yields markerStep − firstStep (unchanged from before).
  signalTotalStepsF = rawEvents.back().stepF - rawEvents.front().stepF;
  if (signalTotalStepsF < 1e-3f) {
    // Degenerate (single row or all-equal steps): fall back to summed durations.
    signalTotalStepsF = 0.0f;
    for (int i = 0; i < signalTotalEvents; i++) signalTotalStepsF += signalEvents[i].durationF;
  }

  // Debug output
  Serial.printf("[SIG] Parsed %d events, %.2f total steps per cycle\n",
                signalTotalEvents, signalTotalStepsF);
  for (int i = 0; i < signalTotalEvents && i < 30; i++) {
    Serial.printf("[SIG] Event %d: step=%.4f dur=%.6f vals=[%d %d %d %d]\n",
                  i, signalEvents[i].stepF, signalEvents[i].durationF,
                  signalEvents[i].val[0], signalEvents[i].val[1],
                  signalEvents[i].val[2], signalEvents[i].val[3]);
  }

  // Per-channel statistics
  for (int c = 0; c < NUM_CHANNELS; c++) {
    float highSteps = 0.0f;
    for (int i = 0; i < signalTotalEvents; i++) {
      if (signalEvents[i].val[c]) highSteps += signalEvents[i].durationF;
    }
    Serial.printf("[SIG] CH%d (%s): %.3f high steps out of %.2f\n",
                  c, channelNames[c].c_str(), highSteps, signalTotalStepsF);
  }

  // Metadata summary
  Serial.printf("[SIG] Metadata → VOLTAGE: %.2f, CYCLES: %d, RPM: %d\n",
                metaVoltage, metaCycles, metaRPM);

  signalMode = MODE_DIGITAL;
  return (signalTotalEvents > 0);
}

// ──────────────────────────────────────────────
//  ANALOG CURVE GEOMETRY
// ──────────────────────────────────────────────
// A C++ port of the editor's chord geometry (src/app/components/analogSignal.ts:
// chordPieces, roundCorners, pieceYAt). Keep it in sync with that file: the
// point is that the DAC draws exactly the curve that is on screen.
//
// A channel's path is a chain of pieces, each a straight line or a quadratic
// with one control point, in signal coordinates: x in steps, y normalized
// (0 = floor, 1 = ceiling). x never goes backwards along the chain.

static inline double clamp01d(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }

static inline bool samePt(const APt &a, const APt &b) {
  return fabs(a.x - b.x) < 1e-9 && fabs(a.y - b.y) < 1e-9;
}

static inline APt lerpPt(const APt &a, const APt &b, double u) {
  return { a.x + (b.x - a.x) * u, a.y + (b.y - a.y) * u };
}

static inline APt quadAt(const APt &p0, const APt &k, const APt &p1, double u) {
  double iu = 1 - u;
  return { iu * iu * p0.x + 2 * iu * u * k.x + u * u * p1.x,
           iu * iu * p0.y + 2 * iu * u * k.y + u * u * p1.y };
}

static inline APiece linePiece(const APt &to) {
  APiece p;
  p.to = to; p.ctrl = to; p.quad = false; p.corner = 0;
  return p;
}

static inline APiece quadPiece(const APt &to, const APt &ctrl) {
  APiece p;
  p.to = to; p.ctrl = ctrl; p.quad = true; p.corner = 0;
  return p;
}

// Chord radius → (q, f):  0 → peak · 0.5 → round · 1 → square pulse.
static void radiusShape(double r, double &q, double &f) {
  double c = clamp01d(r);
  q = c;
  f = c <= 0.5 ? 1 : 2 * (1 - c);
}

// Pieces of one chord from `start` to `end`, bent through the handle (gx, gy).
static void chordPieces(const APt &start, const APt &end, double t, double gy, double r,
                        std::vector<APiece> &out) {
  double gx = start.x + t * (end.x - start.x);

  double lineY = start.y + (end.y - start.y) * t;
  if (fabs(gy - lineY) < 1e-6) { out.push_back(linePiece(end)); return; }

  double q, f;
  radiusShape(r, q, f);
  if (q <= 0) {
    out.push_back(linePiece({ gx, gy }));
    out.push_back(linePiece(end));
    return;
  }

  // Half chord: vertex K on the handle's horizontal, moved q toward the end
  // point; A is where the straight part hands over to the curve.
  double akx = gx + (start.x - gx) * q;
  APt aPt = { start.x + (akx - start.x) * (1 - f), start.y + (gy - start.y) * (1 - f) };
  double bkx = gx + (end.x - gx) * q;
  APt bPt = { end.x + (bkx - end.x) * (1 - f), end.y + (gy - end.y) * (1 - f) };

  if (!samePt(aPt, start)) out.push_back(linePiece(aPt));
  out.push_back(quadPiece({ gx, gy }, { akx, gy }));
  out.push_back(quadPiece(bPt, { bkx, gy }));
  if (!samePt(bPt, end)) out.push_back(linePiece(end));
}

// The [a, b] part of a piece, keeping its exact shape.
static void restrictPiece(const APt &from, const APiece &piece, double a, double b,
                          APt &startOut, APiece &pieceOut) {
  pieceOut = piece;
  if (!piece.quad) {
    startOut = lerpPt(from, piece.to, a);
    pieceOut.to = lerpPt(from, piece.to, b);
    pieceOut.ctrl = pieceOut.to;
    return;
  }
  const APt &k = piece.ctrl;
  const APt &p1 = piece.to;
  double w0 = (1 - a) * (1 - b);
  double wk = a * (1 - b) + b * (1 - a);
  double w1 = a * b;
  startOut = quadAt(from, k, p1, a);
  pieceOut.ctrl = { w0 * from.x + wk * k.x + w1 * p1.x, w0 * from.y + wk * k.y + w1 * p1.y };
  pieceOut.to = quadAt(from, k, p1, b);
}

// Rounds the corners at connection points: each corner eats the last r/2 of
// the incoming piece and the first r/2 of the outgoing one, joined by a
// quadratic whose vertex is the connection point.
static void roundCorners(const APt &start, const std::vector<APiece> &in,
                         std::vector<APiece> &out) {
  size_t n = in.size();
  std::vector<double> head(n, 0), tail(n, 0);
  for (size_t i = 0; i + 1 < n; i++) {
    double r = clamp01d(in[i].corner);
    if (r <= 0) continue;
    tail[i] = r / 2;
    head[i + 1] = r / 2;
  }

  out.clear();
  APt from = start;
  for (size_t i = 0; i < n; i++) {
    APt subStart;
    APiece sub;
    restrictPiece(from, in[i], head[i], 1 - tail[i], subStart, sub);
    if (head[i] > 0) out.push_back(quadPiece(subStart, from));
    // A piece bitten from both ends can shrink to nothing: the two corners met.
    if (sub.quad || !samePt(subStart, sub.to)) out.push_back(sub);
    from = in[i].to;
  }
}

// Height of a piece at x. x never goes backwards inside a piece, so bisecting
// the curve parameter lands on it.
static double pieceYAt(const APt &from, const APiece &piece, double x) {
  double span = piece.to.x - from.x;
  if (span == 0) return piece.to.y;
  if (!piece.quad) return from.y + (piece.to.y - from.y) * ((x - from.x) / span);

  double lo = 0, hi = 1;
  for (int i = 0; i < 24; i++) {
    double mid = (lo + hi) / 2;
    if (quadAt(from, piece.ctrl, piece.to, mid).x < x) lo = mid;
    else hi = mid;
  }
  return quadAt(from, piece.ctrl, piece.to, (lo + hi) / 2).y;
}

// Splits a line on spaces/tabs. Returns how many fields were found.
static int splitFields(const String &line, String *fields, int maxFields) {
  int count = 0;
  int len = line.length();
  int i = 0;
  while (i < len && count < maxFields) {
    while (i < len && (line.charAt(i) == ' ' || line.charAt(i) == '\t')) i++;
    if (i >= len) break;
    int start = i;
    while (i < len && line.charAt(i) != ' ' && line.charAt(i) != '\t') i++;
    fields[count++] = line.substring(start, i);
  }
  return count;
}

void freeAnaBuffers() {
  for (int c = 0; c < 2; c++) {
    if (anaSamples[c]) { free(anaSamples[c]); anaSamples[c] = NULL; }
  }
  anaCount = 0;
  anaChannels = 0;
}

// ──────────────────────────────────────────────
//  PARSE ANALOG FILE
// ──────────────────────────────────────────────
// Row: from  to  channel  level0  level1  handleX%  handleY  radius%  cornerR%
// ('-' = not applicable). Rows are sorted by `from`; within a channel they
// come in path order. One pass of the signal runs from the earliest `from` to
// the latest `to` of the channels being played; outside its own range a
// channel holds its edge value.
bool parseAnalogFile(const String &path) {
  freeSigBuffers();
  freeAnaBuffers();
  signalEvents.clear();
  signalTotalStepsF = 0.0f;
  signalTotalEvents = 0;

  File f = SD.open(path, FILE_READ);
  if (!f) {
    Serial.println(F("[ANA] Cannot open file"));
    return false;
  }

  // Header line (column names in the editor's language)
  if (f.available()) {
    String header = f.readStringUntil('\n');
    header.trim();
    Serial.printf("[ANA] Skipping header: \"%s\"\n", header.c_str());
  }

  metaVoltage    = 0.0f;
  metaCycles     = 0;
  metaRPM        = 0;
  metaStepTime   = 0.0f;
  metaResolution = 0;

  std::vector<AnaSeg> segs;
  int skippedLines = 0;
  int highestChannel = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    if (parseMetaLine(line)) continue;

    String fl[9];
    if (splitFields(line, fl, 9) < 9) {
      if (++skippedLines <= 5) Serial.printf("[ANA] SKIP line: \"%s\"\n", line.c_str());
      continue;
    }

    AnaSeg s;
    s.from     = fl[0].toFloat();
    s.to       = fl[1].toFloat();
    int ch     = fl[2].toInt();
    s.l0       = fl[3].toFloat();
    s.l1       = fl[4].toFloat();
    s.straight = (fl[5] == "-");
    s.t        = s.straight ? 0.0f : fl[5].toFloat() / 100.0f;
    s.h        = s.straight ? s.l1 : fl[6].toFloat();
    s.r        = (fl[7] == "-") ? 0.0f : fl[7].toFloat() / 100.0f;
    s.endR     = (fl[8] == "-") ? 0.0f : fl[8].toFloat() / 100.0f;
    if (ch < 1) continue;
    if (ch > highestChannel) highestChannel = ch;
    if (ch > ANA_NUM_CHANNELS) continue;   // no DAC left for it
    s.ch = (uint8_t)ch;
    segs.push_back(s);
  }
  f.close();

  if (skippedLines > 0) {
    Serial.printf("[ANA] WARNING: %d lines skipped (bad format)\n", skippedLines);
  }
  if (highestChannel > ANA_NUM_CHANNELS) {
    Serial.printf("[ANA] WARNING: file has %d channels, only %d DAC output(s) — "
                  "extra channels are not played\n", highestChannel, ANA_NUM_CHANNELS);
  }
  if (segs.empty()) {
    Serial.println(F("[ANA] No segments found"));
    return false;
  }

  int bits = (metaResolution >= 1 && metaResolution <= 16) ? metaResolution : 12;
  double maxLevel = (double)((1UL << bits) - 1);

  // Channels to play: 1 .. highest channel present (capped at the DACs).
  anaChannels = (uint8_t)min(highestChannel, ANA_NUM_CHANNELS);

  // One pass = earliest start to latest end across the played channels.
  double first = 1e30, last = -1e30;
  for (auto &s : segs) {
    if (s.from < first) first = s.from;
    if (s.to > last) last = s.to;
  }
  double span = last - first;
  if (span <= 1e-6) {
    Serial.println(F("[ANA] Signal has no length"));
    anaChannels = 0;
    return false;
  }

  // Sample count: ANA_MAX_SPP per step, capped. Playback indexes samples by
  // time, so the count only sets the shape's resolution, never the speed.
  double want = span * ANA_MAX_SPP;
  anaCount = (uint32_t)min((double)ANA_MAX_SAMPLES, max(2.0, floor(want + 0.5)));

  // Voltage scales the DAC's full range (3.3 V). No value, or one at or above
  // 3.3 V (meant for an external stage), uses the full range.
  double vScale = (metaVoltage > 0.0f && metaVoltage < DAC_FULL_SCALE_V)
                  ? metaVoltage / DAC_FULL_SCALE_V : 1.0;

  for (uint8_t c = 0; c < anaChannels; c++) {
    anaSamples[c] = (uint8_t *)malloc(anaCount);
    if (!anaSamples[c]) {
      Serial.println(F("[ANA] malloc failed for samples"));
      freeAnaBuffers();
      return false;
    }

    // ── Build this channel's path, exactly like the editor ──
    std::vector<APiece> raw;
    APt start = { 0, 0 };
    bool haveStart = false;
    // Before its first point a channel holds that point's outgoing height —
    // the one after the vertical edge, if it starts with one (as the editor).
    double startHold = 0;
    for (auto &s : segs) {
      if (s.ch != c + 1) continue;
      APt a = { s.from, clamp01d(s.l0 / maxLevel) };
      APt b = { s.to,   clamp01d(s.l1 / maxLevel) };
      if (!haveStart) {
        start = a;
        startHold = (s.to == s.from) ? b.y : a.y;
        haveStart = true;
      }

      size_t before = raw.size();
      if (s.to == s.from) {
        // Vertical edge: two heights on the same step line
        if (fabs(b.y - a.y) < 1e-9) continue;
        raw.push_back(linePiece(b));
      } else if (s.straight) {
        raw.push_back(linePiece(b));
      } else {
        chordPieces(a, b, s.t, clamp01d(s.h / maxLevel), s.r, raw);
      }
      // The piece that lands on the next connection point carries its corner.
      if (raw.size() > before) raw.back().corner = s.endR;
    }

    std::vector<APiece> pieces;
    roundCorners(start, raw, pieces);

    // ── Sample it ──
    size_t k = 0;
    for (uint32_t i = 0; i < anaCount; i++) {
      double x = first + span * (double)i / (double)anaCount;
      double y;
      if (!haveStart) {
        y = 0;
      } else if (pieces.empty() || x <= start.x) {
        y = startHold;
      } else {
        while (k < pieces.size() && pieces[k].to.x < x - 1e-9) k++;
        if (k >= pieces.size()) {
          y = pieces.back().to.y;
        } else {
          const APt &from = (k == 0) ? start : pieces[k - 1].to;
          y = pieceYAt(from, pieces[k], x);
        }
      }
      double dac = clamp01d(y) * 255.0 * vScale;
      anaSamples[c][i] = (uint8_t)(dac + 0.5);
    }

    Serial.printf("[ANA] CH%d: %d pieces → %u samples (DAC GPIO %u)\n",
                  c + 1, (int)pieces.size(), (unsigned)anaCount, (unsigned)anaPins[c]);
  }

  signalTotalStepsF = (float)span;
  signalTotalEvents = (int)segs.size();
  signalMode = MODE_ANALOG;

  Serial.printf("[ANA] Parsed %d segments, %.2f steps per cycle, %d bits, %.2f samples/step\n",
                signalTotalEvents, signalTotalStepsF, bits, (double)anaCount / span);
  Serial.printf("[ANA] Metadata → VOLTAGE: %.2f, CYCLES: %d, RPM: %d\n",
                metaVoltage, metaCycles, metaRPM);
  return true;
}

// ──────────────────────────────────────────────
//  DRAW SIGNAL CONFIGURATION SCREEN
// ──────────────────────────────────────────────
void drawSigConfig() {
  if (!oledPresent) return;
  u8g2.clearBuffer();

  // Header: black background, white text with divider
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1);
  const char *title = "Signal Configuration";
  int titleW = u8g2.getStrWidth(title);
  u8g2.drawStr(64 - titleW / 2, 7, title);
  u8g2.drawHLine(0, HEADER_H - 1, 128);

  u8g2.setFont(u8g2_font_6x10_tr);

  // Signal name (no extension), centered
  String sigName = stripExtension(signalFileName);
  sigName = truncateString(sigName, 124);
  int nameW = u8g2.getStrWidth(sigName.c_str());
  u8g2.drawStr(64 - nameW / 2, 26, sigName.c_str());

  // Cycles selector (highlighted – encoder adjusts this)
  String cycStr = "Cycles: " + String(signalCycles);
  int cycW = u8g2.getStrWidth(cycStr.c_str());
  int boxX = 64 - cycW / 2 - 4;
  int boxW = cycW + 8;
  u8g2.drawRFrame(boxX, 34, boxW, 14, 2);
  u8g2.drawStr(64 - cycW / 2, 45, cycStr.c_str());

  // Hint
  u8g2.setFont(u8g2_font_5x7_tr);
  const char *hint = "Click to start";
  int hintW = u8g2.getStrWidth(hint);
  u8g2.drawStr(64 - hintW / 2, 60, hint);

  u8g2.sendBuffer();
}

// ──────────────────────────────────────────────
//  DRAW PLAYING SCREEN
// ──────────────────────────────────────────────
void drawPlaying() {
  if (!oledPresent) return;
  u8g2.clearBuffer();

  // Header: black background, white text with divider
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1);
  const char *title = isPaused ? "Paused"
                    : (signalMode == MODE_ANALOG ? "Playing Analog" : "Playing Signal");
  int titleW = u8g2.getStrWidth(title);
  u8g2.drawStr(64 - titleW / 2, 7, title);
  u8g2.drawHLine(0, HEADER_H - 1, 128);

  u8g2.setFont(u8g2_font_6x10_tr);

  // Signal name (no extension), centered
  String sigName = stripExtension(signalFileName);
  sigName = truncateString(sigName, 124);
  int sigW = u8g2.getStrWidth(sigName.c_str());
  u8g2.drawStr(64 - sigW / 2, 22, sigName.c_str());

  // RPM display — rounded frame, text centered
  String rpmStr = String(currentRPM) + " RPM";
  int rpmW = u8g2.getStrWidth(rpmStr.c_str());
  int boxX = 64 - rpmW / 2 - 4;
  int boxW = rpmW + 8;
  u8g2.drawRFrame(boxX, 28, boxW, 14, 2);
  u8g2.drawStr(64 - rpmW / 2, 39, rpmStr.c_str());

  // Cycles info centered
  String cycStr = "Cycles: " + String(signalCycles);
  int cycW = u8g2.getStrWidth(cycStr.c_str());
  u8g2.drawStr(64 - cycW / 2, 52, cycStr.c_str());

  // Hint centered
  u8g2.setFont(u8g2_font_5x7_tr);
  const char *hint = isPaused ? "Click:resume  Hold:exit"
                               : "Click:pause   Hold:exit";
  int hintW = u8g2.getStrWidth(hint);
  u8g2.drawStr(64 - hintW / 2, 62, hint);

  u8g2.sendBuffer();
}

// ──────────────────────────────────────────────
//  SIGNAL OUTPUT — Dedicated core 0 task with busy-wait
// ──────────────────────────────────────────────
// A FreeRTOS task pinned to Core 0 runs a tight loop:
//   1. Write GPIOs for current event
//   2. Busy-wait for (event.durationF * tickUs) microseconds
//   3. Advance to next event (wrap to 0 at end)
// This guarantees zero jitter — no ISR latency, no I2C interference.
// Arduino loop() runs on Core 1 so both run independently.

// Flat arrays for task-safe fast access (event-based)
float    *evtDurF       = NULL;  // duration in steps (fractional) per event
uint32_t *evtSetMasks   = NULL;  // GPIO set masks per event
uint32_t *evtClrMasks   = NULL;  // GPIO clear masks per event
volatile int evtCount    = 0;    // number of events

TaskHandle_t sigTaskHandle = NULL;

// The signal generator task — runs on Core 0
// Reads volatile tickUs each event so RPM changes take effect immediately.
void sigGenTask(void *param) {
  disableCore0WDT();

  // idealUs is the high-precision running target time (µs since boot). We add the
  // exact fractional hold of each event to it and round only when deriving the
  // integer microsecond deadline. This keeps sub-µs fractions from being thrown
  // away every step (which made playback drift fast) and prevents accumulation.
  double  idealUs  = (double)esp_timer_get_time();   // absolute time reference
  int64_t nextTick = (int64_t)idealUs;

  while (isPlaying) {
    // Pause handling: drive all outputs LOW and idle until resume or stop.
    // On resume, realign the target clock so the next event starts immediately.
    if (isPaused) {
      GPIO.out_w1tc = ALL_CH_MASK;
      while (isPaused && isPlaying) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      idealUs  = (double)esp_timer_get_time();
      nextTick = (int64_t)idealUs;
      if (!isPlaying) break;
    }

    uint32_t evtIdx = playbackStep;

    // Wrap at end for continuous loop
    if (evtIdx >= (uint32_t)evtCount) {
      evtIdx = 0;
      loopCount++;
    }

    // Write all 4 channels simultaneously via pre-computed masks
    uint32_t sm = evtSetMasks[evtIdx];
    uint32_t cm = evtClrMasks[evtIdx];
    if (sm) GPIO.out_w1ts = sm;
    if (cm) GPIO.out_w1tc = cm;

    // Advance the precise target by this event's exact hold (read volatile tickUs
    // each event so RPM changes take effect immediately), then round the absolute
    // deadline to the nearest microsecond. Rounding the cumulative target — not
    // each increment — means per-event rounding never accumulates into drift.
    idealUs += (double)evtDurF[evtIdx] * (double)tickUs;
    nextTick = (int64_t)(idealUs + 0.5);

    playbackStep = evtIdx + 1;

    // Busy-wait until the ABSOLUTE target time
    while (esp_timer_get_time() < nextTick) {
      // tight spin — no yield, no sleep
    }
  }

  // Done — set all outputs LOW
  GPIO.out_w1tc = ALL_CH_MASK;

  enableCore0WDT();

  sigTaskHandle = NULL;
  vTaskDelete(NULL);   // self-delete
}

// Analog generator task — also on Core 0.
// Instead of stepping sample by sample, it follows the clock: every pass it
// measures the elapsed time, advances the position by elapsed / tickUs steps
// and writes the sample that position falls on. The DAC can't keep up with
// every sample at high RPM (dacWrite takes a few µs), so some get skipped —
// but a revolution always lasts exactly 60 000 000 / RPM µs. Reading tickUs
// every pass makes RPM changes take effect immediately, without a jump.
void anaGenTask(void *param) {
  disableCore0WDT();

  const double total   = (double)signalTotalStepsF;
  const double perStep = (double)anaCount / total;   // samples per step
  double   pos     = 0.0;                            // position in steps
  int64_t  prev    = esp_timer_get_time();
  uint32_t lastIdx = UINT32_MAX;

  while (isPlaying) {
    if (isPaused) {
      for (uint8_t c = 0; c < anaChannels; c++) dacWrite(anaPins[c], 0);
      while (isPaused && isPlaying) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      prev = esp_timer_get_time();   // resume where it paused
      lastIdx = UINT32_MAX;
      if (!isPlaying) break;
    }

    int64_t now = esp_timer_get_time();
    pos += (double)(now - prev) / (double)tickUs;
    prev = now;
    while (pos >= total) { pos -= total; loopCount = loopCount + 1; }

    uint32_t idx = (uint32_t)(pos * perStep);
    if (idx >= anaCount) idx = anaCount - 1;
    if (idx != lastIdx) {
      for (uint8_t c = 0; c < anaChannels; c++) dacWrite(anaPins[c], anaSamples[c][idx]);
      lastIdx = idx;
    }
  }

  for (uint8_t c = 0; c < anaChannels; c++) dacWrite(anaPins[c], 0);

  enableCore0WDT();

  sigTaskHandle = NULL;
  vTaskDelete(NULL);   // self-delete
}

void freeSigBuffers() {
  if (evtDurF)      { free(evtDurF);      evtDurF      = NULL; }
  if (evtSetMasks)  { free(evtSetMasks);  evtSetMasks  = NULL; }
  if (evtClrMasks)  { free(evtClrMasks);  evtClrMasks  = NULL; }
}

// Build flat arrays from signalEvents and pre-compute GPIO masks + store durations
bool prepareSigBuffers() {
  freeSigBuffers();

  int n = signalTotalEvents;
  evtDurF      = (float    *)malloc(n * sizeof(float));
  evtSetMasks  = (uint32_t *)malloc(n * sizeof(uint32_t));
  evtClrMasks  = (uint32_t *)malloc(n * sizeof(uint32_t));

  if (!evtDurF || !evtSetMasks || !evtClrMasks) {
    Serial.println(F("[SIG] malloc failed for event buffers"));
    freeSigBuffers();
    return false;
  }

  for (int i = 0; i < n; i++) {
    // Store fractional duration — actual µs computed on-the-fly using volatile tickUs
    evtDurF[i] = signalEvents[i].durationF;

    uint32_t sm = 0, cm = 0;
    for (int c = 0; c < NUM_CHANNELS; c++) {
      uint32_t pin = sigPins[c];
      if (signalEvents[i].val[c]) {
        sm |= (1UL << pin);
      } else {
        cm |= (1UL << pin);
      }
    }
    evtSetMasks[i] = sm;
    evtClrMasks[i] = cm;
  }

  evtCount = n;
  Serial.printf("[SIG] Pre-computed %d event GPIO masks + durations\n", n);
  return true;
}

// True if `pin` is a DAC output in use by the loaded analog signal.
static bool isActiveDacPin(gpio_num_t pin) {
  if (signalMode != MODE_ANALOG) return false;
  for (uint8_t c = 0; c < anaChannels; c++) {
    if ((uint8_t)pin == anaPins[c]) return true;
  }
  return false;
}

void initOutputPins() {
  for (int c = 0; c < NUM_CHANNELS; c++) {
    if (isActiveDacPin(sigPins[c])) continue;   // the DAC drives this one
    gpio_reset_pin(sigPins[c]);
    gpio_set_direction(sigPins[c], GPIO_MODE_OUTPUT);
    gpio_set_level(sigPins[c], 0);
  }
  for (uint8_t c = 0; c < anaChannels && signalMode == MODE_ANALOG; c++) {
    dacWrite(anaPins[c], 0);
  }
  Serial.println(F("[GPIO] Output pins initialized"));
}

void startPlayback() {
  if (isPlaying) return;

  // Stop any previous task
  if (sigTaskHandle != NULL) {
    isPlaying = false;
    vTaskDelay(pdMS_TO_TICKS(10));  // let task exit
    if (sigTaskHandle != NULL) {
      vTaskDelete(sigTaskHandle);
      sigTaskHandle = NULL;
    }
  }

  initOutputPins();

  if (signalMode == MODE_DIGITAL) {
    if (!prepareSigBuffers()) {
      Serial.println(F("[PLAY] Buffer allocation failed!"));
      return;
    }
  } else if (anaCount == 0 || anaChannels == 0) {
    Serial.println(F("[PLAY] No analog samples loaded!"));
    return;
  }

  // Compute initial tickUs from current RPM
  recalcTickFromRPM();

  // ── DIAGNOSTIC: compare our computed per-step time against the file's STEP_TIME ──
  // If these differ, the 4-5 us/pulse offset comes from our RPM->tick model (our
  // step count / formula), not from jitter. If they match, the offset is in the
  // signal source. The RPM implied by the file's STEP_TIME is also printed so you
  // can see what RPM reproduces the original exactly.
  Serial.println(F("[DIAG] ───────── timing check ─────────"));
  Serial.printf("[DIAG] mode              : %s\n", modeLabel().c_str());
  Serial.printf("[DIAG] total steps/cycle : %.4f\n", signalTotalStepsF);
  Serial.printf("[DIAG] RPM (set)         : %u\n", (unsigned)currentRPM);
  Serial.printf("[DIAG] tickUs (computed) : %.4f us/step\n", (float)tickUs);
  Serial.printf("[DIAG] STEP_TIME (file)  : %.4f us/step\n", metaStepTime);
  if (metaStepTime > 1e-4f) {
    float diff = (float)tickUs - metaStepTime;
    Serial.printf("[DIAG] diff (ours-file)  : %.4f us/step (%.3f%%)\n",
                  diff, 100.0f * diff / metaStepTime);
    float impliedRPM = 60000000.0f / (metaStepTime * signalTotalStepsF);
    Serial.printf("[DIAG] RPM to match file : %.2f\n", impliedRPM);
  }
  Serial.println(F("[DIAG] ──────────────────────────────"));

  playbackStep = 0;
  loopCount = 0;
  isPaused = false;
  isPlaying = true;
  appState = STATE_PLAYING;
  drawPlaying();     // draw BEFORE launching task (no I2C during playback)

  // Launch signal task on Core 0 with highest priority
  bool analog = (signalMode == MODE_ANALOG);
  xTaskCreatePinnedToCore(
    analog ? anaGenTask : sigGenTask,   // task function
    analog ? "AnaGen" : "SigGen",       // name
    analog ? 4096 : 2048,               // stack size (dacWrite needs more)
    NULL,              // param
    configMAX_PRIORITIES - 1,  // highest priority
    &sigTaskHandle,    // handle
    0                  // Core 0 (Arduino runs on Core 1)
  );

  Serial.printf("[PLAY] Started on Core 0: %s, %d %s, %u RPM, %u cycles, "
                "tick=%.3f us\n",
                analog ? "analog" : "digital",
                signalTotalEvents, analog ? "segments" : "events",
                (unsigned)currentRPM, (unsigned)signalCycles, (float)tickUs);
}

void stopPlayback() {
  isPlaying = false;   // signal the task to exit
  isPaused  = false;   // make sure a paused task can exit its idle loop

  // Wait for task to finish
  if (sigTaskHandle != NULL) {
    vTaskDelay(pdMS_TO_TICKS(50));
    if (sigTaskHandle != NULL) {
      vTaskDelete(sigTaskHandle);
      sigTaskHandle = NULL;
    }
    enableCore0WDT();
  }

  if (signalMode == MODE_ANALOG) {
    // Hand the DAC pins back to plain GPIO so they sit LOW like the others.
    for (uint8_t c = 0; c < anaChannels; c++) {
      dacWrite(anaPins[c], 0);
      dacDisable(anaPins[c]);
      gpio_reset_pin((gpio_num_t)anaPins[c]);
      gpio_set_direction((gpio_num_t)anaPins[c], GPIO_MODE_OUTPUT);
      gpio_set_level((gpio_num_t)anaPins[c], 0);
    }
  }

  // Set all outputs LOW
  for (int c = 0; c < NUM_CHANNELS; c++) {
    gpio_set_level(sigPins[c], 0);
  }
  freeSigBuffers();
  Serial.printf("[PLAY] Stopped after %u loops\n", (unsigned)loopCount);
}

// ──────────────────────────────────────────────
//  OPEN SELECTED ITEM (click)
// ──────────────────────────────────────────────
void openSelected() {
  if (entries.empty()) return;

  FileEntry &sel = entries[cursorIndex];
  if (sel.isDir) {
    // Navigate into directory
    if (currentPath.endsWith("/")) {
      currentPath += sel.name;
    } else {
      currentPath += "/" + sel.name;
    }
    loadDirectory(currentPath);
    drawBrowser();
  } else {
    // Open .txt file → parse as signal file (digital or analog) → play
    String path = currentPath.endsWith("/") ? currentPath + sel.name
                                            : currentPath + "/" + sel.name;
    if (!openSignalFile(path, sel.name, false)) {
      // Parse failed, fall back to text viewer
      readFileContent(signalFilePath);
      resetHeaderScroll(sel.name);
      appState = STATE_VIEW_FILE;
      drawFileViewer();
    }
  }
}

// Parses a signal file and, as the verified flow did: with CYCLES in the
// metadata it plays right away, otherwise it goes to the cycles screen.
// `autoStart` (serial, or no OLED to show that screen) always plays right
// away, with the default cycles — "cycles <n>" changes them at any time.
// An RPM in the metadata becomes the start speed.
bool openSignalFile(const String &path, const String &name, bool autoStart) {
  signalFilePath = path;
  signalFileName = name;

  // Show loading
  if (oledPresent) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawStr(20, 35, "Parsing...");
    u8g2.sendBuffer();
  }
  Serial.printf("[FILE] Opening %s\n", path.c_str());

  bool analog = isAnalogFile(path);
  bool ok = analog ? parseAnalogFile(path) : parseSignalFile(path);
  if (!ok) {
    Serial.println(F("[FILE] Not a playable signal file"));
    return false;
  }

  if (metaRPM > 0) {
    int rpm = metaRPM;
    if (rpm < RPM_MIN) rpm = RPM_MIN;
    if (rpm > RPM_MAX) rpm = RPM_MAX;
    currentRPM = (uint32_t)rpm;
  }

  if (metaCycles > 0) {
    // If metadata specifies CYCLES, skip config and play directly
    int c = metaCycles;
    if (c < CYCLES_MIN) c = CYCLES_MIN;
    if (c > CYCLES_MAX) c = CYCLES_MAX;
    signalCycles = (uint8_t)c;
    startPlayback();
  } else {
    signalCycles = CYCLES_DEFAULT;
    if (autoStart || !oledPresent) {
      startPlayback();
    } else {
      // No timing info in metadata — show configuration screen
      appState = STATE_SIG_CONFIG;
      drawSigConfig();
    }
  }
  return true;
}

// ──────────────────────────────────────────────
//  GO BACK (long click)
// ──────────────────────────────────────────────
void goBack() {
  if (appState == STATE_PLAYING) {
    // Stop playback
    stopPlayback();
    // If metadata fully specified timing, go back to browser (config was skipped)
    if (metaCycles > 0 || !oledPresent) {
      appState = STATE_BROWSER;
      resetHeaderScroll(currentPath);
      drawBrowser();
    } else {
      // Go back to signal config
      appState = STATE_SIG_CONFIG;
      drawSigConfig();
    }
    return;
  }

  if (appState == STATE_SIG_CONFIG) {
    // Go back to browser
    appState = STATE_BROWSER;
    resetHeaderScroll(currentPath);
    drawBrowser();
    return;
  }

  if (appState == STATE_VIEW_FILE) {
    // Return to browser from file viewer
    appState = STATE_BROWSER;
    resetHeaderScroll(currentPath);  // restore path scroll state
    drawBrowser();
    return;
  }

  // Navigate up one directory (STATE_BROWSER)
  if (currentPath == "/" || currentPath.isEmpty()) {
    return;   // already at root
  }

  int lastSlash = currentPath.lastIndexOf('/');
  if (lastSlash <= 0) {
    currentPath = "/";
  } else {
    currentPath = currentPath.substring(0, lastSlash);
  }

  loadDirectory(currentPath);
  drawBrowser();
}

// ──────────────────────────────────────────────
//  ENCODER HANDLER
// ──────────────────────────────────────────────
void handleEncoder() {
  // Con attachFullQuad el encoder genera 4 pulsos por cada tick mecánico.
  // Acumulamos la diferencia y sólo actuamos cuando se completa un tick (±4).
  static const int PULSES_PER_TICK = 4;

  int64_t count = encoder.getCount();
  int64_t diff  = count - lastEncoderCount;
  if (diff == 0) return;

  // Calcular cuántos ticks completos hay en la diferencia acumulada
  int ticks = (int)(diff / PULSES_PER_TICK);
  if (ticks == 0) return;   // aún no se completó un tick completo

  // Consumir sólo los pulsos correspondientes a ticks completos
  lastEncoderCount += (int64_t)ticks * PULSES_PER_TICK;

  if (appState == STATE_BROWSER) {
    int prevCursor = cursorIndex;
    // Right = down (+), Left = up (-)
    if (ticks > 0) {
      // Move cursor down
      if (cursorIndex < (int)entries.size() - 1) {
        cursorIndex++;
        // Scroll if cursor goes below visible area
        if (cursorIndex >= scrollOffset + VISIBLE_ITEMS) {
          scrollOffset = cursorIndex - VISIBLE_ITEMS + 1;
        }
      }
    } else {
      // Move cursor up
      if (cursorIndex > 0) {
        cursorIndex--;
        // Scroll if cursor goes above visible area
        if (cursorIndex < scrollOffset) {
          scrollOffset = cursorIndex;
        }
      }
    }
    if (cursorIndex != prevCursor) resetSelectedItemScroll();
    drawBrowser();

  } else if (appState == STATE_VIEW_FILE) {
    int lineH = 9;
    int linesVisible = (64 - HEADER_H) / lineH;
    if (ticks > 0) {
      if (viewScrollY < totalFileLines - linesVisible) {
        viewScrollY++;
      }
    } else {
      if (viewScrollY > 0) {
        viewScrollY--;
      }
    }
    drawFileViewer();

  } else if (appState == STATE_SIG_CONFIG) {
    // Adjust crank revolutions per file (CYCLES_MIN .. CYCLES_MAX)
    int32_t newC = (int32_t)signalCycles + ticks;
    if (newC < CYCLES_MIN) newC = CYCLES_MIN;
    if (newC > CYCLES_MAX) newC = CYCLES_MAX;
    signalCycles = (uint8_t)newC;
    drawSigConfig();

  } else if (appState == STATE_PLAYING) {
    // Adjust RPM mid-generation (100 .. 8000, step 100)
    stepRPM(ticks);
    // Recalculate tickUs — the signal task reads it on-the-fly
    recalcTickFromRPM();
    // Update display to show new RPM
    drawPlaying();
  }
}

// ──────────────────────────────────────────────
//  BUTTON CALLBACKS
// ──────────────────────────────────────────────
void onButtonClick(Button2 &btn) {
  if (appState == STATE_SPLASH) {
    // Skip splash
    appState = STATE_BROWSER;
    drawBrowser();
    return;
  }

  if (appState == STATE_BROWSER) {
    openSelected();
    return;
  }

  if (appState == STATE_SIG_CONFIG) {
    // Click → start playback with current cycles and RPM
    startPlayback();
    return;
  }

  if (appState == STATE_PLAYING) {
    // Short click toggles pause/resume. Long click exits (handled elsewhere).
    isPaused = !isPaused;
    drawPlaying();
    return;
  }
  // In file viewer, click does nothing (long click to go back)
}

void onButtonLongClick(Button2 &btn) {
  if (appState == STATE_BROWSER || appState == STATE_VIEW_FILE ||
      appState == STATE_SIG_CONFIG || appState == STATE_PLAYING) {
    goBack();
  }
}

// ──────────────────────────────────────────────
//  OLED DETECTION
// ──────────────────────────────────────────────
// An SSD1306 answers on 0x3C (most modules) or 0x3D. Nothing answering means
// no screen: the sketch keeps running with the serial console only.
bool detectOled() {
  Wire.begin();
  const uint8_t addrs[] = { 0x3C, 0x3D };
  for (uint8_t a : addrs) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      u8g2.setI2CAddress(a * 2);   // U8g2 takes the 8-bit address
      return true;
    }
  }
  return false;
}

// ──────────────────────────────────────────────
//  SERIAL CONSOLE (115200 baud, one command per line)
// ──────────────────────────────────────────────
// Always on, with or without OLED/encoder. Whatever it changes is also shown
// on the OLED when there is one.

void printHelp() {
  Serial.println(F(""));
  Serial.println(F("──────── Oryx Signal · SD player ────────"));
  Serial.println(F("  ls                 list the current folder"));
  Serial.println(F("  cd <n|name|..|/>   change folder"));
  Serial.println(F("  play <n|name>      open and play a file (digital or analog)"));
  Serial.println(F("  start              play the file waiting on the cycles screen"));
  Serial.println(F("  stop               stop playback"));
  Serial.println(F("  pause | resume     pause / resume playback (also: p)"));
  Serial.println(F("  rpm <100-8000>     set the speed (live while playing)"));
  Serial.println(F("  rpm + | rpm -      speed up / down by 100"));
  Serial.println(F("  cycles <1-8>       set the signal cycles"));
  Serial.println(F("  status             show what is playing"));
  Serial.println(F("  help               this list"));
  Serial.println(F("─────────────────────────────────────────"));
}

void listDirectory() {
  Serial.printf("[DIR] %s\n", currentPath.c_str());
  if (entries.empty()) {
    Serial.println(F("      (empty)"));
    return;
  }
  for (size_t i = 0; i < entries.size(); i++) {
    Serial.printf("  %2u  %s%s\n", (unsigned)(i + 1),
                  entries[i].isDir ? "[DIR] " : "      ", entries[i].name.c_str());
  }
}

// Index into `entries` for "3", "CKP 60-2" or "ckp 60-2.txt"; -1 if none.
int findEntry(const String &arg) {
  if (arg.length() == 0) return -1;
  bool numeric = true;
  for (unsigned i = 0; i < arg.length(); i++) {
    if (!isDigit(arg.charAt(i))) { numeric = false; break; }
  }
  if (numeric) {
    int n = arg.toInt();
    return (n >= 1 && n <= (int)entries.size()) ? n - 1 : -1;
  }
  for (size_t i = 0; i < entries.size(); i++) {
    if (entries[i].name.equalsIgnoreCase(arg) ||
        (!entries[i].isDir && stripExtension(entries[i].name).equalsIgnoreCase(arg))) {
      return (int)i;
    }
  }
  return -1;
}

// Shows the browser again after a folder change, unless a signal is running.
static void showBrowserIfIdle() {
  if (appState == STATE_PLAYING) return;
  appState = STATE_BROWSER;
  resetHeaderScroll(currentPath);
  drawBrowser();
}

void changeDirectory(const String &arg) {
  String target;
  if (arg.length() == 0 || arg == "/") {
    target = "/";
  } else if (arg == "..") {
    int lastSlash = currentPath.lastIndexOf('/');
    target = (lastSlash <= 0) ? "/" : currentPath.substring(0, lastSlash);
  } else if (arg.startsWith("/")) {
    target = arg;
  } else {
    int idx = findEntry(arg);
    if (idx < 0 || !entries[idx].isDir) {
      Serial.printf("[CMD] No such folder: %s\n", arg.c_str());
      return;
    }
    target = currentPath.endsWith("/") ? currentPath + entries[idx].name
                                       : currentPath + "/" + entries[idx].name;
  }

  File dir = SD.open(target);
  bool isDir = dir && dir.isDirectory();
  if (dir) dir.close();
  if (!isDir) {
    Serial.printf("[CMD] No such folder: %s\n", target.c_str());
    return;
  }

  currentPath = target;
  loadDirectory(currentPath);
  showBrowserIfIdle();
  listDirectory();
}

void printStatus() {
  const char *state = "browser";
  if (appState == STATE_PLAYING)         state = isPaused ? "paused" : "playing";
  else if (appState == STATE_SIG_CONFIG) state = "waiting on cycles screen ('start')";
  else if (appState == STATE_VIEW_FILE)  state = "viewing file";

  Serial.println(F("[STATUS] ─────────────────────────"));
  Serial.printf("[STATUS] state        : %s\n", state);
  Serial.printf("[STATUS] folder       : %s\n", currentPath.c_str());
  if (signalFileName.length() > 0) {
    Serial.printf("[STATUS] file         : %s\n", signalFileName.c_str());
    Serial.printf("[STATUS] mode         : %s\n", modeLabel().c_str());
    Serial.printf("[STATUS] steps/cycle  : %.4f\n", signalTotalStepsF);
  }
  Serial.printf("[STATUS] RPM          : %u\n", (unsigned)currentRPM);
  Serial.printf("[STATUS] cycles       : %u\n", (unsigned)signalCycles);
  if (appState == STATE_PLAYING) {
    Serial.printf("[STATUS] tick         : %.4f us/step\n", (float)tickUs);
    Serial.printf("[STATUS] loops        : %u\n", (unsigned)loopCount);
  }
  Serial.printf("[STATUS] OLED         : %s\n", oledPresent ? "yes" : "no");
}

// Applies a new RPM: live while playing, stored for the next start otherwise.
static void setRPM(long rpm) {
  if (rpm < RPM_MIN) rpm = RPM_MIN;
  if (rpm > RPM_MAX) rpm = RPM_MAX;
  currentRPM = (uint32_t)rpm;
  recalcTickFromRPM();   // the signal task reads it on-the-fly
  if (appState == STATE_PLAYING) drawPlaying();
  Serial.printf("[CMD] RPM: %u (tick %.4f us/step)\n", (unsigned)currentRPM, (float)tickUs);
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  int sp = line.indexOf(' ');
  String cmd = sp < 0 ? line : line.substring(0, sp);
  String arg = sp < 0 ? String("") : line.substring(sp + 1);
  cmd.toLowerCase();
  arg.trim();

  if (cmd == "help" || cmd == "?") {
    printHelp();

  } else if (cmd == "ls") {
    listDirectory();

  } else if (cmd == "cd") {
    changeDirectory(arg);

  } else if (cmd == "play") {
    String path, name;
    int idx = findEntry(arg);
    if (idx >= 0) {
      if (entries[idx].isDir) { changeDirectory(arg); return; }
      name = entries[idx].name;
      path = currentPath.endsWith("/") ? currentPath + name : currentPath + "/" + name;
    } else if (arg.startsWith("/")) {
      path = arg;
      name = arg.substring(arg.lastIndexOf('/') + 1);
    } else {
      Serial.printf("[CMD] No such file: %s (try 'ls')\n", arg.c_str());
      return;
    }
    if (appState == STATE_PLAYING) stopPlayback();
    if (!openSignalFile(path, name, true)) showBrowserIfIdle();

  } else if (cmd == "start") {
    if (appState == STATE_SIG_CONFIG) startPlayback();
    else Serial.println(F("[CMD] Nothing waiting to start (use 'play <file>')"));

  } else if (cmd == "stop") {
    if (appState == STATE_PLAYING) stopPlayback();
    else Serial.println(F("[CMD] Nothing playing"));
    showBrowserIfIdle();

  } else if (cmd == "pause" || cmd == "resume" || cmd == "p") {
    if (appState != STATE_PLAYING) { Serial.println(F("[CMD] Nothing playing")); return; }
    if (cmd == "pause")       isPaused = true;
    else if (cmd == "resume") isPaused = false;
    else                      isPaused = !isPaused;
    drawPlaying();
    Serial.println(isPaused ? F("[CMD] Paused") : F("[CMD] Playing"));

  } else if (cmd == "rpm") {
    if (arg == "+")      setRPM((long)currentRPM + RPM_STEP);
    else if (arg == "-") setRPM((long)currentRPM - RPM_STEP);
    else if (arg.length() > 0 && isDigit(arg.charAt(0))) setRPM(arg.toInt());
    else Serial.printf("[CMD] RPM: %u\n", (unsigned)currentRPM);

  } else if (cmd == "cycles") {
    if (arg.length() > 0 && isDigit(arg.charAt(0))) {
      long c = arg.toInt();
      if (c < CYCLES_MIN) c = CYCLES_MIN;
      if (c > CYCLES_MAX) c = CYCLES_MAX;
      signalCycles = (uint8_t)c;
      if (appState == STATE_SIG_CONFIG) drawSigConfig();
      if (appState == STATE_PLAYING)    drawPlaying();
    }
    Serial.printf("[CMD] Cycles: %u\n", (unsigned)signalCycles);

  } else if (cmd == "status") {
    printStatus();

  } else {
    Serial.printf("[CMD] Unknown command: %s (type 'help')\n", cmd.c_str());
  }
}

// Reads whatever arrived on the serial port; runs a command per full line.
void serialPoll() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = buf;
      buf = "";
      handleSerialLine(line);
    } else if (buf.length() < 200) {
      buf += c;
    }
  }
}
