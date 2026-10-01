/**
 * Oryx Signal — standalone ESP32 generator (SD card)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 * Plays the .txt files exported by Oryx Signal Studio straight from an SD
 * card. One sketch for both export modes: the mode is detected from the file.
 *
 *   digital  → one row per state change: step + one 0/1 column per channel.
 *              Up to 16 channels are read. The first 4 go out on GPIOs (CKP,
 *              CMP 1, CMP 2, CMP 3); channels 5 to 16 are kept with the
 *              signal and handed to writeExtraChannels() for your own code.
 *   analog   → one row per chord segment (from, to, channel, levels, handle,
 *              radius, corner radius) plus "RESOLUTION (bits)" metadata. The
 *              curve is rebuilt here with the same geometry as the editor and
 *              sampled into RAM, then played through the two DACs.
 *
 * Big files: a digital file is first read once in 8 KB blocks (metadata,
 * row count, first/last step) with a percentage on screen. If its events fit
 * in RAM it plays from memory, exactly as before. If not (millions of steps),
 * it is STREAMED: a reader task on core 1 keeps a 4096-event ring buffer full
 * from the card while core 0 plays it with the same absolute clock, going
 * back to the start of the file at the end of every pass. If the card cannot
 * keep up with a very fast signal, the output holds and the gap is counted as
 * an "underrun" (see "status").
 *
 * Two ways to drive it — the SD card is always required:
 *   · OLED + encoder: browse, pick the speed unit when a file opens, play,
 *     and edit the speed digit by digit while it plays: turn to move between
 *     the digits, click to edit one (it blinks), turn to change it, click to
 *     finish. A double click switches the unit.
 *   · Serial only (115200 baud): type "help". Works with or without the
 *     OLED and the encoder attached; both can be used at the same time.
 * The OLED is detected at boot (I2C 0x3C / 0x3D). Without it every screen is
 * skipped and the serial console does the whole job.
 *
 * Speed, in the unit you pick (OLED double click, or "unit" on serial):
 *
 *   RPM    100 – 8000        step_time = 60 000 000 / (RPM × total_steps)
 *   us     2.5 – 1 000 000   step_time = the value, µs per step
 *   bps    1 – 400 000       step_time = 1 000 000 / bps (one step = one bit)
 *   Hz     0.5 – 200 000     step_time = 1 000 000 / (2 × Hz): a clock period
 *                            is two steps, one high and one low (I2C, SPI)
 * Timing runs on the CPU cycle counter (240 cycles per µs), so a 2.5 µs step
 * is kept to a fraction of a microsecond. Steps that short only hold from RAM:
 * a streamed file is limited by how fast the card reads (see Big files).
 *
 * File metadata (all optional, at the end of the file):
 *   VOLTAGE: 5            read and kept in metaVoltage, not used: it is there
 *                         for whoever adapts this firmware to their circuit
 *   STEP_TIME (us): 1000  start speed (µs/step) — used first
 *   RPM: 800              start speed (RPM) — used when there is no STEP_TIME
 * The start speed is shown in the unit you pick when the file opens (the
 * last one used is preselected and survives a power cycle).
 * A file opens and plays right away. Older exports may still carry a
 * "CYCLES:" line: it is skipped.
 *
 * Libraries: U8g2, ESP32Encoder, Button2. Boards: classic ESP32 (WROOM /
 * DevKit v1) and ESP32-S3 (DevKitC-1 and compatibles). Also builds as a
 * PlatformIO main.cpp: everything is declared before it is used.
 *
 * ESP32-S3: it has no DAC, so it plays digital files only (analog ones are
 * refused with a message) and its pins differ — see PIN CONFIGURATION. Build
 * it with "USB CDC On Boot" enabled: the serial console then answers on both
 * connectors of the board, the native USB one and the UART bridge.
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <ESP32Encoder.h>
#include <Button2.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include <soc/gpio_struct.h>
#include <esp_timer.h>
#include <esp_cpu.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <soc/soc_caps.h>

// ──────────────────────────────────────────────
//  ESP32-S3 SHIMS
// ──────────────────────────────────────────────
#if !SOC_DAC_SUPPORTED
// No DAC on this chip: analog files are refused before they load (see
// openSignalFile), so the analog code below never runs. These keep it
// compiling unchanged. Macros and not functions: the IDE writes its function
// prototypes above the first function it finds, which here would land before
// the types they use.
#define dacWrite(pin, value) ((void)(pin), (void)(value))
#define dacDisable(pin)      ((void)(pin))
#endif

#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
// With "USB CDC On Boot" `Serial` is the native USB port only. This console
// writes to both connectors (native USB and the UART bridge) and reads from
// whichever has data, so the board answers wherever it is plugged in. Every
// `Serial.` below goes through it.
class DualConsole : public Stream {
public:
  void begin(unsigned long baud) { HWCDCSerial.begin(baud); Serial0.begin(baud); }
  int available() override { return HWCDCSerial.available() + Serial0.available(); }
  int read() override { return HWCDCSerial.available() ? HWCDCSerial.read() : Serial0.read(); }
  int peek() override { return HWCDCSerial.available() ? HWCDCSerial.peek() : Serial0.peek(); }
  void flush() override { HWCDCSerial.flush(); Serial0.flush(); }
  size_t write(uint8_t b) override { HWCDCSerial.write(b); return Serial0.write(b); }
  size_t write(const uint8_t *buf, size_t n) override { HWCDCSerial.write(buf, n); return Serial0.write(buf, n); }
};
static DualConsole dualConsole;
#undef Serial
#define Serial dualConsole
#endif

// ──────────────────────────────────────────────
//  PIN CONFIGURATION
// ──────────────────────────────────────────────
#if defined(CONFIG_IDF_TARGET_ESP32S3)
// ESP32-S3: GPIO 22-25 and 32-37 are not usable, so everything moves. Avoided
// on purpose: strapping pins (0, 3, 45, 46), native USB (19, 20), UART (43,
// 44) and the RGB LED of the dev boards (38, 48). The microSD uses the default
// SPI pins (SCK 12, MOSI 11, MISO 13) and the OLED the default I2C pins
// (SDA 8, SCL 9).
#define ENCODER_PIN_A    4
#define ENCODER_PIN_B    5
#define ENCODER_BTN_PIN  6

#define SD_CS_PIN       10   // SD card CS pin

// Signal output pins (one per channel: CKP, CMP, CMP2, CMP3)
#define NUM_CHANNELS     4
#define SIG_PIN_CH0     15   // CKP
#define SIG_PIN_CH1     16   // CMP
#define SIG_PIN_CH2     17   // CMP 2
#define SIG_PIN_CH3     18   // CMP 3
#else
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
#endif

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

// I2C OLED (SSD1306 128x64) – default SDA=21, SCL=22
// If your board uses different pins, change the constructor below.

// ──────────────────────────────────────────────
//  OBJECTS
// ──────────────────────────────────────────────
// SSD1306 128x64 I2C – full buffer for smooth drawing
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

ESP32Encoder encoder;
Button2 button;

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
  STATE_UNIT_SELECT,   // file loaded: pick the speed unit, then play
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
// Channels read from a digital file: the 16 that Oryx can design. Only the
// first NUM_CHANNELS have a pin; the rest travel in `extra`.
#define MAX_FILE_CHANNELS 16

struct SignalEvent {
  float    stepF;                 // absolute step position from file (fractional)
  float    durationF;             // duration in steps (next_step - this_step), fractional
  uint8_t  val[NUM_CHANNELS];     // channel values (0|1)
  uint16_t extra;                 // channels 5..16, bit 0 = channel 5 (no pin)
};

// Reads a file in big blocks and hands out one line at a time, without a
// String per line: what makes a file with millions of rows readable in
// reasonable time. Only member functions, so the IDE's prototypes are fine.
class BlockReader {
public:
  File     f;
  uint8_t *buf = nullptr;
  size_t   cap = 0;
  size_t   len = 0;
  size_t   pos = 0;
  uint32_t bufStart = 0;   // file offset of buf[0]

  bool open(const String &path, size_t size) {
    f = SD.open(path, FILE_READ);
    if (!f) return false;
    buf = (uint8_t *)malloc(size);
    if (!buf) { f.close(); return false; }
    cap = size; len = pos = 0; bufStart = 0;
    return true;
  }
  void close() {
    if (f) f.close();
    if (buf) { free(buf); buf = nullptr; }
  }
  uint32_t size() { return f ? (uint32_t)f.size() : 0; }
  // Offset of the next unread byte
  uint32_t tell() { return bufStart + pos; }
  void seek(uint32_t off) {
    f.seek(off);
    bufStart = off; len = pos = 0;
  }
  // Copies the next line into `out` (no \r, no \n). Returns its length,
  // or -1 at the end of the file. Overlong lines are cut, never overflow.
  int readLine(char *out, int maxLen) {
    int n = 0;
    bool any = false;
    while (true) {
      if (pos >= len) {
        bufStart += len;
        len = f.read(buf, cap);
        pos = 0;
        if (len == 0) break;
      }
      char c = (char)buf[pos++];
      any = true;
      if (c == '\n') break;
      if (c == '\r') continue;
      if (n < maxLen - 1) out[n++] = c;
    }
    out[n] = 0;
    return any ? n : -1;
  }
};

// What one read of a digital file tells (see scanSignalFile).
// Steps are doubles: past 4 million a float cannot tell fractions of a step
// apart (it resolves half a step), and files that long are what streaming is
// for. Durations are taken as double differences and only then made float.
struct ScanInfo {
  uint32_t rows;        // data rows, end marker included
  double   firstStep;
  double   lastStep;
  uint16_t firstBits;   // bit c = channel c (all 16)
  uint16_t lastBits;
  double   gridStep;    // smallest gap between rows
  int      channels;
  uint32_t dataStart;   // file offset right after the header line
  uint32_t size;
};

// One event of the streaming ring buffer: 8 bytes.
struct StreamEvt {
  float    dur;         // steps it holds
  uint16_t bits;        // bit c = channel c
  uint16_t flags;       // bit 0: first event of a pass of the signal
};

std::vector<SignalEvent> signalEvents;   // parsed event list
float  signalTotalStepsF = 0;            // total step-duration of one cycle (fractional)
int    signalTotalEvents = 0;            // number of events (transitions)
int    fileChannels      = 0;            // channel columns in the loaded digital file
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
float   metaVoltage    = 0.0f;   // VOLTAGE: read only, nothing uses it (yours to use)
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

// Speed unit. RPM keeps the verified formula above; the other three give the
// step time straight away, without needing the file's step count.
#define UNIT_US   0
#define UNIT_RPM  1
#define UNIT_BPS  2
#define UNIT_HZ   3
#define UNIT_COUNT 4
static const char *UNIT_NAMES[UNIT_COUNT] = { "us", "rpm", "bps", "hz" };
static const char *UNIT_LABELS[UNIT_COUNT] = { "us/step", "RPM", "bps", "Hz" };
static const float UNIT_MIN[UNIT_COUNT] = { 2.5f,       RPM_MIN, 1.0f,      0.5f };
static const float UNIT_MAX[UNIT_COUNT] = { 1000000.0f, RPM_MAX, 400000.0f, 200000.0f };
uint8_t speedUnit  = UNIT_RPM;   // the last one used, restored from NVS at boot
// Unit highlighted on the unit selection screen
uint8_t unitCursor = UNIT_RPM;
// Start speed the open file asks for, in µs/step (0 = the file has none)
float   fileStartUs = 0.0f;
// The last unit is kept in NVS so it is preselected after a power cycle
Preferences prefs;
// Value in the current unit when it is not RPM (RPM lives in currentRPM).
float   speedValue = 1000.0f;

// The speed is shown with a fixed number of digits per unit, so every digit
// keeps its place on screen while it is edited ("0001000.00 us/step").
static const uint8_t UNIT_INT_DIGITS[UNIT_COUNT] = { 7, 4, 6, 6 };   // us, rpm, bps, hz
static const uint8_t UNIT_DECIMALS[UNIT_COUNT]   = { 2, 0, 0, 2 };

// Playing screen cursor: 0..digits-1 are the digits (left to right), then
// the unit label, then the Pause/Resume button.
uint8_t  playCursor   = 0;
bool     editingDigit = false;   // the digit under the cursor blinks and the encoder changes it
bool     blinkOn      = true;
uint32_t blinkAt      = 0;
#define BLINK_MS 350

// Per-step duration in microseconds (computed from the speed, not user-set directly).
// Kept as float so the sub-microsecond fraction is preserved — truncating it to
// an integer biased every step short, making playback a few µs/pulse too fast.
volatile float tickUs = 1000.0f;

// Playback state
volatile bool     isPlaying        = false;
volatile bool     isPaused         = false;   // short-click toggles this during playback
volatile uint32_t playbackStep     = 0;
volatile uint32_t loopCount        = 0;     // completed signal cycles
bool              streamMode       = false; // the loaded file plays from the card (too big for RAM)
volatile uint32_t underruns        = 0;     // times the card fell behind while streaming

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
void onButtonDoubleClick(Button2 &btn);
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
void drawPlaying();
void startPlayback();
void stopPlayback();
void freeSigBuffers();
void freeAnaBuffers();
void recalcTickFromRPM();
void stepRPM(int ticks);
void recalcTick();
void stepSpeed(int ticks);
void setSpeedUnit(uint8_t unit);
uint8_t speedDigitCount();
String speedDigits();
void editSpeedDigit(int dir);
void resetPlayCursor();
void setSpeedFromUs(uint8_t unit, float us);
void saveSpeedUnit();
void drawUnitSelect();
void playWithUnit(uint8_t unit);
String speedLabel();
String modeLabel();
String stripExtension(const String &name);
bool openSignalFile(const String &path, const String &name, bool askUnit);
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

  // ── Last speed unit ──
  prefs.begin("oryx-sd", false);
  speedUnit = prefs.getUChar("unit", UNIT_RPM);
  if (speedUnit >= UNIT_COUNT) speedUnit = UNIT_RPM;
  unitCursor = speedUnit;

  // ── Button ──
  button.begin(ENCODER_BTN_PIN, INPUT_PULLUP, true);   // active LOW
  button.setClickHandler(onButtonClick);
  button.setLongClickDetectedHandler(onButtonLongClick);
  button.setDoubleClickHandler(onButtonDoubleClick);
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
  // While a big file streams, give the card reader core 1 between passes
  // (it runs at this same priority; see startStreaming)
  if (streamMode && isPlaying) vTaskDelay(1);

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

    case STATE_UNIT_SELECT:
      handleEncoder();
      break;

    case STATE_PLAYING: {
      // Speed changes mid-generation via encoder, digit by digit
      handleEncoder();
      // The "SD too slow" title shows up once, when the first gap happens
      static uint32_t shownUnderruns = 0;
      if (streamMode && (underruns > 0) != (shownUnderruns > 0)) {
        shownUnderruns = underruns;
        drawPlaying();
      }
      // The digit being edited blinks (only then is the screen redrawn on its own)
      if (editingDigit && millis() - blinkAt >= BLINK_MS) {
        blinkOn = !blinkOn;
        blinkAt = millis();
        drawPlaying();
      }
      break;
    }
  }
}

// ──────────────────────────────────────────────
//  SPLASH SCREEN
// ──────────────────────────────────────────────
// Two centered lines: the name and, a bit smaller, what this sketch is.
void drawSplash() {
  if (!oledPresent) return;
  u8g2.clearBuffer();
  u8g2.setDrawColor(1);
  const char *title = "Oryx Signal Studio";
  u8g2.setFont(u8g2_font_7x13B_tr);           // 7 px per letter: fits the 128 px width
  u8g2.drawStr(64 - u8g2.getStrWidth(title) / 2, 30, title);
  const char *subtitle = "TEST CODE";
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(64 - u8g2.getStrWidth(subtitle) / 2, 46, subtitle);
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

// Step time from the current speed. RPM goes through the verified formula
// untouched; the other units do not depend on the file's step count.
void recalcTick() {
  if (speedUnit == UNIT_RPM) { recalcTickFromRPM(); return; }
  float t;
  if (speedUnit == UNIT_US)       t = speedValue;
  else if (speedUnit == UNIT_BPS) t = 1000000.0f / speedValue;
  else                            t = 1000000.0f / (2.0f * speedValue);
  if (t < 0.001f) t = 0.001f;
  tickUs = t;
}

// About 10 % of the value per detent, so the knob is useful from 5 µs to
// a million. RPM keeps its fixed RPM_STEP.
static float smartStep(float v, float minStep) {
  float mag = floorf(log10f(v > 1.0f ? v : 1.0f));
  float st = powf(10.0f, mag - 1.0f);
  return st < minStep ? minStep : st;
}

void stepSpeed(int ticks) {
  if (speedUnit == UNIT_RPM) { stepRPM(ticks); recalcTick(); return; }
  float minStep = speedUnit == UNIT_HZ ? 0.5f : 1.0f;
  float v = speedValue + (ticks > 0 ? 1 : -1) * smartStep(speedValue, minStep);
  if (v < UNIT_MIN[speedUnit]) v = UNIT_MIN[speedUnit];
  if (v > UNIT_MAX[speedUnit]) v = UNIT_MAX[speedUnit];
  speedValue = v;
  recalcTick();
}

// Another unit at the same speed: the number changes, the signal does not.
// Going to RPM needs the file's step count; without a file the RPM is kept.
void setSpeedUnit(uint8_t unit) {
  if (unit >= UNIT_COUNT || unit == speedUnit) return;
  setSpeedFromUs(unit, tickUs);
}

// Sets a speed given in µs/step, expressed in `unit`. Going to RPM needs the
// file's step count; without a file the RPM is kept.
void setSpeedFromUs(uint8_t unit, float us) {
  if (unit >= UNIT_COUNT) return;
  if (us > 0.0f) {
    if (unit == UNIT_RPM) {
      if (signalTotalStepsF > 1e-3f) {
        long rpm = lroundf(60000000.0f / (us * signalTotalStepsF));
        if (rpm < RPM_MIN) rpm = RPM_MIN;
        if (rpm > RPM_MAX) rpm = RPM_MAX;
        currentRPM = (uint32_t)rpm;
      }
    } else {
      float v = unit == UNIT_US  ? us
              : unit == UNIT_BPS ? 1000000.0f / us
              :                    1000000.0f / (2.0f * us);
      if (v < UNIT_MIN[unit]) v = UNIT_MIN[unit];
      if (v > UNIT_MAX[unit]) v = UNIT_MAX[unit];
      speedValue = v;
    }
  }
  speedUnit = unit;
  recalcTick();
}

// Remembers the unit for the next file and the next power-up.
void saveSpeedUnit() {
  prefs.putUChar("unit", speedUnit);
}

// How many editable digits the current unit has.
uint8_t speedDigitCount() {
  return UNIT_INT_DIGITS[speedUnit] + UNIT_DECIMALS[speedUnit];
}

// The current speed with its fixed width: "0001000.00", "0800", "009600".
String speedDigits() {
  uint8_t dec = UNIT_DECIMALS[speedUnit];
  int width = UNIT_INT_DIGITS[speedUnit] + (dec ? dec + 1 : 0);
  float v = speedUnit == UNIT_RPM ? (float)currentRPM : speedValue;
  char buf[16];
  snprintf(buf, sizeof(buf), "%0*.*f", width, dec, v);
  return String(buf);
}

// Adds or takes one from the digit under the cursor (with carry), within the
// unit's limits. Applied live: the signal task reads tickUs on the fly.
void editSpeedDigit(int dir) {
  uint8_t ints = UNIT_INT_DIGITS[speedUnit];
  float place = playCursor < ints ? powf(10.0f, (float)(ints - 1 - playCursor))
                                  : powf(10.0f, -(float)(playCursor - ints + 1));
  if (speedUnit == UNIT_RPM) {
    long rpm = (long)currentRPM + dir * (long)place;
    if (rpm < RPM_MIN) rpm = RPM_MIN;
    if (rpm > RPM_MAX) rpm = RPM_MAX;
    currentRPM = (uint32_t)rpm;
  } else {
    float v = speedValue + dir * place;
    if (v < UNIT_MIN[speedUnit]) v = UNIT_MIN[speedUnit];
    if (v > UNIT_MAX[speedUnit]) v = UNIT_MAX[speedUnit];
    float scale = powf(10.0f, (float)UNIT_DECIMALS[speedUnit]);
    speedValue = roundf(v * scale) / scale;   // no float crumbs from adding 0.01
  }
  recalcTick();
}

// Cursor on the first significant digit, nothing being edited.
void resetPlayCursor() {
  String d = speedDigits();
  uint8_t ints = UNIT_INT_DIGITS[speedUnit];
  playCursor = ints - 1;   // the units digit, if every other one is zero
  for (uint8_t i = 0; i < ints; i++) {
    if (d.charAt(i) != '0') { playCursor = i; break; }
  }
  editingDigit = false;
  blinkOn = true;
}

// "800 RPM", "1000.00 us/step", "9600 bps", "100000.00 Hz"
String speedLabel() {
  if (speedUnit == UNIT_RPM) return String(currentRPM) + " RPM";
  int decimals = speedUnit == UNIT_BPS ? 0 : 2;
  return String(speedValue, decimals) + " " + UNIT_LABELS[speedUnit];
}

// "Digital 4ch" / "Analog 1ch" — shown under the RPM.
String modeLabel() {
  if (signalMode == MODE_ANALOG) return "Analog " + String(anaChannels) + "ch";
  if (fileChannels > NUM_CHANNELS) {
    return "Digital " + String(NUM_CHANNELS) + "ch +" + String(fileChannels - NUM_CHANNELS) + " data";
  }
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

  // Only the first lines: a big file would not fit in RAM as Strings
  const int VIEW_MAX_LINES = 300;
  while (f.available() && (int)fileLines.size() < VIEW_MAX_LINES) {
    String line = f.readStringUntil('\n');
    line.trim();
    fileLines.push_back(line);
  }
  if (f.available()) fileLines.push_back("... (first " + String(VIEW_MAX_LINES) + " lines)");
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
    return true;   // older exports: no longer used
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
// Only analog files carry "RESOLUTION (bits)", and it is in the metadata at
// the end: reading the last 4 KB is enough, instead of the whole file (which
// took minutes on a file with millions of rows).
bool isAnalogFile(const String &path) {
  File f = SD.open(path, FILE_READ);
  if (!f) return false;
  uint32_t size = f.size();
  uint32_t from = size > 4096 ? size - 4096 : 0;
  f.seek(from);
  bool analog = false;
  if (from > 0) f.readStringUntil('\n');   // first line may be cut in half
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.startsWith("RESOLUTION")) { analog = true; break; }
  }
  f.close();
  return analog;
}

// One data row, without Strings: step, then up to 16 channel columns (any
// value other than 0 is high, same as the in-memory parser). False if there
// is no channel column.
bool parseRowFast(const char *line, double *step, uint16_t *bits, int *cols) {
  char *end;
  *step = strtod(line, &end);
  if (end == line) return false;
  const char *p = end;
  uint16_t b = 0;
  int c = 0;
  while (*p && c < MAX_FILE_CHANNELS) {
    while (*p == ' ' || *p == '\t') p++;
    if (!*p) break;
    long v = strtol(p, &end, 10);
    if (end == p) {                 // not a number ('-'): low, skip the token
      while (*p && *p != ' ' && *p != '\t') p++;
    } else {
      p = end;
      while (*p && *p != ' ' && *p != '\t') p++;
    }
    if (v != 0) b |= (uint16_t)(1u << c);
    c++;
  }
  *bits = b;
  *cols = c;
  return c > 0;
}

// Reads a digital file once, in blocks: metadata, how many rows, first and
// last step, and whether the last row is an end-of-cycle marker. Shows the
// progress, since a big file takes a while.
bool scanSignalFile(const String &path, ScanInfo *info) {
  BlockReader r;
  if (!r.open(path, 8192)) {
    Serial.println(F("[SIG] Cannot open file"));
    return false;
  }
  metaVoltage = 0.0f; metaRPM = 0; metaStepTime = 0.0f; metaResolution = 0;
  memset(info, 0, sizeof(*info));
  info->size = r.size();
  info->gridStep = 1.0f;

  static char line[256];
  r.readLine(line, sizeof(line));          // header (column names)
  info->dataStart = r.tell();

  double minDiff = 1e30;
  double prevStep = 0.0;
  int    lastPct = -1;
  while (true) {
    int n = r.readLine(line, sizeof(line));
    if (n < 0) break;
    if (n == 0) continue;
    if (!isDigit(line[0])) {
      // Metadata (and the credit line, which has no ':')
      String m(line);
      m.trim();
      parseMetaLine(m);
      continue;
    }
    double st; uint16_t bits; int cols;
    if (!parseRowFast(line, &st, &bits, &cols)) continue;
    if (info->rows == 0) { info->firstStep = st; info->firstBits = bits; }
    else {
      double d = st - prevStep;
      if (d > 1e-6 && d < minDiff) minDiff = d;
    }
    prevStep = st;
    info->lastStep = st;
    info->lastBits = bits;
    if (cols > info->channels) info->channels = cols;
    info->rows++;

    if ((info->rows & 0x3FFF) == 0) {
      int pct = info->size ? (int)((uint64_t)r.tell() * 100 / info->size) : 0;
      if (pct != lastPct) {
        lastPct = pct;
        Serial.printf("[SIG] Reading %d%% (%u rows)\n", pct, (unsigned)info->rows);
        if (oledPresent) {
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_6x10_tr);
          String a = "Reading " + String(pct) + "%";
          String b = String(info->rows) + " rows";
          u8g2.drawStr(64 - u8g2.getStrWidth(a.c_str()) / 2, 28, a.c_str());
          u8g2.drawStr(64 - u8g2.getStrWidth(b.c_str()) / 2, 42, b.c_str());
          u8g2.sendBuffer();
        }
      }
    }
  }
  r.close();
  if (minDiff < 1e29) info->gridStep = minDiff;
  if (info->channels > MAX_FILE_CHANNELS) info->channels = MAX_FILE_CHANNELS;
  Serial.printf("[SIG] %u rows, %d channels, steps %.4f..%.4f\n",
                (unsigned)info->rows, info->channels, info->firstStep, info->lastStep);
  return info->rows > 0;
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
    double   stepF;              // original fractional step from file (double: see ScanInfo)
    uint8_t  val[NUM_CHANNELS];
    uint16_t extra;              // channels 5..16
  };
  std::vector<RawEvent> rawEvents;

  // Reset metadata to defaults
  metaVoltage    = 0.0f;
  metaRPM        = 0;
  metaStepTime   = 0.0f;
  metaResolution = 0;

  int skippedLines = 0;
  fileChannels = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;

    // ── Check for metadata lines ──
    if (parseMetaLine(line)) continue;

    // Data rows start with their step number. Anything else — such as the
    // "Generated with Oryx" credit line at the end of the file — is not a row.
    if (!isDigit(line.charAt(0))) continue;

    // ── Tokenize by TAB or consecutive spaces ──
    // Step + up to MAX_FILE_CHANNELS channel columns
    String fields[MAX_FILE_CHANNELS + 1];
    int fieldCount = 0;
    int len = line.length();
    int i = 0;
    while (i < len && fieldCount < MAX_FILE_CHANNELS + 1) {
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
    ev.stepF = strtod(fields[0].c_str(), nullptr);   // double, to keep fractions on long signals
    for (int c = 0; c < NUM_CHANNELS; c++) {
      ev.val[c] = ((c + 1) < fieldCount && fields[c + 1].toInt() != 0) ? 1 : 0;
    }
    // Channels past the pins: only kept, see writeExtraChannels()
    ev.extra = 0;
    for (int c = NUM_CHANNELS; c < fieldCount - 1; c++) {
      if (fields[c + 1].toInt() != 0) ev.extra |= (uint16_t)(1u << (c - NUM_CHANNELS));
    }
    if (fieldCount - 1 > fileChannels) fileChannels = fieldCount - 1;
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
  // With more than 4 channels the marker has to match on those too (on a
  // 4-channel file `extra` is always 0, so nothing changes there).
  if (hasEndMarker && rawEvents[numRaw - 1].extra != rawEvents[0].extra) hasEndMarker = false;

  // Finest grid step — used only as the last-event duration when no wrap marker
  // is present (the period is then unknown and this is the best estimate).
  float gridStep = 1.0f;
  {
    double minDiff = 1e30;
    for (int i = 1; i < numRaw; i++) {
      double d = rawEvents[i].stepF - rawEvents[i - 1].stepF;
      if (d > 1e-6 && d < minDiff) minDiff = d;
    }
    if (minDiff < 1e29) gridStep = (float)minDiff;
  }

  // Number of real (playable) events: drop the trailing marker row if present.
  int numEvents = hasEndMarker ? (numRaw - 1) : numRaw;

  for (int i = 0; i < numEvents; i++) {
    SignalEvent se;
    se.stepF = (float)rawEvents[i].stepF;
    for (int c = 0; c < NUM_CHANNELS; c++) {
      se.val[c] = rawEvents[i].val[c];
    }
    se.extra = rawEvents[i].extra;

    if (i < numRaw - 1) {
      // Duration in steps (fractional) = next step position - this step position.
      // For the last real event with a marker, rawEvents[i + 1] IS the marker,
      // so this yields the correct wrap duration.
      se.durationF = (float)(rawEvents[i + 1].stepF - rawEvents[i].stepF);
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
  signalTotalStepsF = (float)(rawEvents.back().stepF - rawEvents.front().stepF);
  if (signalTotalStepsF < 1e-3f) {
    // Degenerate (single row or all-equal steps): fall back to summed durations.
    signalTotalStepsF = 0.0f;
    for (int i = 0; i < signalTotalEvents; i++) signalTotalStepsF += signalEvents[i].durationF;
  }

  // Debug output
  Serial.printf("[SIG] Parsed %d events, %.2f total steps per cycle\n",
                signalTotalEvents, signalTotalStepsF);
  if (fileChannels > NUM_CHANNELS) {
    Serial.printf("[SIG] %d channels in the file: 1-%d go out on pins, %d-%d are kept for writeExtraChannels()\n",
                  fileChannels, NUM_CHANNELS, NUM_CHANNELS + 1, fileChannels);
  }
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
  Serial.printf("[SIG] Metadata → VOLTAGE: %.2f, RPM: %d\n",
                metaVoltage, metaRPM);

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
      double dac = clamp01d(y) * 255.0;
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
  Serial.printf("[ANA] Metadata → VOLTAGE: %.2f, RPM: %d\n",
                metaVoltage, metaRPM);
  return true;
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
  // A streamed file the card cannot keep up with says so in the title
  const char *title = isPaused ? "Paused"
                    : (streamMode && underruns > 0) ? "SD too slow: lower speed"
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

  // Speed, digit by digit (the font is 6 px wide for every character). The
  // digit under the cursor is underlined; while it is being edited it also
  // blinks. Then the unit label, which is a cursor stop of its own.
  String digits = speedDigits();
  const char *unitTxt = UNIT_LABELS[speedUnit];
  uint8_t ndig = speedDigitCount();
  uint8_t ints = UNIT_INT_DIGITS[speedUnit];
  if (playCursor > ndig + 1) playCursor = ndig + 1;   // the unit changed from serial
  int totalW = (digits.length() + 1 + strlen(unitTxt)) * 6;
  int x0 = 64 - totalW / 2;
  const int yv = 38;
  for (uint8_t i = 0; i < digits.length(); i++) {
    int x = x0 + i * 6;
    char ch = digits.charAt(i);
    int d = ch == '.' ? -1 : (i < ints ? i : i - 1);   // digit index of this char
    bool here = d >= 0 && d == playCursor;
    if (!(here && editingDigit && !blinkOn)) {
      char one[2] = { ch, 0 };
      u8g2.drawStr(x, yv, one);
    }
    if (here) u8g2.drawHLine(x, yv + 2, 5);
  }
  int xu = x0 + (digits.length() + 1) * 6;
  u8g2.drawStr(xu, yv, unitTxt);
  if (playCursor == ndig) u8g2.drawRFrame(xu - 2, yv - 10, strlen(unitTxt) * 6 + 3, 13, 2);

  // Pause / Resume, the last cursor stop
  const char *pz = isPaused ? "Resume" : "Pause";
  int pzW = u8g2.getStrWidth(pz);
  if (playCursor == ndig + 1) {
    u8g2.drawRBox(64 - pzW / 2 - 4, 43, pzW + 8, 12, 2);
    u8g2.setDrawColor(0);
  }
  u8g2.drawStr(64 - pzW / 2, 53, pz);
  u8g2.setDrawColor(1);

  // Hint: what a click does where the cursor is
  u8g2.setFont(u8g2_font_5x7_tr);
  const char *hint = editingDigit         ? "Turn: change  Click: done"
                   : playCursor < ndig    ? "Click: edit  Hold: exit"
                   : playCursor == ndig   ? "Click: unit  Hold: exit"
                   : isPaused             ? "Click: resume  Hold: exit"
                   :                        "Click: pause  Hold: exit";
  int hintW = u8g2.getStrWidth(hint);
  u8g2.drawStr(64 - hintW / 2, 63, hint);

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
uint16_t *evtExtra      = NULL;  // channels 5..16 per event; NULL when the file has <= 4
volatile int evtCount    = 0;    // number of events

// ──────────────────────────────────────────────
//  CHANNELS WITHOUT A PIN (5 to 16) — YOUR CODE GOES HERE
// ──────────────────────────────────────────────
// Current value of the channels past the 4 pins: bit 0 = channel 5, bit 11 =
// channel 16. Updated on every event while playing, back to 0 when stopped
// or paused.
volatile uint16_t extraChannels = 0;

// Called on every event with channels 5..16, only for files that have them.
// Empty on purpose: this is where to send them out (an I2C expander, a
// 74HC595, another board). It runs inside the playback loop on core 0, so
// keep it short or the signal loses timing accuracy.
static inline void writeExtraChannels(uint16_t bits) {
  (void)bits;
}

TaskHandle_t sigTaskHandle = NULL;

// 64-bit CPU cycle clock for the playback tasks (they always run on core 0,
// and each core has its own counter). The 32-bit counter wraps every ~18 s
// at 240 MHz; it is read far more often than that while playing, and after a
// pause the tasks realign their clock anyway.
static uint32_t cycLast = 0;
static uint64_t cycHigh = 0;
static inline uint64_t cyc64() {
  uint32_t c = (uint32_t)esp_cpu_get_cycle_count();
  if (c < cycLast) cycHigh += (1ULL << 32);
  cycLast = c;
  return cycHigh | c;
}

// The signal generator task — runs on Core 0
// Reads volatile tickUs each event so RPM changes take effect immediately.
void sigGenTask(void *param) {
  disableCore0WDT();

  // idealQ is the high-precision running target time, in CPU cycles × 65536
  // (16 fractional bits). We add the exact fractional hold of each event to it
  // and round only when deriving the deadline. This keeps fractions from being
  // thrown away every step (which made playback drift fast) and prevents
  // accumulation. Counting cycles instead of whole microseconds keeps steps of
  // a few µs (200 kHz clocks) accurate to a fraction of a microsecond, and
  // integer math keeps the loop short: the ESP32 has no FPU for doubles.
  const float cyclesPerUs = (float)getCpuFrequencyMhz();
  uint64_t idealQ = cyc64() << 16;                    // absolute time reference
  uint64_t target = idealQ >> 16;
  float    lastTick = -1.0f, tickQ = 0.0f;            // tickUs in cycles × 65536

  while (isPlaying) {
    // Pause handling: drive all outputs LOW and idle until resume or stop.
    // On resume, realign the target clock so the next event starts immediately.
    if (isPaused) {
      GPIO.out_w1tc = ALL_CH_MASK;
      if (evtExtra) { extraChannels = 0; writeExtraChannels(0); }
      while (isPaused && isPlaying) {
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      idealQ = cyc64() << 16;
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
    if (evtExtra) {
      extraChannels = evtExtra[evtIdx];
      writeExtraChannels(extraChannels);
    }

    // Advance the precise target by this event's exact hold (read volatile tickUs
    // each event so RPM changes take effect immediately), then round the absolute
    // deadline to the nearest microsecond. Rounding the cumulative target — not
    // each increment — means per-event rounding never accumulates into drift.
    float t = tickUs;                                  // read once: it changes live
    if (t != lastTick) { lastTick = t; tickQ = t * cyclesPerUs * 65536.0f; }
    idealQ += (uint64_t)(evtDurF[evtIdx] * tickQ + 0.5f);
    target = (idealQ + 32768) >> 16;

    playbackStep = evtIdx + 1;

    // Busy-wait until the ABSOLUTE target time
    while ((int64_t)(cyc64() - target) < 0) {
      // tight spin — no yield, no sleep
    }
  }

  // Done — set all outputs LOW
  GPIO.out_w1tc = ALL_CH_MASK;
  if (evtExtra) { extraChannels = 0; writeExtraChannels(0); }

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

// ──────────────────────────────────────────────
//  STREAMING (files too big for RAM)
// ──────────────────────────────────────────────
#define STREAM_RING 4096                 // events in the ring (power of two)
ScanInfo   streamInfo;
StreamEvt *ring         = NULL;
volatile uint32_t ringHead = 0;          // written by the reader (core 1)
volatile uint32_t ringTail = 0;          // read by the player (core 0)
volatile bool readerRunning = false;
TaskHandle_t readerTaskHandle = NULL;
uint32_t streamSetLut[16], streamClrLut[16];   // 4 pin bits -> GPIO masks

static inline void ringPush(float dur, uint16_t bits, uint16_t flags) {
  StreamEvt &e = ring[ringHead & (STREAM_RING - 1)];
  e.dur = dur < 1e-6f ? 1e-6f : dur;
  e.bits = bits;
  e.flags = flags;
  __sync_synchronize();                  // the event is written before it is published
  ringHead = ringHead + 1;
}

// Core 1: reads rows in blocks and keeps the ring full. Each event holds its
// values until the next row (same as the in-memory parser); at the end of
// the file the pass wraps: an end marker is dropped (its step already closed
// the last event), otherwise the last row holds one grid step.
void sdReaderTask(void *param) {
  BlockReader r;
  if (!r.open(signalFilePath, 8192)) {
    Serial.println(F("[STREAM] Cannot open the file"));
    readerRunning = false;
    readerTaskHandle = NULL;
    vTaskDelete(NULL);
    return;
  }
  bool endMarker = streamInfo.rows >= 2 && streamInfo.lastBits == streamInfo.firstBits;
  r.seek(streamInfo.dataStart);
  static char line[256];
  bool     havePrev = false, prevFirst = false, passStart = true;
  double   prevStep = 0.0;
  uint16_t prevBits = 0;
  uint32_t sinceYield = 0;

  while (readerRunning) {
    if (ringHead - ringTail >= STREAM_RING) { vTaskDelay(1); continue; }
    int n = r.readLine(line, sizeof(line));
    if (n < 0) {                         // end of the pass
      if (havePrev && !endMarker) ringPush((float)streamInfo.gridStep, prevBits, prevFirst ? 1 : 0);
      havePrev = false;
      passStart = true;
      r.seek(streamInfo.dataStart);
      continue;
    }
    if (n == 0 || !isDigit(line[0])) continue;
    double st; uint16_t bits; int cols;
    if (!parseRowFast(line, &st, &bits, &cols)) continue;
    if (havePrev) ringPush((float)(st - prevStep), prevBits, prevFirst ? 1 : 0);
    prevStep = st; prevBits = bits; prevFirst = passStart; passStart = false;
    havePrev = true;
    // With the ring well fed, let the loop (screen, encoder) run now and then
    if (++sinceYield >= 512) {
      sinceYield = 0;
      if (ringHead - ringTail > STREAM_RING / 2) vTaskDelay(1);
    }
  }
  r.close();
  readerTaskHandle = NULL;
  vTaskDelete(NULL);
}

// Core 0: plays the ring with the same absolute clock as sigGenTask.
void streamGenTask(void *param) {
  disableCore0WDT();
  const float cyclesPerUs = (float)getCpuFrequencyMhz();
  uint64_t idealQ = cyc64() << 16;                    // cycles × 65536, as in sigGenTask
  uint64_t target = idealQ >> 16;
  float    lastTick = -1.0f, tickQ = 0.0f;
  bool    first    = true;
  bool    extras   = fileChannels > NUM_CHANNELS;

  while (isPlaying) {
    if (isPaused) {
      GPIO.out_w1tc = ALL_CH_MASK;
      if (extras) { extraChannels = 0; writeExtraChannels(0); }
      while (isPaused && isPlaying) vTaskDelay(pdMS_TO_TICKS(10));
      idealQ = cyc64() << 16;
      if (!isPlaying) break;
    }

    if (ringTail == ringHead) {
      // The card fell behind: hold the outputs until it catches up, then
      // restart the clock from now (no catching up with a burst).
      underruns = underruns + 1;
      while (isPlaying && !isPaused && ringTail == ringHead) { }
      idealQ = cyc64() << 16;
      continue;
    }

    StreamEvt e = ring[ringTail & (STREAM_RING - 1)];
    __sync_synchronize();
    ringTail = ringTail + 1;

    if (e.flags & 1) { if (!first) loopCount = loopCount + 1; first = false; }
    uint8_t lo = e.bits & 0x0F;
    if (streamSetLut[lo]) GPIO.out_w1ts = streamSetLut[lo];
    if (streamClrLut[lo]) GPIO.out_w1tc = streamClrLut[lo];
    if (extras) {
      extraChannels = e.bits >> NUM_CHANNELS;
      writeExtraChannels(extraChannels);
    }

    float t = tickUs;
    if (t != lastTick) { lastTick = t; tickQ = t * cyclesPerUs * 65536.0f; }
    idealQ += (uint64_t)(e.dur * tickQ + 0.5f);
    target = (idealQ + 32768) >> 16;
    playbackStep = playbackStep + 1;
    while ((int64_t)(cyc64() - target) < 0) {
      // tight spin — no yield, no sleep
    }
  }

  GPIO.out_w1tc = ALL_CH_MASK;
  if (extras) { extraChannels = 0; writeExtraChannels(0); }
  enableCore0WDT();
  sigTaskHandle = NULL;
  vTaskDelete(NULL);
}

// Starts the reader and waits until the ring is full (or 2 s) before playing.
bool startStreaming() {
  if (!ring) ring = (StreamEvt *)malloc(STREAM_RING * sizeof(StreamEvt));
  if (!ring) { Serial.println(F("[STREAM] No memory for the buffer")); return false; }
  for (int lo = 0; lo < 16; lo++) {
    uint32_t sm = 0, cm = 0;
    for (int c = 0; c < NUM_CHANNELS; c++) {
      if (lo & (1 << c)) sm |= (1UL << sigPins[c]); else cm |= (1UL << sigPins[c]);
    }
    streamSetLut[lo] = sm; streamClrLut[lo] = cm;
  }
  ringHead = 0;
  ringTail = 0;
  underruns = 0;
  readerRunning = true;
  // Same priority as the Arduino loop (1), which sleeps 1 ms per pass while a
  // file streams: the reader gets nearly all of core 1, yet the screen, the
  // encoder and the serial console never starve. With a higher priority, a
  // card that could not keep up (very fast signals) left them frozen.
  xTaskCreatePinnedToCore(sdReaderTask, "SdRead", 6144, NULL, 1, &readerTaskHandle, 1);
  uint32_t t0 = millis();
  while (readerRunning && ringHead < STREAM_RING - 8 && millis() - t0 < 2000) delay(5);
  if (!readerRunning) return false;
  Serial.printf("[STREAM] Buffer primed with %u events\n", (unsigned)ringHead);
  return true;
}

void stopStreaming() {
  readerRunning = false;
  uint32_t t0 = millis();
  while (readerTaskHandle != NULL && millis() - t0 < 1500) delay(5);
  if (ring) { free(ring); ring = NULL; }
  if (underruns) {
    Serial.printf("[STREAM] %u underruns: the card could not keep up at this speed\n",
                  (unsigned)underruns);
  }
}

void freeSigBuffers() {
  if (evtDurF)      { free(evtDurF);      evtDurF      = NULL; }
  if (evtSetMasks)  { free(evtSetMasks);  evtSetMasks  = NULL; }
  if (evtClrMasks)  { free(evtClrMasks);  evtClrMasks  = NULL; }
  if (evtExtra)     { free(evtExtra);     evtExtra     = NULL; }
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
  // Channels 5..16 only take memory when the file has them
  if (fileChannels > NUM_CHANNELS) {
    evtExtra = (uint16_t *)malloc(n * sizeof(uint16_t));
    if (!evtExtra) {
      Serial.println(F("[SIG] malloc failed for channels 5-16"));
      freeSigBuffers();
      return false;
    }
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
    if (evtExtra) evtExtra[i] = signalEvents[i].extra;
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

  if (signalMode == MODE_DIGITAL && streamMode) {
    // Big file: nothing to prepare here, the ring is filled when it starts
  } else if (signalMode == MODE_DIGITAL) {
    if (!prepareSigBuffers()) {
      Serial.println(F("[PLAY] Buffer allocation failed!"));
      return;
    }
  } else if (anaCount == 0 || anaChannels == 0) {
    Serial.println(F("[PLAY] No analog samples loaded!"));
    return;
  }

  // Compute initial tickUs from the current speed
  recalcTick();

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
  if (signalMode == MODE_DIGITAL && streamMode && !startStreaming()) {
    stopStreaming();
    Serial.println(F("[PLAY] Could not start reading the file"));
    return;
  }
  isPlaying = true;
  appState = STATE_PLAYING;
  resetPlayCursor();
  drawPlaying();     // draw BEFORE launching task (no I2C during playback)

  // Launch signal task on Core 0 with highest priority
  bool analog = (signalMode == MODE_ANALOG);
  xTaskCreatePinnedToCore(
    analog ? anaGenTask : (streamMode ? streamGenTask : sigGenTask),   // task function
    analog ? "AnaGen" : "SigGen",       // name
    analog ? 4096 : 2048,               // stack size (dacWrite needs more)
    NULL,              // param
    configMAX_PRIORITIES - 1,  // highest priority
    &sigTaskHandle,    // handle
    0                  // Core 0 (Arduino runs on Core 1)
  );

  Serial.printf("[PLAY] Started on Core 0: %s%s, %d %s, %s, "
                "tick=%.3f us\n",
                analog ? "analog" : "digital", streamMode ? " (streamed)" : "",
                signalTotalEvents, analog ? "segments" : "events",
                speedLabel().c_str(), (float)tickUs);
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
  if (streamMode) stopStreaming();
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
    if (!openSignalFile(path, sel.name, true)) {
      // Parse failed, fall back to text viewer
      readFileContent(signalFilePath);
      resetHeaderScroll(sel.name);
      appState = STATE_VIEW_FILE;
      drawFileViewer();
    }
  }
}

// Parses a signal file and plays it right away. An RPM in the metadata
// becomes the start speed.
// With the OLED (askUnit) the unit selection screen comes first; from the
// serial console, or without a screen, it plays right away in the last unit.
bool openSignalFile(const String &path, const String &name, bool askUnit) {
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
#if !SOC_DAC_SUPPORTED
  if (analog) {
    Serial.println(F("[FILE] Analog file: this board has no DAC (ESP32-S3), digital files only"));
    if (oledPresent) {
      u8g2.clearBuffer();
      u8g2.drawStr(4, 28, "Analog file:");
      u8g2.drawStr(4, 44, "needs ESP32 (DAC)");
      u8g2.sendBuffer();
      delay(2000);
    }
    return false;
  }
#endif
  bool ok;
  streamMode = false;
  if (analog) {
    ok = parseAnalogFile(path);
  } else {
    // One fast pass first. Small files then load into RAM as always; big
    // ones (their events would not fit) play streamed from the card.
    ScanInfo info;
    ok = scanSignalFile(path, &info);
    uint32_t heap = ESP.getFreeHeap();
    bool fits = ok && info.rows <= 3000 && info.rows * 80UL < heap / 2;
    if (ok && fits) {
      ok = parseSignalFile(path);
    } else if (ok) {
      freeSigBuffers();
      freeAnaBuffers();
      signalEvents.clear();
      streamInfo = info;
      streamMode = true;
      fileChannels = info.channels;
      bool endMarker = info.rows >= 2 && info.lastBits == info.firstBits;
      signalTotalEvents = (int)(endMarker ? info.rows - 1 : info.rows);
      signalTotalStepsF = (float)(info.lastStep - info.firstStep);   // same as the in-memory parser
      signalMode = MODE_DIGITAL;
      if (signalTotalStepsF < 1e-3f) {
        Serial.println(F("[SIG] Signal has no length"));
        ok = false;
      } else {
        Serial.printf("[SIG] Too big for RAM (%u rows, %u bytes free): streaming from the card\n",
                      (unsigned)info.rows, (unsigned)heap);
      }
    }
  }
  if (!ok) {
    streamMode = false;
    Serial.println(F("[FILE] Not a playable signal file"));
    return false;
  }

  // Start speed (the amount, not the unit): the file's STEP_TIME first, else
  // its RPM turned into µs/step. Without either, the last speed is kept.
  fileStartUs = 0.0f;
  if (metaStepTime > 0.0f) {
    fileStartUs = metaStepTime;
  } else if (metaRPM > 0 && signalTotalStepsF > 1e-3f) {
    fileStartUs = 60000000.0f / ((float)metaRPM * signalTotalStepsF);
  }
  Serial.printf("[FILE] Start speed: %s\n",
                metaStepTime > 0.0f ? "STEP_TIME" : (metaRPM > 0 ? "RPM" : "last used"));

  if (askUnit && oledPresent) {
    unitCursor = speedUnit;   // the last unit used comes preselected
    appState = STATE_UNIT_SELECT;
    drawUnitSelect();
    return true;
  }
  playWithUnit(speedUnit);
  return true;
}

// Applies the file's start speed in `unit` and plays.
void playWithUnit(uint8_t unit) {
  if (fileStartUs > 0.0f) {
    setSpeedFromUs(unit, fileStartUs);
  } else {
    setSpeedUnit(unit);   // no speed in the file: the last one, in this unit
  }
  saveSpeedUnit();
  startPlayback();
}

// ──────────────────────────────────────────────
//  DRAW UNIT SELECTION SCREEN
// ──────────────────────────────────────────────
// The four units, each with the start speed it would play at; the encoder
// moves, a click plays, holding goes back to the files.
void drawUnitSelect() {
  if (!oledPresent) return;
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.setDrawColor(1);
  const char *title = "Speed unit";
  u8g2.drawStr(64 - u8g2.getStrWidth(title) / 2, 7, title);
  u8g2.drawHLine(0, HEADER_H - 1, 128);

  // What each unit would show, without touching the live speed
  float us = fileStartUs > 0.0f ? fileStartUs : (float)tickUs;
  u8g2.setFont(u8g2_font_6x10_tr);
  for (uint8_t u = 0; u < UNIT_COUNT; u++) {
    int y = HEADER_H + 2 + u * 11;
    String value;
    if (u == UNIT_RPM) {
      long rpm = signalTotalStepsF > 1e-3f && us > 0.0f
                 ? lroundf(60000000.0f / (us * signalTotalStepsF)) : (long)currentRPM;
      if (rpm < RPM_MIN) rpm = RPM_MIN;
      if (rpm > RPM_MAX) rpm = RPM_MAX;
      value = String(rpm);
    } else {
      float v = u == UNIT_US  ? us
              : u == UNIT_BPS ? 1000000.0f / us
              :                 1000000.0f / (2.0f * us);
      if (v < UNIT_MIN[u]) v = UNIT_MIN[u];
      if (v > UNIT_MAX[u]) v = UNIT_MAX[u];
      value = String(v, u == UNIT_BPS ? 0 : 2);
    }
    if (u == unitCursor) {
      u8g2.drawBox(0, y, 128, 11);
      u8g2.setDrawColor(0);
    }
    u8g2.drawStr(4, y + 9, UNIT_LABELS[u]);
    u8g2.drawStr(124 - u8g2.getStrWidth(value.c_str()), y + 9, value.c_str());
    u8g2.setDrawColor(1);
  }
  u8g2.sendBuffer();
}

// ──────────────────────────────────────────────
//  GO BACK (long click)
// ──────────────────────────────────────────────
void goBack() {
  if (appState == STATE_PLAYING) {
    // Stop playback
    stopPlayback();
    appState = STATE_BROWSER;
    resetHeaderScroll(currentPath);
    drawBrowser();
    return;
  }

  if (appState == STATE_UNIT_SELECT) {
    // Back to the files without playing
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


  } else if (appState == STATE_UNIT_SELECT) {
    // Move between the four units, without wrapping
    int c = (int)unitCursor + (ticks > 0 ? 1 : -1);
    if (c < 0) c = 0;
    if (c >= UNIT_COUNT) c = UNIT_COUNT - 1;
    unitCursor = (uint8_t)c;
    drawUnitSelect();

  } else if (appState == STATE_PLAYING) {
    if (editingDigit) {
      // Change the blinking digit — live, the signal task reads tickUs on the fly
      editSpeedDigit(ticks > 0 ? 1 : -1);
      blinkOn = true;            // show the new value right away
      blinkAt = millis();
    } else {
      // Move between the digits, the unit and Pause, without wrapping
      int c = (int)playCursor + (ticks > 0 ? 1 : -1);
      int last = speedDigitCount() + 1;
      if (c < 0) c = 0;
      if (c > last) c = last;
      playCursor = (uint8_t)c;
    }
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

  if (appState == STATE_UNIT_SELECT) {
    // Click → play in the highlighted unit
    playWithUnit(unitCursor);
    return;
  }

  if (appState == STATE_PLAYING) {
    // Click acts on what the cursor is on: a digit (start/finish editing it),
    // the unit (next one, same speed) or Pause/Resume. Hold exits.
    uint8_t ndig = speedDigitCount();
    if (playCursor < ndig) {
      editingDigit = !editingDigit;
      blinkOn = true;
      blinkAt = millis();
      if (!editingDigit) {
        Serial.printf("[CMD] Speed: %s (tick %.4f us/step)\n", speedLabel().c_str(), (float)tickUs);
      }
    } else if (playCursor == ndig) {
      setSpeedUnit((speedUnit + 1) % UNIT_COUNT);
      saveSpeedUnit();
      playCursor = speedDigitCount();   // stay on the unit
    } else {
      isPaused = !isPaused;
    }
    drawPlaying();
    return;
  }
  // In file viewer, click does nothing (long click to go back)
}

// Double click while playing: next speed unit. Anywhere else it counts as a
// single click, so a quick double press in the browser still opens.
void onButtonDoubleClick(Button2 &btn) {
  if (appState != STATE_PLAYING) { onButtonClick(btn); return; }
  // The digit count changes with the unit: a cursor on the unit or on Pause
  // stays there, one on a digit goes back to the first significant digit.
  uint8_t past = playCursor >= speedDigitCount() ? playCursor - speedDigitCount() : 255;
  setSpeedUnit((speedUnit + 1) % UNIT_COUNT);
  saveSpeedUnit();
  if (past != 255) { playCursor = speedDigitCount() + past; editingDigit = false; }
  else resetPlayCursor();
  drawPlaying();
  Serial.printf("[UNIT] %s (tick %.4f us/step)\n", speedLabel().c_str(), (float)tickUs);
}

void onButtonLongClick(Button2 &btn) {
  // While a digit is being edited, holding only finishes the edit
  if (appState == STATE_PLAYING && editingDigit) {
    editingDigit = false;
    drawPlaying();
    return;
  }
  if (appState == STATE_BROWSER || appState == STATE_VIEW_FILE ||
      appState == STATE_UNIT_SELECT || appState == STATE_PLAYING) {
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
  Serial.println(F("  stop               stop playback"));
  Serial.println(F("  pause | resume     pause / resume playback (also: p)"));
  Serial.println(F("  rpm <100-8000>     set the speed (live while playing)"));
  Serial.println(F("  rpm + | rpm -      speed up / down by 100"));
  Serial.println(F("  speed <v> [unit]   set the speed, in us | rpm | bps | hz"));
  Serial.println(F("  speed + | speed -  speed up / down, in the current unit"));
  Serial.println(F("  unit <us|rpm|bps|hz>  same speed, shown in another unit"));
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
  else if (appState == STATE_UNIT_SELECT) state = "picking the speed unit on the screen";
  else if (appState == STATE_VIEW_FILE)  state = "viewing file";

  Serial.println(F("[STATUS] ─────────────────────────"));
  Serial.printf("[STATUS] state        : %s\n", state);
  Serial.printf("[STATUS] folder       : %s\n", currentPath.c_str());
  if (signalFileName.length() > 0) {
    Serial.printf("[STATUS] file         : %s\n", signalFileName.c_str());
    Serial.printf("[STATUS] mode         : %s\n", modeLabel().c_str());
    Serial.printf("[STATUS] steps/cycle  : %.4f\n", signalTotalStepsF);
  }
  Serial.printf("[STATUS] speed        : %s\n", speedLabel().c_str());
  if (appState == STATE_PLAYING) {
    Serial.printf("[STATUS] tick         : %.4f us/step\n", (float)tickUs);
    Serial.printf("[STATUS] loops        : %u\n", (unsigned)loopCount);
    if (streamMode) {
      Serial.printf("[STATUS] streamed     : yes, %u in buffer, %u underruns\n",
                    (unsigned)(ringHead - ringTail), (unsigned)underruns);
    }
  }
  Serial.printf("[STATUS] OLED         : %s\n", oledPresent ? "yes" : "no");
}

// Applies a new RPM: live while playing, stored for the next start otherwise.
static void setRPM(long rpm) {
  if (rpm < RPM_MIN) rpm = RPM_MIN;
  if (rpm > RPM_MAX) rpm = RPM_MAX;
  currentRPM = (uint32_t)rpm;
  speedUnit = UNIT_RPM;
  recalcTick();   // the signal task reads it on-the-fly
  if (appState == STATE_PLAYING) drawPlaying();
  Serial.printf("[CMD] Speed: %s (tick %.4f us/step)\n", speedLabel().c_str(), (float)tickUs);
}

// "us", "rpm", "bps", "hz" → unit index, or UNIT_COUNT if it is none of them.
static uint8_t parseUnit(String name) {
  name.toLowerCase();
  if (name == "us/step") name = "us";
  for (uint8_t u = 0; u < UNIT_COUNT; u++) if (name == UNIT_NAMES[u]) return u;
  return UNIT_COUNT;
}

// "speed <value> [unit]" · "speed +" · "speed -" · "speed" (show)
static void speedCommand(String arg) {
  if (arg == "+" || arg == "-") {
    stepSpeed(arg == "+" ? 1 : -1);
  } else if (arg.length() > 0) {
    int sp = arg.indexOf(' ');
    String value = sp < 0 ? arg : arg.substring(0, sp);
    String unitName = sp < 0 ? String("") : arg.substring(sp + 1);
    unitName.trim();
    if (unitName.length() > 0) {
      uint8_t u = parseUnit(unitName);
      if (u == UNIT_COUNT) { Serial.println(F("[CMD] Unit: us | rpm | bps | hz")); return; }
      speedUnit = u;
      saveSpeedUnit();
    }
    float v = value.toFloat();
    if (speedUnit == UNIT_RPM) { setRPM(lroundf(v)); return; }
    if (v < UNIT_MIN[speedUnit]) v = UNIT_MIN[speedUnit];
    if (v > UNIT_MAX[speedUnit]) v = UNIT_MAX[speedUnit];
    speedValue = v;
    recalcTick();
  }
  if (appState == STATE_PLAYING) drawPlaying();
  Serial.printf("[CMD] Speed: %s (tick %.4f us/step)\n", speedLabel().c_str(), (float)tickUs);
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

  } else if ((cmd == "ls" || cmd == "cd") && streamMode && isPlaying) {
    // The card is being read for the signal: one reader at a time
    Serial.println(F("[CMD] Stop playback first (the file is being read from the card)"));

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
    if (!openSignalFile(path, name, false)) showBrowserIfIdle();

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

  } else if (cmd == "speed") {
    speedCommand(arg);

  } else if (cmd == "unit") {
    if (arg.length() > 0) {
      uint8_t u = parseUnit(arg);
      if (u == UNIT_COUNT) { Serial.println(F("[CMD] Unit: us | rpm | bps | hz")); return; }
      setSpeedUnit(u);
      saveSpeedUnit();
      if (appState == STATE_PLAYING) drawPlaying();
    }
    Serial.printf("[CMD] Speed: %s (tick %.4f us/step)\n", speedLabel().c_str(), (float)tickUs);

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
