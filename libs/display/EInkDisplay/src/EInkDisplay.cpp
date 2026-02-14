#include "EInkDisplay.h"

#include <cstring>
#include <fstream>
#include <vector>

// SSD1677 command definitions
// Initialization and reset
#define CMD_SOFT_RESET 0x12             // Soft reset
#define CMD_BOOSTER_SOFT_START 0x0C     // Booster soft-start control
#define CMD_DRIVER_OUTPUT_CONTROL 0x01  // Driver output control
#define CMD_BORDER_WAVEFORM 0x3C        // Border waveform control
#define CMD_TEMP_SENSOR_CONTROL 0x18    // Temperature sensor control

// RAM and buffer management
#define CMD_DATA_ENTRY_MODE 0x11     // Data entry mode
#define CMD_SET_RAM_X_RANGE 0x44     // Set RAM X address range
#define CMD_SET_RAM_Y_RANGE 0x45     // Set RAM Y address range
#define CMD_SET_RAM_X_COUNTER 0x4E   // Set RAM X address counter
#define CMD_SET_RAM_Y_COUNTER 0x4F   // Set RAM Y address counter
#define CMD_WRITE_RAM_BW 0x24        // Write to BW RAM (current frame)
#define CMD_WRITE_RAM_RED 0x26       // Write to RED RAM (used for fast refresh)
#define CMD_AUTO_WRITE_BW_RAM 0x46   // Auto write BW RAM
#define CMD_AUTO_WRITE_RED_RAM 0x47  // Auto write RED RAM

// Display update and refresh
#define CMD_DISPLAY_UPDATE_CTRL1 0x21  // Display update control 1
#define CMD_DISPLAY_UPDATE_CTRL2 0x22  // Display update control 2
#define CMD_MASTER_ACTIVATION 0x20     // Master activation
#define CTRL1_NORMAL 0x00              // Normal mode - compare RED vs BW for partial
#define CTRL1_BYPASS_RED 0x40          // Bypass RED RAM (treat as 0) - for full refresh

// LUT and voltage settings
#define CMD_WRITE_LUT 0x32       // Write LUT
#define CMD_GATE_VOLTAGE 0x03    // Gate voltage
#define CMD_SOURCE_VOLTAGE 0x04  // Source voltage
#define CMD_WRITE_VCOM 0x2C      // Write VCOM
#define CMD_WRITE_TEMP 0x1A      // Write temperature

// Power management
#define CMD_DEEP_SLEEP 0x10  // Deep sleep

// Custom LUT for fast refresh
const unsigned char lut_grayscale[] PROGMEM = {
    // 00 black/white
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 01 light gray
    0x54, 0x54, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 10 gray
    0xAA, 0xA0, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 11 dark gray
    0xA2, 0x22, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // L4 (VCOM)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // TP/RP groups (global timing)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G0: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G1: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G2: A=0 B=0 C=0 D=0 RP=0 (4 frames)
    0x00, 0x00, 0x00, 0x00, 0x00,  // G3: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G4: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G5: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G6: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G7: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G8: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G9: A=0 B=0 C=0 D=0 RP=0

    // Frame rate
    0x8F, 0x8F, 0x8F, 0x8F, 0x8F,

    // Voltages (VGH, VSH1, VSH2, VSL, VCOM)
    0x17, 0x41, 0xA8, 0x32, 0x30,

    // Reserved
    0x00, 0x00};

const unsigned char lut_grayscale_revert[] PROGMEM = {
    // 00 black/white
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 10 gray
    0x54, 0x54, 0x54, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 01 light gray
    0xA8, 0xA8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // 11 dark gray
    0xFC, 0xFC, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    // L4 (VCOM)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // TP/RP groups (global timing)
    0x01, 0x01, 0x01, 0x01, 0x01,  // G0: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x01,  // G1: A=1 B=1 C=1 D=1 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G2: A=0 B=0 C=0 D=0 RP=0 (4 frames)
    0x01, 0x01, 0x01, 0x01, 0x00,  // G3: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G4: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G5: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G6: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G7: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G8: A=0 B=0 C=0 D=0 RP=0
    0x00, 0x00, 0x00, 0x00, 0x00,  // G9: A=0 B=0 C=0 D=0 RP=0

    // Frame rate
    0x8F, 0x8F, 0x8F, 0x8F, 0x8F,

    // Voltages (VGH, VSH1, VSH2, VSL, VCOM)
    0x17, 0x41, 0xA8, 0x32, 0x30,

    // Reserved
    0x00, 0x00};

EInkDisplay::EInkDisplay(int8_t sclk, int8_t mosi, int8_t cs, int8_t dc, int8_t rst, int8_t busy)
    : _sclk(sclk),
      _mosi(mosi),
      _cs(cs),
      _dc(dc),
      _rst(rst),
      _busy(busy),
      frameBuffer(nullptr),
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
      frameBufferActive(nullptr),
#endif
      isScreenOn(false),
      customLutActive(false),
      inGrayscaleMode(false),
      drawGrayscale(false),
      refreshInProgress(false),
      ramDataEntryConfigured(false),
      ramAreaConfigured(false),
      ramAreaX(0),
      ramAreaY(0),
      ramAreaW(0),
      ramAreaH(0) {
  if (Serial) Serial.printf("[%lu] EInkDisplay: Constructor called\n", millis());
  if (Serial) Serial.printf("[%lu]   SCLK=%d, MOSI=%d, CS=%d, DC=%d, RST=%d, BUSY=%d\n", millis(), sclk, mosi, cs, dc, rst, busy);
}

void EInkDisplay::begin() {
  if (Serial) Serial.printf("[%lu] EInkDisplay: begin() called\n", millis());

  frameBuffer = frameBuffer0;
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  frameBufferActive = frameBuffer1;
#endif
  ramDataEntryConfigured = false;
  ramAreaConfigured = false;

  // Initialize to white
  memset(frameBuffer0, 0xFF, BUFFER_SIZE);
#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  if (Serial) Serial.printf("[%lu]   Static frame buffer (%lu bytes = 48KB)\n", millis(), BUFFER_SIZE);
#else
  memset(frameBuffer1, 0xFF, BUFFER_SIZE);
  if (Serial) Serial.printf("[%lu]   Static frame buffers (2 x %lu bytes = 96KB)\n", millis(), BUFFER_SIZE);
#endif

  if (Serial) Serial.printf("[%lu]   Initializing e-ink display driver...\n", millis());

  // Initialize SPI with custom pins
  SPI.begin(_sclk, -1, _mosi, _cs);
  spiSettings = SPISettings(40000000, MSBFIRST, SPI_MODE0);  // MODE0 is standard for SSD1677
  if (Serial) Serial.printf("[%lu]   SPI initialized at 40 MHz, Mode 0\n", millis());

  // Setup GPIO pins
  pinMode(_cs, OUTPUT);
  pinMode(_dc, OUTPUT);
  pinMode(_rst, OUTPUT);
  pinMode(_busy, INPUT);

  digitalWrite(_cs, HIGH);
  digitalWrite(_dc, HIGH);

  if (Serial) Serial.printf("[%lu]   GPIO pins configured\n", millis());

  // Reset display
  resetDisplay();

  // Initialize display controller
  initDisplayController();

  if (Serial) Serial.printf("[%lu]   E-ink display driver initialized\n", millis());
}

// ============================================================================
// Low-level display control methods
// ============================================================================

void EInkDisplay::resetDisplay() {
  if (Serial) Serial.printf("[%lu]   Resetting display...\n", millis());
  digitalWrite(_rst, HIGH);
  delay(20);
  digitalWrite(_rst, LOW);
  delay(2);
  digitalWrite(_rst, HIGH);
  delay(20);
  if (Serial) Serial.printf("[%lu]   Display reset complete\n", millis());
}

void EInkDisplay::sendCommand(uint8_t command) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, LOW);  // Command mode
  digitalWrite(_cs, LOW);  // Select chip
  SPI.transfer(command);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendData(uint8_t data) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);  // Data mode
  digitalWrite(_cs, LOW);   // Select chip
  SPI.transfer(data);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendData(const uint8_t* data, uint32_t length) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, HIGH);       // Data mode
  digitalWrite(_cs, LOW);        // Select chip
  SPI.writeBytes(data, length);  // Transfer all bytes
  digitalWrite(_cs, HIGH);       // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendCommandData(uint8_t command, uint8_t data) {
#ifdef EINK_DISPLAY_FORCE_BASELINE
  sendCommand(command);
  sendData(data);
  return;
#endif
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, LOW);   // Command mode
  digitalWrite(_cs, LOW);   // Select chip
  SPI.transfer(command);
  digitalWrite(_dc, HIGH);  // Data mode
  SPI.transfer(data);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::sendCommandData(uint8_t command, const uint8_t* data, uint32_t length) {
#ifdef EINK_DISPLAY_FORCE_BASELINE
  sendCommand(command);
  sendData(data, length);
  return;
#endif
  SPI.beginTransaction(spiSettings);
  digitalWrite(_dc, LOW);   // Command mode
  digitalWrite(_cs, LOW);   // Select chip
  SPI.transfer(command);
  digitalWrite(_dc, HIGH);  // Data mode
  SPI.writeBytes(data, length);
  digitalWrite(_cs, HIGH);  // Deselect chip
  SPI.endTransaction();
}

void EInkDisplay::waitWhileBusy(const char* comment) {
  unsigned long start = millis();
  while (digitalRead(_busy) == HIGH) {
    delay(1);
    if (millis() - start > 10000) {
      if (Serial) Serial.printf("[%lu]   Timeout waiting for busy%s\n", millis(), comment ? comment : "");
      break;
    }
  }
  if (comment) {
    if (Serial) Serial.printf("[%lu]   Wait complete: %s (%lu ms)\n", millis(), comment, millis() - start);
  }
}

void EInkDisplay::initDisplayController() {
  if (Serial) Serial.printf("[%lu]   Initializing SSD1677 controller...\n", millis());

  const uint8_t TEMP_SENSOR_INTERNAL = 0x80;

  // Soft reset
  sendCommand(CMD_SOFT_RESET);
  waitWhileBusy(" CMD_SOFT_RESET");

  // Temperature sensor control (internal)
  sendCommandData(CMD_TEMP_SENSOR_CONTROL, TEMP_SENSOR_INTERNAL);

  // Booster soft-start control (GDEQ0426T82 specific values)
  const uint8_t boosterSoftStart[] = {0xAE, 0xC7, 0xC3, 0xC0, 0x40};
  sendCommandData(CMD_BOOSTER_SOFT_START, boosterSoftStart, sizeof(boosterSoftStart));

  // Driver output control: set display height (480) and scan direction
  const uint16_t HEIGHT = 480;
  const uint8_t driverOutput[] = {
      static_cast<uint8_t>((HEIGHT - 1) & 0xFF), static_cast<uint8_t>((HEIGHT - 1) >> 8), 0x02};
  sendCommandData(CMD_DRIVER_OUTPUT_CONTROL, driverOutput, sizeof(driverOutput));

  // Border waveform control
  sendCommandData(CMD_BORDER_WAVEFORM, 0x01);

  // Set up full screen RAM area
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);

  if (Serial) Serial.printf("[%lu]   Clearing RAM buffers...\n", millis());
  sendCommandData(CMD_AUTO_WRITE_BW_RAM, 0xF7);  // Auto write BW RAM
  waitWhileBusy(" CMD_AUTO_WRITE_BW_RAM");

  sendCommandData(CMD_AUTO_WRITE_RED_RAM, 0xF7);  // Fill with white pattern
  waitWhileBusy(" CMD_AUTO_WRITE_RED_RAM");

  if (Serial) Serial.printf("[%lu]   SSD1677 controller initialized\n", millis());
}

void EInkDisplay::setRamArea(const uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  constexpr uint8_t DATA_ENTRY_X_INC_Y_DEC = 0x01;
  if (w == 0 || h == 0) {
    return;
  }

#ifdef EINK_DISPLAY_FORCE_BASELINE
  // Baseline path: always rewrite data-entry mode, ranges and counters.
  const uint16_t yReversed = DISPLAY_HEIGHT - y - h;
  const uint16_t xEnd = x + w - 1;
  const uint16_t yStart = yReversed + h - 1;
  const uint8_t ramXRange[] = {static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>(x >> 8),
                               static_cast<uint8_t>(xEnd & 0xFF), static_cast<uint8_t>(xEnd >> 8)};
  const uint8_t ramYRange[] = {static_cast<uint8_t>(yStart & 0xFF), static_cast<uint8_t>(yStart >> 8),
                               static_cast<uint8_t>(yReversed & 0xFF), static_cast<uint8_t>(yReversed >> 8)};
  const uint8_t ramXCounter[] = {static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>(x >> 8)};
  const uint8_t ramYCounter[] = {static_cast<uint8_t>(yStart & 0xFF), static_cast<uint8_t>(yStart >> 8)};

  sendCommand(CMD_DATA_ENTRY_MODE);
  sendData(DATA_ENTRY_X_INC_Y_DEC);

  sendCommand(CMD_SET_RAM_X_RANGE);
  sendData(ramXRange, sizeof(ramXRange));

  sendCommand(CMD_SET_RAM_Y_RANGE);
  sendData(ramYRange, sizeof(ramYRange));

  sendCommand(CMD_SET_RAM_X_COUNTER);
  sendData(ramXCounter, sizeof(ramXCounter));

  sendCommand(CMD_SET_RAM_Y_COUNTER);
  sendData(ramYCounter, sizeof(ramYCounter));
  return;
#else
  // Reverse Y coordinate (gates are reversed on this display)
  const uint16_t yReversed = DISPLAY_HEIGHT - y - h;
  const uint16_t xEnd = x + w - 1;
  const uint16_t yStart = yReversed + h - 1;
  const uint8_t ramXCounter[] = {static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>(x >> 8)};
  const uint8_t ramYCounter[] = {static_cast<uint8_t>(yStart & 0xFF), static_cast<uint8_t>(yStart >> 8)};
  const bool windowChanged = !ramAreaConfigured || x != ramAreaX || y != ramAreaY || w != ramAreaW || h != ramAreaH;

  if (!ramDataEntryConfigured) {
    // Set data entry mode (X increment, Y decrement for reversed gates)
    sendCommandData(CMD_DATA_ENTRY_MODE, DATA_ENTRY_X_INC_Y_DEC);
    ramDataEntryConfigured = true;
  }

  if (windowChanged) {
    const uint8_t ramXRange[] = {static_cast<uint8_t>(x & 0xFF), static_cast<uint8_t>(x >> 8),
                                 static_cast<uint8_t>(xEnd & 0xFF), static_cast<uint8_t>(xEnd >> 8)};
    const uint8_t ramYRange[] = {static_cast<uint8_t>(yStart & 0xFF), static_cast<uint8_t>(yStart >> 8),
                                 static_cast<uint8_t>(yReversed & 0xFF), static_cast<uint8_t>(yReversed >> 8)};

    // Set RAM X/Y address range only when window changes.
    sendCommandData(CMD_SET_RAM_X_RANGE, ramXRange, sizeof(ramXRange));
    sendCommandData(CMD_SET_RAM_Y_RANGE, ramYRange, sizeof(ramYRange));

    ramAreaConfigured = true;
    ramAreaX = x;
    ramAreaY = y;
    ramAreaW = w;
    ramAreaH = h;
  }

  // Set RAM X address counter - X is in PIXELS
  sendCommandData(CMD_SET_RAM_X_COUNTER, ramXCounter, sizeof(ramXCounter));

  // Set RAM Y address counter - Y is in PIXELS
  sendCommandData(CMD_SET_RAM_Y_COUNTER, ramYCounter, sizeof(ramYCounter));
#endif
}

void EInkDisplay::clearScreen(const uint8_t color) const {
  memset(frameBuffer, color, BUFFER_SIZE);
}

void EInkDisplay::drawImage(const uint8_t* imageData, const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h,
                            const bool fromProgmem) const {
  if (!frameBuffer) {
    if (Serial) Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

  const uint16_t xByte = x / 8;
  const uint16_t imageWidthBytes = w / 8;
  if (xByte >= DISPLAY_WIDTH_BYTES || y >= DISPLAY_HEIGHT || imageWidthBytes == 0 || h == 0) {
    return;
  }

  const uint16_t rowsToCopy = (y + h > DISPLAY_HEIGHT) ? (DISPLAY_HEIGHT - y) : h;
  const uint16_t colsToCopy = (xByte + imageWidthBytes > DISPLAY_WIDTH_BYTES) ? (DISPLAY_WIDTH_BYTES - xByte) : imageWidthBytes;

  // Copy image data to frame buffer
  for (uint16_t row = 0; row < rowsToCopy; row++) {
    const uint16_t destOffset = (y + row) * DISPLAY_WIDTH_BYTES + xByte;
    const uint16_t srcOffset = row * imageWidthBytes;
    uint8_t* dest = &frameBuffer[destOffset];
    const uint8_t* src = &imageData[srcOffset];

    if (!fromProgmem) {
      memcpy(dest, src, colsToCopy);
    } else {
      for (uint16_t col = 0; col < colsToCopy; col++) {
        dest[col] = pgm_read_byte(&src[col]);
      }
    }
  }

  if (Serial) Serial.printf("[%lu]   Image drawn to frame buffer\n", millis());
}

// Draws only black pixels from the image, leaves white pixels clear (unchanged in framebuffer)
void EInkDisplay::drawImageTransparent(const uint8_t* imageData, const uint16_t x, const uint16_t y, const uint16_t w, const uint16_t h,
                                     const bool fromProgmem) const {
  if (!frameBuffer) {
    if (Serial) Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

  const uint16_t xByte = x / 8;
  const uint16_t imageWidthBytes = w / 8;
  if (xByte >= DISPLAY_WIDTH_BYTES || y >= DISPLAY_HEIGHT || imageWidthBytes == 0 || h == 0) {
    return;
  }

  const uint16_t rowsToCopy = (y + h > DISPLAY_HEIGHT) ? (DISPLAY_HEIGHT - y) : h;
  const uint16_t colsToCopy = (xByte + imageWidthBytes > DISPLAY_WIDTH_BYTES) ? (DISPLAY_WIDTH_BYTES - xByte) : imageWidthBytes;

#ifdef EINK_DISPLAY_FORCE_BASELINE
  // Baseline path: branch on fromProgmem inside the hot loop.
  for (uint16_t row = 0; row < rowsToCopy; row++) {
    const uint16_t destOffset = (y + row) * DISPLAY_WIDTH_BYTES + xByte;
    const uint16_t srcOffset = row * imageWidthBytes;
    uint8_t* dest = &frameBuffer[destOffset];
    const uint8_t* src = &imageData[srcOffset];

    for (uint16_t col = 0; col < colsToCopy; col++) {
      const uint8_t srcByte = fromProgmem ? pgm_read_byte(&src[col]) : src[col];
      dest[col] &= srcByte;
    }
  }
#else
  // Optimized path: hoist fromProgmem branch out of the inner loop.
  if (!fromProgmem) {
    for (uint16_t row = 0; row < rowsToCopy; row++) {
      const uint16_t destOffset = (y + row) * DISPLAY_WIDTH_BYTES + xByte;
      const uint16_t srcOffset = row * imageWidthBytes;
      uint8_t* dest = &frameBuffer[destOffset];
      const uint8_t* src = &imageData[srcOffset];

      for (uint16_t col = 0; col < colsToCopy; col++) {
        dest[col] &= src[col];
      }
    }
  } else {
    for (uint16_t row = 0; row < rowsToCopy; row++) {
      const uint16_t destOffset = (y + row) * DISPLAY_WIDTH_BYTES + xByte;
      const uint16_t srcOffset = row * imageWidthBytes;
      uint8_t* dest = &frameBuffer[destOffset];
      const uint8_t* src = &imageData[srcOffset];

      for (uint16_t col = 0; col < colsToCopy; col++) {
        dest[col] &= pgm_read_byte(&src[col]);
      }
    }
  }
#endif

  if (Serial) Serial.printf("[%lu]   Transparent image drawn to frame buffer\n", millis());
}

void EInkDisplay::writeRamBuffer(uint8_t ramBuffer, const uint8_t* data, uint32_t size) {
  const char* bufferName = (ramBuffer == CMD_WRITE_RAM_BW) ? "BW" : "RED";
  const unsigned long startTime = millis();
  if (Serial) Serial.printf("[%lu]   Writing frame buffer to %s RAM (%lu bytes)...\n", startTime, bufferName, size);

  sendCommandData(ramBuffer, data, size);

  const unsigned long duration = millis() - startTime;
  if (Serial) Serial.printf("[%lu]   %s RAM write complete (%lu ms)\n", millis(), bufferName, duration);
}

void EInkDisplay::setFramebuffer(const uint8_t* bwBuffer) const {
  memcpy(frameBuffer, bwBuffer, BUFFER_SIZE);
}

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
void EInkDisplay::swapBuffers() {
  uint8_t* temp = frameBuffer;
  frameBuffer = frameBufferActive;
  frameBufferActive = temp;
}
#endif

void EInkDisplay::grayscaleRevert() {
  if (!inGrayscaleMode) {
    return;
  }

  inGrayscaleMode = false;

  // Load the revert LUT
  setCustomLUT(true, lut_grayscale_revert);
  refreshDisplay(FAST_REFRESH);
  setCustomLUT(false);
}

void EInkDisplay::copyGrayscaleLsbBuffers(const uint8_t* lsbBuffer) {
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  writeRamBuffer(CMD_WRITE_RAM_BW, lsbBuffer, BUFFER_SIZE);
}

void EInkDisplay::copyGrayscaleMsbBuffers(const uint8_t* msbBuffer) {
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  writeRamBuffer(CMD_WRITE_RAM_RED, msbBuffer, BUFFER_SIZE);
}

void EInkDisplay::copyGrayscaleBuffers(const uint8_t* lsbBuffer, const uint8_t* msbBuffer) {
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  writeRamBuffer(CMD_WRITE_RAM_BW, lsbBuffer, BUFFER_SIZE);
  writeRamBuffer(CMD_WRITE_RAM_RED, msbBuffer, BUFFER_SIZE);
}

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
/**
 * In single buffer mode, this should be called with the previously written BW buffer
 * to reconstruct the RED buffer for proper differential fast refreshes following a
 * grayscale display.
 */
void EInkDisplay::cleanupGrayscaleBuffers(const uint8_t* bwBuffer) {
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  writeRamBuffer(CMD_WRITE_RAM_RED, bwBuffer, BUFFER_SIZE);
}
#endif

void EInkDisplay::displayBuffer(RefreshMode mode, const bool turnOffScreen) {
  while (!pollRefreshComplete()) {
    delay(1);
  }
  // Guard against state desync: even if refreshInProgress is false, never write RAM while panel BUSY is high.
  if (isBusy()) {
    waitWhileBusy(" pre-displayBuffer busy");
    if (isBusy()) {
      if (Serial) Serial.printf("[%lu]   ERROR: Panel still busy before displayBuffer\n", millis());
      return;
    }
  }

  if (!isScreenOn && !turnOffScreen)
  {
    // Force half refresh if screen is off
    mode = HALF_REFRESH;
  }

  // If currently in grayscale mode, revert first to black/white
  if (inGrayscaleMode) {
    inGrayscaleMode = false;
    grayscaleRevert();
  }

  // Set up full screen RAM area
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);

  if (mode != FAST_REFRESH) {
    // For full/half refresh RED RAM is bypassed, so writing BW only is sufficient.
    writeRamBuffer(CMD_WRITE_RAM_BW, frameBuffer, BUFFER_SIZE);
  } else {
    // For fast refresh, write to BW buffer only
    writeRamBuffer(CMD_WRITE_RAM_BW, frameBuffer, BUFFER_SIZE);
    // In single buffer mode, the RED RAM should already contain the previous frame
    // In dual buffer mode, we write back frameBufferActive which is the last frame
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    writeRamBuffer(CMD_WRITE_RAM_RED, frameBufferActive, BUFFER_SIZE);
#endif
  }

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  swapBuffers();
#endif

  // Refresh the display
  refreshDisplay(mode, turnOffScreen);

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  // In single buffer mode always sync RED RAM after refresh to prepare for next fast refresh
  // This ensures RED contains the currently displayed frame for differential comparison
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  writeRamBuffer(CMD_WRITE_RAM_RED, frameBuffer, BUFFER_SIZE);
#endif
}

bool EInkDisplay::displayBufferAsync(RefreshMode mode, const bool turnOffScreen) {
  if (!isScreenOn && !turnOffScreen) {
    // Force half refresh if screen is off
    mode = HALF_REFRESH;
  }

  // If currently in grayscale mode, revert first to black/white
  if (inGrayscaleMode) {
    inGrayscaleMode = false;
    grayscaleRevert();
  }

  // Do not queue a new transfer while panel is still processing prior update.
  if (!pollRefreshComplete()) {
    return false;
  }
  if (isBusy()) {
    return false;
  }

  // Set up full screen RAM area
  setRamArea(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);

  if (mode != FAST_REFRESH) {
    // For full/half refresh RED RAM is bypassed, so writing BW only is sufficient.
    writeRamBuffer(CMD_WRITE_RAM_BW, frameBuffer, BUFFER_SIZE);
  } else {
    // For fast refresh, write to BW buffer only
    writeRamBuffer(CMD_WRITE_RAM_BW, frameBuffer, BUFFER_SIZE);
    // In single buffer mode, the RED RAM should already contain the previous frame
    // In dual buffer mode, we write back frameBufferActive which is the last frame
#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
    writeRamBuffer(CMD_WRITE_RAM_RED, frameBufferActive, BUFFER_SIZE);
#endif
  }

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  swapBuffers();
#endif

  return refreshDisplayAsync(mode, turnOffScreen);
}

// EXPERIMENTAL: Windowed update support
// Displays only a rectangular region of the frame buffer, preserving the rest of the screen.
// Requirements: x and w must be byte-aligned (multiples of 8 pixels)
void EInkDisplay::displayWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const bool turnOffScreen) {
  if (Serial) Serial.printf("[%lu]   Displaying window at (%d,%d) size (%dx%d)\n", millis(), x, y, w, h);

  while (!pollRefreshComplete()) {
    delay(1);
  }
  if (isBusy()) {
    waitWhileBusy(" pre-displayWindow busy");
    if (isBusy()) {
      if (Serial) Serial.printf("[%lu]   ERROR: Panel still busy before displayWindow\n", millis());
      return;
    }
  }

  // Validate bounds
  if (x + w > DISPLAY_WIDTH || y + h > DISPLAY_HEIGHT) {
    if (Serial) Serial.printf("[%lu]   ERROR: Window bounds exceed display dimensions!\n", millis());
    return;
  }

  // Validate byte alignment
  if (x % 8 != 0 || w % 8 != 0) {
    if (Serial) Serial.printf("[%lu]   ERROR: Window x and width must be byte-aligned (multiples of 8)!\n", millis());
    return;
  }

  if (!frameBuffer) {
    if (Serial) Serial.printf("[%lu]   ERROR: Frame buffer not allocated!\n", millis());
    return;
  }

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  if (!frameBufferActive) {
    if (Serial) Serial.printf("[%lu]   ERROR: Active frame buffer not allocated in dual buffer mode!\n", millis());
    return;
  }
#endif

  // displayWindow is not supported while the rest of the screen has grayscale content, revert it
  if (inGrayscaleMode) {
    inGrayscaleMode = false;
    grayscaleRevert();
  }

  // Calculate window transfer size
  const uint16_t windowWidthBytes = w / 8;
  const uint32_t windowTransferBytes = windowWidthBytes * h;
  const uint16_t xByte = x / 8;

  if (Serial) Serial.printf("[%lu]   Window transfer size: %lu bytes (%d x %d pixels)\n", millis(), windowTransferBytes, w, h);

  auto writeWindowFromBuffer = [&](const uint8_t ramBuffer, const uint8_t* sourceBuffer) {
    // reset window ranges and address counters before each ram write
    setRamArea(x, y, w, h);
    SPI.beginTransaction(spiSettings);
    digitalWrite(_dc, LOW);  // Command mode
    digitalWrite(_cs, LOW);  // Select chip
    SPI.transfer(ramBuffer);
    digitalWrite(_dc, HIGH);  // Data mode

    for (uint16_t row = 0; row < h; row++) {
      const uint32_t srcOffset = static_cast<uint32_t>(y + row) * DISPLAY_WIDTH_BYTES + xByte;
      SPI.writeBytes(&sourceBuffer[srcOffset], windowWidthBytes);
    }

    digitalWrite(_cs, HIGH);  // Deselect chip
    SPI.endTransaction();
  };

  // Configure RAM area for window
  setRamArea(x, y, w, h);

  // Write to BW RAM (current frame)
  writeWindowFromBuffer(CMD_WRITE_RAM_BW, frameBuffer);

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  // Dual buffer: stream previous frame from frameBufferActive.
  writeWindowFromBuffer(CMD_WRITE_RAM_RED, frameBufferActive);
#endif

  // Perform fast refresh
  refreshDisplay(FAST_REFRESH, turnOffScreen);

#ifdef EINK_DISPLAY_SINGLE_BUFFER_MODE
  // Post-refresh: Sync RED RAM with current window (for next fast refresh)
  setRamArea(x, y, w, h);
  writeWindowFromBuffer(CMD_WRITE_RAM_RED, frameBuffer);
#endif
  // post-refresh: sync RED RAM with current window
  writeWindowFromBuffer(CMD_WRITE_RAM_RED, frameBuffer);

#ifndef EINK_DISPLAY_SINGLE_BUFFER_MODE
  for (uint16_t row = 0; row < h; row++) {
    const uint32_t srcOffset = static_cast<uint32_t>(y + row) * DISPLAY_WIDTH_BYTES + xByte;
    memcpy(&frameBufferActive[srcOffset], &frameBuffer[srcOffset], windowWidthBytes);
  }
#endif

  if (Serial) Serial.printf("[%lu]   Window display complete\n", millis());
}

void EInkDisplay::displayGrayBuffer(const bool turnOffScreen) {
  while (!pollRefreshComplete()) {
    delay(1);
  }

  drawGrayscale = false;
  inGrayscaleMode = true;

  // activate the custom LUT for grayscale rendering and refresh
  setCustomLUT(true, lut_grayscale);
  refreshDisplay(FAST_REFRESH, turnOffScreen);
  setCustomLUT(false);
}

bool EInkDisplay::isBusy() const {
  return digitalRead(_busy) == HIGH;
}

bool EInkDisplay::pollRefreshComplete() {
  if (!refreshInProgress) {
    return true;
  }

  if (isBusy()) {
    return false;
  }

  refreshInProgress = false;
  return true;
}

void EInkDisplay::refreshDisplay(const RefreshMode mode, const bool turnOffScreen) {
  const char* refreshType = (mode == FULL_REFRESH) ? "full" : (mode == HALF_REFRESH) ? "half" : "fast";

  if (!refreshDisplayAsync(mode, turnOffScreen)) {
    // Preserve blocking behavior: wait for ongoing refresh and retry once.
    waitWhileBusy(" pre-refresh");
    refreshInProgress = false;
    if (!refreshDisplayAsync(mode, turnOffScreen)) {
      if (Serial) Serial.printf("[%lu]   ERROR: Failed to start refresh (%s)\n", millis(), refreshType);
      return;
    }
  }

  // Wait for display to finish updating
  if (Serial) Serial.printf("[%lu]   Waiting for display refresh...\n", millis());
  waitWhileBusy(refreshType);
  refreshInProgress = false;
}

bool EInkDisplay::refreshDisplayAsync(const RefreshMode mode, const bool turnOffScreen) {
  if (!pollRefreshComplete()) {
    return false;
  }
  if (isBusy()) {
    return false;
  }

  // Configure Display Update Control 1
  sendCommandData(CMD_DISPLAY_UPDATE_CTRL1, (mode == FAST_REFRESH) ? CTRL1_NORMAL : CTRL1_BYPASS_RED);

  // best guess at display mode bits:
  // bit | hex | name                    | effect
  // ----+-----+--------------------------+-------------------------------------------
  // 7   | 80  | CLOCK_ON                | Start internal oscillator
  // 6   | 40  | ANALOG_ON               | Enable analog power rails (VGH/VGL drivers)
  // 5   | 20  | TEMP_LOAD               | Load temperature (internal or I2C)
  // 4   | 10  | LUT_LOAD                | Load waveform LUT
  // 3   | 08  | MODE_SELECT             | Mode 1/2
  // 2   | 04  | DISPLAY_START           | Run display
  // 1   | 02  | ANALOG_OFF_PHASE        | Shutdown step 1 (undocumented)
  // 0   | 01  | CLOCK_OFF               | Disable internal oscillator

  // Select appropriate display mode based on refresh type
  uint8_t displayMode = 0x00;

  // Enable counter and analog if not already on
  if (!isScreenOn) {
    isScreenOn = true;
    displayMode |= 0xC0;  // Set CLOCK_ON and ANALOG_ON bits
  }

  // Turn off screen if requested
  if (turnOffScreen) {
    isScreenOn = false;
    displayMode |= 0x03;  // Set ANALOG_OFF_PHASE and CLOCK_OFF bits
  }

  if (mode == FULL_REFRESH) {
    displayMode |= 0x34;
  } else if (mode == HALF_REFRESH) {
    // Write high temp to the register for a faster refresh
    sendCommandData(CMD_WRITE_TEMP, 0x5A);
    displayMode |= 0xD4;
  } else {  // FAST_REFRESH
    displayMode |= customLutActive ? 0x0C : 0x1C;
  }

  // Power on and refresh display
  const char* refreshType = (mode == FULL_REFRESH) ? "full" : (mode == HALF_REFRESH) ? "half" : "fast";
  if (Serial) Serial.printf("[%lu]   Powering on display 0x%02X (%s refresh)...\n", millis(), displayMode, refreshType);
  sendCommandData(CMD_DISPLAY_UPDATE_CTRL2, displayMode);

  sendCommand(CMD_MASTER_ACTIVATION);
  refreshInProgress = true;
  return true;
}

void EInkDisplay::setCustomLUT(const bool enabled, const unsigned char* lutData) {
  if (enabled) {
    if (Serial) Serial.printf("[%lu]   Loading custom LUT...\n", millis());

    uint8_t lutWaveform[105];
    for (uint16_t i = 0; i < sizeof(lutWaveform); i++) {
      lutWaveform[i] = pgm_read_byte(&lutData[i]);
    }

    // Load custom LUT (first 105 bytes: VS + TP/RP + frame rate)
    sendCommandData(CMD_WRITE_LUT, lutWaveform, sizeof(lutWaveform));

    // Set voltage values from bytes 105-109
    sendCommandData(CMD_GATE_VOLTAGE, pgm_read_byte(&lutData[105]));  // VGH

    const uint8_t sourceVoltages[] = {
        pgm_read_byte(&lutData[106]), pgm_read_byte(&lutData[107]), pgm_read_byte(&lutData[108])};
    sendCommandData(CMD_SOURCE_VOLTAGE, sourceVoltages, sizeof(sourceVoltages));  // VSH1, VSH2, VSL

    sendCommandData(CMD_WRITE_VCOM, pgm_read_byte(&lutData[109]));  // VCOM

    customLutActive = true;
    if (Serial) Serial.printf("[%lu]   Custom LUT loaded\n", millis());
  } else {
    customLutActive = false;
    if (Serial) Serial.printf("[%lu]   Custom LUT disabled\n", millis());
  }
}

void EInkDisplay::deepSleep() {
  while (!pollRefreshComplete()) {
    delay(1);
  }

  if (Serial) Serial.printf("[%lu]   Preparing display for deep sleep...\n", millis());

  // First, power down the display properly
  // This shuts down the analog power rails and clock
  if (isScreenOn) {
    sendCommandData(CMD_DISPLAY_UPDATE_CTRL1, CTRL1_BYPASS_RED);  // Normal mode
    sendCommandData(CMD_DISPLAY_UPDATE_CTRL2, 0x03);              // ANALOG_OFF_PHASE and CLOCK_OFF

    sendCommand(CMD_MASTER_ACTIVATION);

    // Wait for the power-down sequence to complete
    waitWhileBusy(" display power-down");

    isScreenOn = false;
  }

  // Now enter deep sleep mode
  if (Serial) Serial.printf("[%lu]   Entering deep sleep mode...\n", millis());
  sendCommandData(CMD_DEEP_SLEEP, 0x01);  // Enter deep sleep
  ramDataEntryConfigured = false;
  ramAreaConfigured = false;
}

void EInkDisplay::saveFrameBufferAsPBM(const char* filename) {
#ifndef ARDUINO
  const uint8_t* buffer = getFrameBuffer();

  std::ofstream file(filename, std::ios::binary);
  if (!file) {
    if (Serial) Serial.printf("Failed to open %s for writing\n", filename);
    return;
  }

  // Rotate the image 90 degrees counterclockwise when saving
  // Original buffer: 800x480 (landscape)
  // Output image: 480x800 (portrait)
  const int DISPLAY_WIDTH_LOCAL = DISPLAY_WIDTH;    // 800
  const int DISPLAY_HEIGHT_LOCAL = DISPLAY_HEIGHT;  // 480
  const int DISPLAY_WIDTH_BYTES_LOCAL = DISPLAY_WIDTH_LOCAL / 8;

  file << "P4\n";  // Binary PBM
  file << DISPLAY_HEIGHT_LOCAL << " " << DISPLAY_WIDTH_LOCAL << "\n";

  // Create rotated buffer
  std::vector<uint8_t> rotatedBuffer((DISPLAY_HEIGHT_LOCAL / 8) * DISPLAY_WIDTH_LOCAL, 0);

  for (int outY = 0; outY < DISPLAY_WIDTH_LOCAL; outY++) {
    for (int outX = 0; outX < DISPLAY_HEIGHT_LOCAL; outX++) {
      int inX = outY;
      int inY = DISPLAY_HEIGHT_LOCAL - 1 - outX;

      int inByteIndex = inY * DISPLAY_WIDTH_BYTES_LOCAL + (inX / 8);
      int inBitPosition = 7 - (inX % 8);
      bool isWhite = (buffer[inByteIndex] >> inBitPosition) & 1;

      int outByteIndex = outY * (DISPLAY_HEIGHT_LOCAL / 8) + (outX / 8);
      int outBitPosition = 7 - (outX % 8);
      if (!isWhite) {  // Invert: e-ink white=1 -> PBM black=1
        rotatedBuffer[outByteIndex] |= (1 << outBitPosition);
      }
    }
  }

  file.write(reinterpret_cast<const char*>(rotatedBuffer.data()), rotatedBuffer.size());
  file.close();
  if (Serial) Serial.printf("Saved framebuffer to %s\n", filename);
#else
  (void)filename;
  if (Serial) Serial.println("saveFrameBufferAsPBM is not supported on Arduino builds.");
#endif
}
