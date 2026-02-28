/**
 * @file ResenseHEX.cpp
 * @brief Implementation of the Resense HEX32 library
 */

#include "ResenseHEX.h"

// Define static member variables
float ResenseHEX::_forceMax = 5000.0f;
float ResenseHEX::_torqueMax = 10.0f;
float ResenseHEX::_tempMax = 150.0f;
uint16_t ResenseHEX::_readTimeoutMs = 300;
uint16_t ResenseHEX::_tareTimeoutMs = 20000;

// Constructor - initialize with a serial stream reference
ResenseHEX::ResenseHEX(Stream &serial)
  : _serial(serial) {}

// Complete measurement: trigger + read + validate
bool ResenseHEX::triggerAndRead(HexFrame &frame) {
  _flushInput();                        // flush UART input
  softwareTrigger();                    // trigger measurement
  frame.timestamp = _getTime();         // save timestamp
  if (!readFrame(frame)) return false;  // read data and return false if incorrect

  return true;
}

// Read a 28-byte frame into HexFrame struct without modifying timestamp
bool ResenseHEX::readFrame(HexFrame &frame) {
  uint8_t frameData[FRAME_DATA_SIZE];
  if (!_readFrameData(frameData)) return false;

  if (!_copyFrameDataToHexFrame(frame, frameData)) return false;

  return true;
}

// Read a 28-byte frame into HexFrame struct and adds timestamp
bool ResenseHEX::readFrameAndTimestamp(HexFrame &frame) {
  uint8_t frameData[FRAME_DATA_SIZE];

  if (!_readFrameData(frameData)) return false;  // in case of timeout or partial frameData
  frame.timestamp = _getTime();

  if (!_copyFrameDataToHexFrame(frame, frameData)) return false;  // in case of corruption

  return true;
}

// Gets a uint8_t frameData of FRAME_DATA_SIZE and fills it with the available serial data
bool ResenseHEX::_readFrameData(uint8_t *frameData) {
  uint8_t idx = 0;
  unsigned long start = _getTime();

  while (idx < FRAME_DATA_SIZE && (_getTime() - start) < _readTimeoutMs) {
    if (_serial.available()) {
      frameData[idx++] = _serial.read();
    }
  }
  if (idx != FRAME_DATA_SIZE) return false;  // timeout or partial frame

  return true;
}

// Copies frameData into a HexFrame and checks for corruption
bool ResenseHEX::_copyFrameDataToHexFrame(HexFrame &frame, uint8_t *frameData) {
  memcpy(&frame.fx, &frameData[0], 4);
  memcpy(&frame.fy, &frameData[4], 4);
  memcpy(&frame.fz, &frameData[8], 4);
  memcpy(&frame.mx, &frameData[12], 4);
  memcpy(&frame.my, &frameData[16], 4);
  memcpy(&frame.mz, &frameData[20], 4);
  memcpy(&frame.temperature, &frameData[24], 4);

  if (!_validateFrameCorruption(frame)) return false;

  return true;
}

// Checks for corrupted HexFrame without checking limits
bool ResenseHEX::_validateFrameCorruption(const HexFrame &frame) const {
  // Check for NaN or extreme values
  if (isnan(frame.fx) || isinf(frame.fx)) return false;
  if (isnan(frame.fy) || isinf(frame.fy)) return false;
  if (isnan(frame.fz) || isinf(frame.fz)) return false;
  if (isnan(frame.mx) || isinf(frame.mx)) return false;
  if (isnan(frame.my) || isinf(frame.my)) return false;
  if (isnan(frame.mz) || isinf(frame.mz)) return false;
  if (isnan(frame.temperature) || isinf(frame.temperature)) return false;

  return true;
}

// Compares Frame values with set user limits
bool ResenseHEX::validateLimits(const HexFrame &frame) const {
  // Check for NaN or extreme values
  if (fabsf(frame.fx) > _forceMax) return false;
  if (fabsf(frame.fy) > _forceMax) return false;
  if (fabsf(frame.fz) > _forceMax) return false;
  if (fabsf(frame.mx) > _torqueMax) return false;
  if (fabsf(frame.my) > _torqueMax) return false;
  if (fabsf(frame.mz) > _torqueMax) return false;
  if (fabsf(frame.temperature) > _tempMax) return false;

  return true;
}

// Flush all pending bytes from the UART receive buffer
void ResenseHEX::_flushInput() {
  while (_serial.available() > 0) {
    _serial.read();  // discard byte
  }
}

// Send pre-compiled software trigger over UART
void ResenseHEX::softwareTrigger() {
  _serial.write(SOFTWARE_TRIGGER_CMD, SOFTWARE_TRIGGER_LEN);
  _serial.flush();  // Ensure it's sent immediately
}

// Send pre-compiled tare command over UART
void ResenseHEX::tare() {
  _serial.write(TARE_CMD, TARE_LEN);
  _serial.flush();  // Ensure it's sent immediately
}

// Tare the system and block until completed
bool ResenseHEX::tareBlocking() {
  tare();
  uint8_t frameData[FRAME_DATA_SIZE];
  unsigned long start = _getTime();
  int nrReads = 0;

  // cycle through Frame reads until MIN_TARE_READS is reached or timeout
  while (nrReads < MIN_TARE_READS) {
    if ((_getTime() - start) >= _tareTimeoutMs) {
      return false; // timeout exceeded
    }
    softwareTrigger();
    delay(10);
    if (_readFrameData(frameData)) {  // read to clear buffer
      nrReads++;
    } else {
      return false;  // corruption or timeout
    }
  }
  return true;
}

/**
 * @brief Send command over UART dynamically
 */
void ResenseHEX::sendCommand(const char *cmd) {
  _serial.write(cmd, strlen(cmd));
  _serial.flush();  // Ensure it's sent immediately
}

// Set force threshold
void ResenseHEX::setMaxForce(float forceMax) {
  _forceMax = forceMax;
}

// Set torque threshold
void ResenseHEX::setMaxTorque(float torqueMax) {
  _torqueMax = torqueMax;
}

// Set temperature threshold
void ResenseHEX::setMaxTemperature(float tempMax) {
  _tempMax = tempMax;
}

// Set trigger timeout in milliseconds
void ResenseHEX::setReadTimeout(uint16_t timeoutMs) {
  _readTimeoutMs = timeoutMs;
}

// set timeout for taring
void ResenseHEX::setTareTimeout(uint16_t timeoutMs) {
  _tareTimeoutMs = timeoutMs;
}

// Get force threshold
float ResenseHEX::getMaxForce() const {
  return _forceMax;
}

// Get torque threshold
float ResenseHEX::getMaxTorque() const {
  return _torqueMax;
}

// Get temperature threshold
float ResenseHEX::getMaxTemperature() const {
  return _tempMax;
}

// Get trigger timeout
uint16_t ResenseHEX::getReadTimeout() const {
  return _readTimeoutMs;
}

// Get current time in ms
unsigned long ResenseHEX::_getTime() {
  return millis();
}

// get timeout for taring
uint16_t ResenseHEX::getTareTimeout() const {
  return _tareTimeoutMs;
}
