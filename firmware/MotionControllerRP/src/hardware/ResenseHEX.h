/**
 * @file ResenseHEX.h
 * @brief Arduino library for Resense HEX32 6-axis Force/Torque Sensor
 * 
 * This library provides a clean interface to read force and torque data from
 * the Resense HEX32 sensor over UART. The library is MCU-agnostic and works
 * with any Arduino-compatible board (ESP32, ESP32-S3, Arduino, STM32, etc.)
 * as long as the target provides HardwareSerial compatibility.
 * 
 * @author Mark Suppelt
 * @version 1.0.1
 * @date 2025-12-09
 * 
 * Connection (HEX32 UART pin header):
 *   Pin 1: VDD (5V) 
 *   Pin 2: MCU TX (3.3V)
 *   Pin 3: MCU RX (3.3V)
 *   Pin 4: Trigger (optional)
 *   Pin 5: GND
 * 
 * Hardware Trigger (optional):
 *   MCU GPIO (any pin) -> HEX32 trigger input (3.3V or 5V pulse)
 * 
 * UART Configuration:
 *   Baud Rate: 2,000,000 bit/s (default, configurable)
 *   Data Bits: 8
 *   Parity: None
 *   Stop Bits: 1
 * 
 * Data Frame (28 bytes total):
 *   Bytes 0-3:   fx (float32)
 *   Bytes 4-7:   fy (float32)
 *   Bytes 8-11:  fz (float32)
 *   Bytes 12-15: mx (float32)
 *   Bytes 16-19: my (float32)
 *   Bytes 20-23: mz (float32)
 *   Bytes 24-27: temperature (float32)
 */

#ifndef RESENSE_HEX_H
#define RESENSE_HEX_H

#include <Arduino.h>
#include <Stream.h>  // Explicit (sometimes needed on some platforms)
#include <cstring>

/**
 * @struct HexFrame
 * @brief Data structure holding one complete measurement frame from HEX
 */
struct HexFrame {
  float fx;                 ///< Force in X direction (N or raw)
  float fy;                 ///< Force in Y direction (N or raw)
  float fz;                 ///< Force in Z direction (N or raw)
  float mx;                 ///< Torque around X axis (mNm or raw)
  float my;                 ///< Torque around Y axis (mNm or raw)
  float mz;                 ///< Torque around Z axis (mNm or raw)
  float temperature;        ///< Sensor temperature (°C)
  unsigned long timestamp;  ///< timestamp of measurement (millis)
};

/**
 * @class ResenseHEX
 * @brief UART interface for Resense HEX 6-axis F/T sensors
 */
class ResenseHEX {
public:
  static constexpr unsigned long DEFAULT_BAUD = 2000000UL;  ///< HEX default: 2 Mbaud
  static constexpr uint32_t DEFAULT_CONFIG = SERIAL_8N1;    ///< HEX default: 8-N-1

  /**
   * @brief Constructor
   * @param serial Reference to a stream object (HardwareSerial, SoftwareSerial, etc.)
   */
  explicit ResenseHEX(Stream &serial);

  /**
   * @brief Perform complete measurement cycle: trigger + read + validate
   *
   * Sends software trigger command, captures timestamp immediately, then reads
   * and validates one complete frame. Combines softwareTrigger() + readFrame()
   * into single atomic operation with built-in timeout.
   *
   * @param[out] frame HexFrame struct to populate with validated data
   * @return true if trigger sent and valid frame received, false on timeout/invalid
   */
  bool triggerAndRead(HexFrame &frame);

  /**
   * @brief Attempt to read one complete frame from the sensor and converts it to a HexFrame struct
   * 
   * This function accumulates bytes from the serial stream and returns true
   * when a complete 28-byte frame has been received and decoded.
   * 
   * @param[out] frame Reference to HexFrame struct to populate
   * @return true if a complete frame was successfully read, false otherwise
   */
  bool readFrame(HexFrame &frame);

  /**
   * @brief Attempt to read one complete frame from the sensor and converts it to a HexFrame struct with timestamp
   * 
   * same as read Frame but additionally timestamps the HexFrame
   * 
   * @param[out] frame Reference to HexFrame struct to populate
   * @return true if a complete frame was successfully read, false otherwise
   */
  bool readFrameAndTimestamp(HexFrame &frame);

  /**
 * @brief Send pre-compiled software trigger over UART
 */
  void softwareTrigger();

  /**
 * @brief Send pre-compiled tare command over UART
 */
  void tare();

  /**
   * @brief tare the system and block until completed
   *
   * @return true if tare has been completed successfully, false otherwise
   */
  bool tareBlocking();

  /**
  * @brief Send command over UART dynamically
  */
  void sendCommand(const char *cmd);

  /**
   * @brief Set maximum force validation threshold
   * 
   * @param forceMax Maximum absolute force in Newtons
   */
  void setMaxForce(float forceMax);

  /**
   * @brief Set maximum torque validation threshold
   * 
   * @param torqueMax Maximum absolute torque in mNm
   */
  void setMaxTorque(float torqueMax);

  /**
   * @brief Set maximum temperature validation threshold
   * 
   * @param tempMax Maximum absolute temperature in °C
   */
  void setMaxTemperature(float tempMax);

  /**
   * @brief Set software trigger response timeout
   * 
   * @param timeoutMs Timeout in milliseconds
   */
  void setReadTimeout(uint16_t timeoutMs);

  /**
   * @brief Get current force validation threshold
   * @return Force threshold in Newtons
   */
  float getMaxForce() const;

  /**
   * @brief Get current torque validation threshold
   * @return Torque threshold in mNm
   */
  float getMaxTorque() const;

  /**
   * @brief Get current temperature validation threshold
   * @return Temperature threshold in °C
   */
  float getMaxTemperature() const;

  /**
   * @brief Get current software trigger timeout
   * @return Timeout in milliseconds
   */
  uint16_t getReadTimeout() const;

  /**
   * @brief checks HexFrame against user set limits
   * 
   * @param frame The frame to validate
   * @return true if all values appear reasonable
   */
  bool validateLimits(const HexFrame &frame) const;

private:
  static constexpr size_t FRAME_DATA_SIZE = 28;              //< HEX frameData size
  static constexpr size_t MIN_TARE_READS = 1000;        //< min number of softwaretrigger reads have to be accomplished before system is tared

  static constexpr const char *SOFTWARE_TRIGGER_CMD = "SAMPLE\r\n";  //< ASCII for software trigger
  static constexpr const char *TARE_CMD = "TARA\r\n";                //< ASCII for TARA/taring

  // precalculate command lengths at compile time
  static constexpr size_t SOFTWARE_TRIGGER_LEN = strlen(SOFTWARE_TRIGGER_CMD);
  static constexpr size_t TARE_LEN = strlen(TARE_CMD);

  // Limits for validation purposes (absolute)
  static float _forceMax;
  static float _torqueMax;
  static float _tempMax;
  static uint16_t _readTimeoutMs;  //< Default timeout ms
  static uint16_t _tareTimeoutMs;  ///< Current tare timeout in ms

  Stream &_serial;  //< Reference to serial stream

  /**
   * @brief Perform basic sanity checks on float values WITHOUT considering user set limits
   * 
   * @param frame The frame to validate
   * @return true if all values appear reasonable
   */
  bool _validateFrameCorruption(const HexFrame &frame) const;

  /**
   * @brief Flush all pending bytes from the UART receive buffer
   *
   * Non-blocking: reads and discards all available bytes from the attached
   * Stream so the next frame starts cleanly at a byte boundary.
   */
  void _flushInput();

  /**
   * @brief gets the current timestamp
   *
   */
  unsigned long _getTime();

    /**
   * @brief Set tare timeout in milliseconds
   */
  void setTareTimeout(uint16_t timeoutMs);

  /**
   * @brief Get current tare timeout in milliseconds
   */
  uint16_t getTareTimeout() const;

  /**
   * @brief Read one raw 28-byte frame into a user buffer
   *
   * This function fills the internal buffer from the serial stream and, when
   * at least FRAME_DATA_SIZE bytes are available, copies them verbatim into the
   * provided buffer (no decoding or validation).
   *
   * @param[out] frameData User-provided buffer of size >= FRAME_DATA_SIZE
   * @return true if 28 bytes were copied into frameData, false otherwise
   */
  bool _readFrameData(uint8_t *frameData);

  /**
 * @brief copies frameData into a HexFrame and checks for corruption
 */
  bool _copyFrameDataToHexFrame(HexFrame &frame, uint8_t *frameData);
};

#endif