#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace max3010x {

static const char *const TAG = "max3010x.sensor";

/// This class implements support for the MAX3010x heart rate and oximeter sensor.
class MAX3010xComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_heart_rate_sensor(sensor::Sensor *heart_rate_sensor) { heart_rate_sensor_ = heart_rate_sensor; }
  void set_oximeter_sensor(sensor::Sensor *oximeter_sensor) { oximeter_sensor_ = oximeter_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void setup_sensor(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange);

  uint32_t getRed(void);                   // Returns immediate red value
  uint32_t getIR(void);                    // Returns immediate IR value
  bool safeCheck(uint8_t maxTimeToCheck);  // Given a max amount of time, check for new data

  // Configuration
  void softReset();
  void shutDown();
  void wakeUp();

  void setLEDMode(uint8_t mode);

  void setADCRange(uint8_t adcRange);
  void setSampleRate(uint8_t sampleRate);
  void setPulseWidth(uint8_t pulseWidth);

  void setPulseAmplitudeRed(uint8_t value);
  void setPulseAmplitudeIR(uint8_t value);

  // Multi-led configuration mode (page 22)
  void enableSlot(uint8_t slotNumber, uint8_t device);  // Given slot number, assign a device to slot
  void disableSlots(void);

  // Data Collection

  // Interrupts (page 13, 14)
  uint8_t getINT1(void);   // Returns the main interrupt group
  uint8_t getINT2(void);   // Returns the temp ready interrupt
  void enableAFULL(void);  // Enable/disable individual interrupts
  void disableAFULL(void);
  void enableDATARDY(void);
  void disableDATARDY(void);
  void enableALCOVF(void);
  void disableALCOVF(void);
  void enablePROXINT(void);
  void disablePROXINT(void);
  void enableDIETEMPRDY(void);
  void disableDIETEMPRDY(void);

  // FIFO Configuration (page 18)
  void setFIFOAverage(uint8_t samples);
  void enableFIFORollover();
  void disableFIFORollover();
  void setFIFOAlmostFull(uint8_t samples);

  // FIFO Reading
  uint16_t check(void);         // Checks for new data and fills FIFO
  uint8_t available(void);      // Tells caller how many new samples are available (head - tail)
  void nextSample(void);        // Advances the tail of the sense array
  uint32_t getFIFORed(void);    // Returns the FIFO sample pointed to by tail
  uint32_t getFIFOIR(void);     // Returns the FIFO sample pointed to by tail

  uint8_t getWritePointer(void);
  uint8_t getReadPointer(void);
  void clearFIFO(void);  // Sets the read/write pointers to zero

  // Die Temperature
  float readTemperature();

  // Detecting ID/Revision
  uint8_t getRevisionID();
  uint8_t readPartID();

  // Setup the IC with user selectable settings
  void setup(byte powerLevel = 0x1F, byte sampleAverage = 4, byte ledMode = 3, int sampleRate = 400,
             int pulseWidth = 411, int adcRange = 4096);

  // Low-level I2C communication
  uint8_t readRegister8(uint8_t address, uint8_t reg);
  void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);

 private:
  TwoWire *_i2cPort;  // The generic connection to user's chosen I2C hardware
  uint8_t _i2caddr;

  // activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  byte activeLEDs;  // Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

  uint8_t revisionID;

  void readRevisionID();

  void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);

#define STORAGE_SIZE 4  // Each long is 4 bytes so limit this to fit on your micro
  typedef struct Record {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    byte head;
    byte tail;
  } sense_struct;  // This is our circular buffer of readings from the sensor

  sense_struct sense;

 protected:
  bool checkForBeat(int32_t sample);
  int16_t averageDCEstimator(int32_t *p, uint16_t x);
  int16_t lowPassFIRFilter(int16_t din);
  int32_t mul16(int16_t x, int16_t y);

  // Read the heart rate value
  float read_heart_rate_(const uint8_t *data);
  // Read the O2 saturation value
  float read_oximeter_(const uint8_t *data);
  uint8_t read_u8_(uint8_t a_register);
  uint16_t read_u16_le_(uint8_t a_register);
  int16_t read_s16_le_(uint8_t a_register);

  virtual bool read_byte(uint8_t a_register, uint8_t *data) = 0;
  virtual bool write_byte(uint8_t a_register, uint8_t data) = 0;
  virtual bool read_bytes(uint8_t a_register, uint8_t *data, size_t len) = 0;
  virtual bool read_byte_16(uint8_t a_register, uint16_t *data) = 0;

  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *oximeter_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    WRONG_CHIP_ID,
  } error_code_{NONE};

  bool read_byte(uint8_t a_register, uint8_t *data) override;
  bool write_byte(uint8_t a_register, uint8_t data) override;
  bool read_bytes(uint8_t a_register, uint8_t *data, size_t len) override;
  bool read_byte_16(uint8_t a_register, uint16_t *data) override;
};

}  // namespace max3010x
}  // namespace esphome
