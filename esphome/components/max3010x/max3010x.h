#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace max3010x {

/// This class implements support for the MAX3010x heart rate and oximeter sensor.
class MAX3010xComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void set_interrupt_pin(binary_sensor::BinarySensor *interrupt_pin) { interrupt_pin_ = interrupt_pin; }
  void set_heart_rate_sensor(sensor::Sensor *heart_rate_sensor) { heart_rate_sensor_ = heart_rate_sensor; }
  void set_oximeter_sensor(sensor::Sensor *oximeter_sensor) { oximeter_sensor_ = oximeter_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }

  // ========== INTERNAL METHODS ==========
  void setup() override;
  void dump_config() override;
  void update() override;

 protected:
  void setup_sensor(uint8_t powerLevel = 0x1F, uint8_t sampleAverage = 4, uint8_t ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096);

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
  uint16_t check();       // Checks for new data and fills FIFO
  uint8_t available(void);    // Tells caller how many new samples are available (head - tail)
  void nextSample(void);      // Advances the tail of the sense array
  uint32_t getFIFORed(void);  // Returns the FIFO sample pointed to by tail
  uint32_t getFIFOIR(void);   // Returns the FIFO sample pointed to by tail

  uint8_t getWritePointer(void);
  uint8_t getReadPointer(void);
  void clearFIFO(void);  // Sets the read/write pointers to zero

  // Die Temperature
  float readTemperature();

  // Detecting ID/Revision
  uint8_t getRevisionID();
  uint8_t readPartID();

  // Low-level I2C communication
  uint8_t readRegister8(uint8_t reg);
  void writeRegister8(uint8_t reg, uint8_t value);

private:
  // activeLEDs is the number of channels turned on, and can be 1 to 3. 2 is common for Red+IR.
  uint8_t activeLEDs;  // Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

  uint8_t revisionID;

  uint8_t readRevisionID();

  void readModifyWrite(uint8_t reg, uint8_t mask, uint8_t thing);

  uint8_t fifo[192];

#define STORAGE_SIZE 4  // Each long is 4 uint8_ts so limit this to fit on your micro
  typedef struct Record {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
  } sense_struct;  // This is our circular buffer of readings from the sensor

  sense_struct sense;

protected:
  bool checkForBeat(uint32_t sample);
  int16_t averageDCEstimator(int32_t *p, uint16_t x);
  int16_t lowPassFIRFilter(int16_t din);

  uint8_t read_u8_(uint8_t a_register);
  uint16_t read_u16_le_(uint8_t a_register);
  int16_t read_s16_le_(uint8_t a_register);

  binary_sensor::BinarySensor *interrupt_pin_{nullptr};
  sensor::Sensor *heart_rate_sensor_{nullptr};
  sensor::Sensor *oximeter_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    WRONG_CHIP_ID,
  } error_code_{NONE};

  int16_t IR_AC_Max = 20;
  int16_t IR_AC_Min = -20;

  int16_t IR_AC_Signal_Current = 0;
  int16_t IR_AC_Signal_Previous;
  int16_t IR_AC_Signal_min = 0;
  int16_t IR_AC_Signal_max = 0;
  int16_t IR_Average_Estimated;

  int16_t positiveEdge = 0;
  int16_t negativeEdge = 0;
  int32_t ir_avg_reg = 0;

  int16_t cbuf[32];
  uint8_t offset = 0;

  long lastBeat = 0;
  float beatsPerMinute = 0.0;

};

}  // namespace max3010x
}  // namespace esphome
