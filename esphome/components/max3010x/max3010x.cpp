#include <cmath>
#include <cstddef>
#include <cstdint>

#include "max3010x.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <esphome/components/sensor/sensor.h>
#include "esphome/components/i2c/i2c.h"
#include <esphome/core/component.h>

namespace esphome {
namespace max3010x {

static const char *const TAG = "max3010x";

static const uint8_t MAX3010X_REGISTER_ISR1 = 0x00;
static const uint8_t MAX3010X_REGISTER_ISR2 = 0x01;
static const uint8_t MAX3010X_REGISTER_INTEN1 = 0x02;
static const uint8_t MAX3010X_REGISTER_INTEN2 = 0x03;

static const uint8_t MAX3010X_REGISTER_FIFO_WR_PTR = 0x04;
static const uint8_t MAX3010X_REGISTER_OVF_COUNTER = 0x05;
static const uint8_t MAX3010X_REGISTER_FIFO_RD_PTR = 0x06;
static const uint8_t MAX3010X_REGISTER_FIFO_DATA = 0x07;

static const uint8_t MAX3010X_REGISTER_FIFO_CFG = 0x08;
static const uint8_t MAX3010X_REGISTER_MODE_CFG = 0x09;
static const uint8_t MAX3010X_REGISTER_SPO2_CFG = 0x0A;
static const uint8_t MAX3010X_REGISTER_LED_PULSE_AMPL1_CFG = 0x0C;
static const uint8_t MAX3010X_REGISTER_LED_PULSE_AMPL2_CFG = 0x0D;
static const uint8_t MAX3010X_REGISTER_MULTILED_MODE_CTRL1 = 0x11;
static const uint8_t MAX3010X_REGISTER_MULTILED_MODE_CTRL2 = 0x12;

static const uint8_t MAX3010X_REGISTER_DIE_TEMP_INT = 0x1F;
static const uint8_t MAX3010X_REGISTER_DIE_TEMP_FRAC = 0x20;
static const uint8_t MAX3010X_REGISTER_DIE_TEMP_CFG = 0x21;

static const uint8_t MAX3010X_REGISTER_REV_ID = 0xFE;
static const uint8_t MAX3010X_REGISTER_PART_ID = 0xFF;

static const uint8_t MAX3010X_PART_ID = 0x15;

// MAX3010X Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX3010X_INT_A_FULL_MASK = (uint8_t) ~0b10000000;
static const uint8_t MAX3010X_INT_A_FULL_ENABLE = 0x80;
static const uint8_t MAX3010X_INT_A_FULL_DISABLE = 0x00;

static const uint8_t MAX3010X_INT_DATA_RDY_MASK = (uint8_t) ~0b01000000;
static const uint8_t MAX3010X_INT_DATA_RDY_ENABLE = 0x40;
static const uint8_t MAX3010X_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX3010X_INT_ALC_OVF_MASK = (uint8_t) ~0b00100000;
static const uint8_t MAX3010X_INT_ALC_OVF_ENABLE = 0x20;
static const uint8_t MAX3010X_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_MASK = (uint8_t) ~0b00000010;
static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX3010X_SAMPLEAVG_MASK = (uint8_t) ~0b11100000;
static const uint8_t MAX3010X_SAMPLEAVG_1 = 0x00;
static const uint8_t MAX3010X_SAMPLEAVG_2 = 0x20;
static const uint8_t MAX3010X_SAMPLEAVG_4 = 0x40;
static const uint8_t MAX3010X_SAMPLEAVG_8 = 0x60;
static const uint8_t MAX3010X_SAMPLEAVG_16 = 0x80;
static const uint8_t MAX3010X_SAMPLEAVG_32 = 0xA0;

static const uint8_t MAX3010X_ROLLOVER_MASK = 0xEF;
static const uint8_t MAX3010X_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX3010X_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX3010X_A_FULL_MASK = 0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX3010X_SHUTDOWN_MASK = 0x7F;
static const uint8_t MAX3010X_SHUTDOWN = 0x80;
static const uint8_t MAX3010X_WAKEUP = 0x00;

static const uint8_t MAX3010X_RESET_MASK = 0xBF;
static const uint8_t MAX3010X_RESET = 0x40;

static const uint8_t MAX3010X_MODE_MASK = 0xF8;
static const uint8_t MAX3010X_MODE_REDONLY = 0x02;
static const uint8_t MAX3010X_MODE_REDIRONLY = 0x03;
static const uint8_t MAX3010X_MODE_MULTILED = 0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX3010X_ADCRANGE_MASK = 0x9F;
static const uint8_t MAX3010X_ADCRANGE_2048 = 0x00;
static const uint8_t MAX3010X_ADCRANGE_4096 = 0x20;
static const uint8_t MAX3010X_ADCRANGE_8192 = 0x40;
static const uint8_t MAX3010X_ADCRANGE_16384 = 0x60;

static const uint8_t MAX3010X_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX3010X_SAMPLERATE_50 = 0x00;
static const uint8_t MAX3010X_SAMPLERATE_100 = 0x04;
static const uint8_t MAX3010X_SAMPLERATE_200 = 0x08;
static const uint8_t MAX3010X_SAMPLERATE_400 = 0x0C;
static const uint8_t MAX3010X_SAMPLERATE_800 = 0x10;
static const uint8_t MAX3010X_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX3010X_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX3010X_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX3010X_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX3010X_PULSEWIDTH_69 = 0x00;
static const uint8_t MAX3010X_PULSEWIDTH_118 = 0x01;
static const uint8_t MAX3010X_PULSEWIDTH_215 = 0x02;
static const uint8_t MAX3010X_PULSEWIDTH_411 = 0x03;

// Multi-LED Mode configuration (pg 22)
static const uint8_t MAX3010X_SLOT1_MASK = 0xF8;
static const uint8_t MAX3010X_SLOT2_MASK = 0x8F;
static const uint8_t MAX3010X_SLOT3_MASK = 0xF8;
static const uint8_t MAX3010X_SLOT4_MASK = 0x8F;

static const uint8_t SLOT_NONE = 0x00;
static const uint8_t SLOT_RED_LED = 0x01;
static const uint8_t SLOT_IR_LED = 0x02;
static const uint8_t SLOT_NONE_PILOT = 0x04;
static const uint8_t SLOT_RED_PILOT = 0x05;
static const uint8_t SLOT_IR_PILOT = 0x06;

static const uint8_t MAX3010X_ADDRESS = 0x57;  // 7-bit I2C address

#define I2C_SPEED_STANDARD 100000
#define I2C_SPEED_FAST 400000

#define I2C_BUFFER_LENGTH 192

// Mostly based on https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/tree/master/src

void MAX3010xComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MAX3010x...");

  // Mark as not failed before initializing. Some devices will turn off sensors to save on batteries
  // and when they come back on, the COMPONENT_STATE_FAILED bit must be unset on the component.
  if ((this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_FAILED) {
    this->component_state_ &= ~COMPONENT_STATE_MASK;
    this->component_state_ |= COMPONENT_STATE_CONSTRUCTION;
  }

  // Step 1: Initial Communication and Verification
  // Check that a MAX3010X is connected
  uint8_t partId = readPartID();
  if (partId != MAX3010X_PART_ID) {
    // Error -- Part ID read from MAX3010X does not match expected part ID.
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
    this->error_code_ = WRONG_CHIP_ID;
    this->status_set_error();
    ESP_LOGE(TAG, "Wrong chip ID read. Expecting 0x%x, but read 0x%x", MAX3010X_PART_ID, partId);
    return;
  }

  // Populate revision ID
  readRevisionID();
  ESP_LOGI(TAG, "MAX3010x revision 0x%x", revisionID);

  //powerLevel = 0x1F, sampleAverage = 4, ledMode = 3, sampleRate = 400, pulseWidth = 411, adcRange = 4096
  setup_sensor(0x1F, 4, 2, 400, 411, 4096);
  setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
}

void MAX3010xComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MAX3010x:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX3010x failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Heart rate", this->heart_rate_sensor_);
  LOG_SENSOR("  ", "Oximeter", this->oximeter_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
}

void MAX3010xComponent::update() {
  // Only start operation after ~5 s in order to ensure the network is connected
  // (and thus we can see logging)
  if (millis() < 5000) {
    return;
  }

  check();

  uint8_t items = available();
  while (items > 0) {
    //uint32_t ir = getIR();
    uint32_t ir = getFIFOIR();
    //uint32_t ir = sense.IR[sense.head];
    if (checkForBeat(ir)) {
      long now = millis();
      long delta = now - lastBeat;
      lastBeat = now;
      beatsPerMinute = 60000.0 / delta;
      ESP_LOGI(TAG, "Beat!  BPM = %.0f", beatsPerMinute);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        this->heart_rate_sensor_->publish_state(beatsPerMinute);
      }
    }
    nextSample();
    items = available();
  }
}

//
// Configuration
//

// Begin Interrupt configuration
uint8_t MAX3010xComponent::getINT1(void) {
  uint8_t data;
  if (read_register(MAX3010X_REGISTER_ISR1, &data, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error reading ISR1");
    return 0xBD;
  }
  return data;
}
uint8_t MAX3010xComponent::getINT2(void) {
  uint8_t data;
  if (read_register(MAX3010X_REGISTER_ISR2, &data, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error reading ISR2");
    return 0xBD;
  }
  return data;
}

void MAX3010xComponent::enableAFULL(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_A_FULL_MASK, MAX3010X_INT_A_FULL_ENABLE);
}
void MAX3010xComponent::disableAFULL(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_A_FULL_MASK, MAX3010X_INT_A_FULL_DISABLE);
}

void MAX3010xComponent::enableDATARDY(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_DATA_RDY_MASK, MAX3010X_INT_DATA_RDY_ENABLE);
}
void MAX3010xComponent::disableDATARDY(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_DATA_RDY_MASK, MAX3010X_INT_DATA_RDY_DISABLE);
}

void MAX3010xComponent::enableALCOVF(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_ALC_OVF_MASK, MAX3010X_INT_ALC_OVF_ENABLE);
}
void MAX3010xComponent::disableALCOVF(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_ALC_OVF_MASK, MAX3010X_INT_ALC_OVF_DISABLE);
}

void MAX3010xComponent::enableDIETEMPRDY(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN2, MAX3010X_INT_DIE_TEMP_RDY_MASK, MAX3010X_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX3010xComponent::disableDIETEMPRDY(void) {
  readModifyWrite(MAX3010X_REGISTER_INTEN2, MAX3010X_INT_DIE_TEMP_RDY_MASK, MAX3010X_INT_DIE_TEMP_RDY_DISABLE);
}

// End Interrupt configuration

void MAX3010xComponent::softReset(void) {
  readModifyWrite(MAX3010X_REGISTER_MODE_CFG, MAX3010X_RESET_MASK, MAX3010X_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
    uint8_t response;
    if (read_register(MAX3010X_REGISTER_MODE_CFG, &response, 1, true) != i2c::ERROR_OK) {
      this->status_set_warning();
      ESP_LOGW(TAG, "Error reading MODE_CFG register");
      return;
    }
    if ((response & MAX3010X_RESET) == 0)
      break;   // We're done!
    delay(1);  // Let's not over burden the I2C bus
  }
}

void MAX3010xComponent::shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  readModifyWrite(MAX3010X_REGISTER_MODE_CFG, MAX3010X_SHUTDOWN_MASK, MAX3010X_SHUTDOWN);
}

void MAX3010xComponent::wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  readModifyWrite(MAX3010X_REGISTER_MODE_CFG, MAX3010X_SHUTDOWN_MASK, MAX3010X_WAKEUP);
}

void MAX3010xComponent::setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  readModifyWrite(MAX3010X_REGISTER_MODE_CFG, MAX3010X_MODE_MASK, mode);
}

void MAX3010xComponent::setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX3010X_ADCRANGE_2048, _4096, _8192, _16384
  readModifyWrite(MAX3010X_REGISTER_SPO2_CFG, MAX3010X_ADCRANGE_MASK, adcRange);
}

void MAX3010xComponent::setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX3010X_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  readModifyWrite(MAX3010X_REGISTER_SPO2_CFG, MAX3010X_SAMPLERATE_MASK, sampleRate);
}

void MAX3010xComponent::setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX3010X_PULSEWIDTH_69, _188, _215, _411
  readModifyWrite(MAX3010X_REGISTER_SPO2_CFG, MAX3010X_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX3010xComponent::setPulseAmplitudeRed(uint8_t amplitude) {
  if (this->write_register(MAX3010X_REGISTER_LED_PULSE_AMPL1_CFG, &amplitude, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error writing LED pulse amplitude 1 register");
    return;
  }
}

void MAX3010xComponent::setPulseAmplitudeIR(uint8_t amplitude) {
  if (this->write_register(MAX3010X_REGISTER_LED_PULSE_AMPL2_CFG, &amplitude, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error writing LED pulse amplitude 2 register");
    return;
  }
}

// Given a slot number assign a thing to it
// Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
// Assigning a SLOT_RED_LED will pulse LED
// Assigning a SLOT_RED_PILOT will ??
void MAX3010xComponent::enableSlot(uint8_t slotNumber, uint8_t device) {
  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      readModifyWrite(MAX3010X_REGISTER_MULTILED_MODE_CTRL1, MAX3010X_SLOT1_MASK, device);
      break;
    case (2):
      readModifyWrite(MAX3010X_REGISTER_MULTILED_MODE_CTRL1, MAX3010X_SLOT2_MASK, device << 4);
      break;
    case (3):
      readModifyWrite(MAX3010X_REGISTER_MULTILED_MODE_CTRL2, MAX3010X_SLOT3_MASK, device);
      break;
    case (4):
      readModifyWrite(MAX3010X_REGISTER_MULTILED_MODE_CTRL2, MAX3010X_SLOT4_MASK, device << 4);
      break;
    default:
      // Shouldn't be here!
      break;
  }
}

// Clears all slot assignments
void MAX3010xComponent::disableSlots(void) {
  uint8_t value = 0;
  if (this->write_register(MAX3010X_REGISTER_MULTILED_MODE_CTRL1, &value, 1, false) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
  if (this->write_register(MAX3010X_REGISTER_MULTILED_MODE_CTRL2, &value, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
}

//
// FIFO Configuration
//

// Set sample average (Table 3, Page 18)
void MAX3010xComponent::setFIFOAverage(uint8_t numberOfSamples) {
  readModifyWrite(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_SAMPLEAVG_MASK, numberOfSamples);
}

// Resets all points to start in a known state
// Page 15 recommends clearing FIFO before beginning a read
void MAX3010xComponent::clearFIFO(void) {
  uint8_t value = 0;
  if (this->write_register(MAX3010X_REGISTER_FIFO_WR_PTR, &value, 1, false) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
  if (this->write_register(MAX3010X_REGISTER_OVF_COUNTER, &value, 1, false) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
  if (this->write_register(MAX3010X_REGISTER_FIFO_RD_PTR, &value, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
}

// Enable roll over if FIFO over flows
void MAX3010xComponent::enableFIFORollover(void) {
  readModifyWrite(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_ROLLOVER_MASK, MAX3010X_ROLLOVER_ENABLE);
}

// Disable roll over if FIFO over flows
void MAX3010xComponent::disableFIFORollover(void) {
  readModifyWrite(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_ROLLOVER_MASK, MAX3010X_ROLLOVER_DISABLE);
}

// Set number of samples to trigger the almost full interrupt (Page 18)
// Power on default is 32 samples
// Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX3010xComponent::setFIFOAlmostFull(uint8_t numberOfSamples) {
  readModifyWrite(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_A_FULL_MASK, numberOfSamples);
}

// Read the FIFO Write Pointer
uint8_t MAX3010xComponent::getWritePointer(void) {
  uint8_t ptr;
  if (this->read_register(MAX3010X_REGISTER_FIFO_WR_PTR, &ptr, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error reading FIFO write pointer");
    return 0xBD;
  }
  return ptr;
}

// Read the FIFO Read Pointer
uint8_t MAX3010xComponent::getReadPointer(void) {
  uint8_t ptr;
  if (this->read_register(MAX3010X_REGISTER_FIFO_RD_PTR, &ptr, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error reading FIFO read pointer");
    return 0xBD;
  }
  return ptr;
}

// Die Temperature
// Returns temp in C
float MAX3010xComponent::readTemperature() {
  // DIE_TEMP_RDY interrupt must be enabled
  // See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19

  // Step 1: Config die temperature register to take 1 temperature sample
  uint8_t value = 0x01;
  if (this->write_register(MAX3010X_REGISTER_DIE_TEMP_CFG, &value, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    return NAN;
  }

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
    // uint8_t response;
    // read_byte(MAX3010X_DIETEMPCONFIG, &response); //Original way
    // if ((response & 0x01) == 0) break; //We're done!

    // Check to see if DIE_TEMP_RDY interrupt is set
    uint8_t response;
    if (this->read_register(MAX3010X_REGISTER_ISR2, &response, 1, true) != i2c::ERROR_OK) {
      this->status_set_warning();
      return NAN;
    }
    if ((response & MAX3010X_INT_DIE_TEMP_RDY_ENABLE) > 0)
      break;   // We're done!
    delay(1);  // Let's not over burden the I2C bus
  }
  // TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = (int8_t)readRegister8(MAX3010X_REGISTER_DIE_TEMP_INT);
  uint8_t tempFrac = readRegister8(MAX3010X_REGISTER_DIE_TEMP_FRAC);  // Causes the clearing of the DIE_TEMP_RDY
                                                                      // interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float) tempInt + ((float) tempFrac * 0.0625);
}

//
// Device ID and Revision
//
uint8_t MAX3010xComponent::readPartID() { return readRegister8(MAX3010X_REGISTER_PART_ID); }

uint8_t MAX3010xComponent::readRevisionID() { revisionID = readRegister8(MAX3010X_REGISTER_REV_ID); return revisionID; }

uint8_t MAX3010xComponent::getRevisionID() { return revisionID; }

// Setup the sensor
// The MAX3010X has many settings. By default we select:
//  Sample Average = 4
//  Mode = MultiLED
//  ADC Range = 16384 (62.5pA per LSB)
//  Sample rate = 50
// Use the default setup if you are just getting started with the MAX3010X sensor
void MAX3010xComponent::setup_sensor(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth,
                              int adcRange) {
  softReset();  // Reset all configuration, threshold, and data registers to POR values

  // FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1)
    setFIFOAverage(MAX3010X_SAMPLEAVG_1);  // No averaging per FIFO record
  else if (sampleAverage == 2)
    setFIFOAverage(MAX3010X_SAMPLEAVG_2);
  else if (sampleAverage == 4)
    setFIFOAverage(MAX3010X_SAMPLEAVG_4);
  else if (sampleAverage == 8)
    setFIFOAverage(MAX3010X_SAMPLEAVG_8);
  else if (sampleAverage == 16)
    setFIFOAverage(MAX3010X_SAMPLEAVG_16);
  else if (sampleAverage == 32)
    setFIFOAverage(MAX3010X_SAMPLEAVG_32);
  else
    setFIFOAverage(MAX3010X_SAMPLEAVG_4);

  // setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover();  // Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 2)
    setLEDMode(MAX3010X_MODE_REDIRONLY);  // Red and IR
  else
    setLEDMode(MAX3010X_MODE_REDONLY);  // Red only
  activeLEDs = ledMode;                 // Used to control how many bytes to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (adcRange < 4096)
    setADCRange(MAX3010X_ADCRANGE_2048);  // 7.81pA per LSB
  else if (adcRange < 8192)
    setADCRange(MAX3010X_ADCRANGE_4096);  // 15.63pA per LSB
  else if (adcRange < 16384)
    setADCRange(MAX3010X_ADCRANGE_8192);  // 31.25pA per LSB
  else if (adcRange == 16384)
    setADCRange(MAX3010X_ADCRANGE_16384);  // 62.5pA per LSB
  else
    setADCRange(MAX3010X_ADCRANGE_2048);

  if (sampleRate < 100)
    setSampleRate(MAX3010X_SAMPLERATE_50);  // Take 50 samples per second
  else if (sampleRate < 200)
    setSampleRate(MAX3010X_SAMPLERATE_100);
  else if (sampleRate < 400)
    setSampleRate(MAX3010X_SAMPLERATE_200);
  else if (sampleRate < 800)
    setSampleRate(MAX3010X_SAMPLERATE_400);
  else if (sampleRate < 1000)
    setSampleRate(MAX3010X_SAMPLERATE_800);
  else if (sampleRate < 1600)
    setSampleRate(MAX3010X_SAMPLERATE_1000);
  else if (sampleRate < 3200)
    setSampleRate(MAX3010X_SAMPLERATE_1600);
  else if (sampleRate == 3200)
    setSampleRate(MAX3010X_SAMPLERATE_3200);
  else
    setSampleRate(MAX3010X_SAMPLERATE_50);

  // The longer the pulse width the longer range of detection you'll have
  // At 69us and 0.4mA it's about 2 inches
  // At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118)
    setPulseWidth(MAX3010X_PULSEWIDTH_69);  // Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215)
    setPulseWidth(MAX3010X_PULSEWIDTH_118);  // 16 bit resolution
  else if (pulseWidth < 411)
    setPulseWidth(MAX3010X_PULSEWIDTH_215);  // 17 bit resolution
  else if (pulseWidth == 411)
    setPulseWidth(MAX3010X_PULSEWIDTH_411);  // 18 bit resolution
  else
    setPulseWidth(MAX3010X_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Default is 0x1F which gets us 6.4mA
  // powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  // powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  // powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  // powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1)
    enableSlot(2, SLOT_IR_LED);
  // enableSlot(1, SLOT_RED_PILOT);
  // enableSlot(2, SLOT_IR_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO();  // Reset the FIFO before we begin checking the sensor
}

//
// Data Collection
//

// Tell caller how many samples are available
uint8_t MAX3010xComponent::available(void) {
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0)
    numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

// Report the most recent red value
uint32_t MAX3010xComponent::getRed(void) {
  // Check the sensor for new data for 250ms
  if (safeCheck(250))
    return (sense.red[sense.head]);
  else
    return (0);  // Sensor failed to find new data
}

// Report the most recent IR value
uint32_t MAX3010xComponent::getIR(void) {
  // Check the sensor for new data for 250ms
  if (safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return (0);  // Sensor failed to find new data
}

// Report the next Red value in the FIFO
uint32_t MAX3010xComponent::getFIFORed(void) { return (sense.red[sense.tail]); }

// Report the next IR value in the FIFO
uint32_t MAX3010xComponent::getFIFOIR(void) { return (sense.IR[sense.tail]); }

// Advance the tail
void MAX3010xComponent::nextSample(void) {
  if (available())  // Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE;  // Wrap condition
  }
}

// Polls the sensor for new data
// Call regularly
// If new data is available, it updates the head and tail in the main struct
// Returns number of new samples obtained
uint16_t MAX3010xComponent::check() {
  // Read register FIDO_DATA in (3-byte * number of active LED) chunks
  // Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t readPointer;
  uint8_t writePointer;
  if (this->read_register(MAX3010X_REGISTER_FIFO_RD_PTR, &readPointer, 1, false) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error reading FIFO read pointer");
    return 0;
  }
  if (this->read_register(MAX3010X_REGISTER_FIFO_WR_PTR, &writePointer, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    ESP_LOGW(TAG, "Error reading FIFO read pointer");
    return 0;
  }

  int numberOfSamples = 0;

  // Do we have new data?
  if (readPointer != writePointer) {
    // Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0)
      numberOfSamples += 32;  // Wrap condition

    // We now have the number of readings, now calc bytes to read
    // For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    // We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    // I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    // Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0) {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH) {
        // If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
        // 32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.

        toGet = I2C_BUFFER_LENGTH -
                (I2C_BUFFER_LENGTH % (activeLEDs * 3));  // Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      read_register(MAX3010X_REGISTER_FIFO_DATA, fifo, toGet, true);

      int ptr = 0;

      while (toGet > 0) {
        sense.head++;                // Advance the head of the storage struct
        sense.head %= STORAGE_SIZE;  // Wrap condition

        uint32_t tempLong;

        // Convert array to long
        //tempLong = temp[2] | (temp[1] << 8) | ((temp[0] & 0x3) << 16);
        tempLong = fifo[ptr+2] | (fifo[ptr+1] << 8) | ((fifo[ptr] & 0x3) << 16);
        ptr += 3;

        sense.red[sense.head] = tempLong;  // Store this reading into the sense array

        if (activeLEDs > 1) {
          // Burst read three more bytes - IR
          tempLong = fifo[ptr+2] | (fifo[ptr+1] << 8) | ((fifo[ptr] & 0x3) << 16);
          ptr += 3;

          sense.IR[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    }  // End while (bytesLeftToRead > 0)

  }  // End readPtr != writePtr

  ESP_LOGD(TAG, "End of check. sense.head=%d, sense.tail=%d, sense.IR[sense.head]=%d", sense.head, sense.tail, sense.IR[sense.head]);

  return (numberOfSamples);  // Let the world know how much new data we found
}

// Check for new data but give up after a certain amount of time
// Returns true if new data was found
// Returns false if new data was not found
bool MAX3010xComponent::safeCheck(uint8_t maxTimeToCheck) {
  uint32_t markTime = millis();

  while (1) {
    if (millis() - markTime > maxTimeToCheck)
      return (false);

    if (check() == true)  // We found new data!
      return (true);

    delay(1);
  }
}

// Given a register, read it, mask it, and then set the thing
void MAX3010xComponent::readModifyWrite(uint8_t reg, uint8_t mask, uint8_t thing) {
  // Grab current register context
  uint8_t value;

  if (this->read_register(reg, &value, 1, false) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  // Zero-out the portions of the register we're interested in
  value &= mask;
  value |= thing;

  // Change contents
  if (this->write_register(reg, &value, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
}

//
// Low-level I2C Communication
//
uint16_t MAX3010xComponent::read_u16_le_(uint8_t a_register) {
  uint16_t data = 0;
  if (this->read_register16(a_register, (uint8_t*)&data, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    return 0xBADD;
  }
  return (data >> 8) | (data << 8);
}
int16_t MAX3010xComponent::read_s16_le_(uint8_t a_register) { return this->read_u16_le_(a_register); }

uint8_t MAX3010xComponent::readRegister8(uint8_t a_register) {
  uint8_t data;
  if (this->read_register(a_register, &data, 1, true) != i2c::ERROR_OK) {
    this->status_set_warning();
    return 0xBD;
  }
  return data;
}

bool MAX3010xComponent::checkForBeat(uint32_t sample)
{
  bool beatDetected = false;

  //  Save current state
  IR_AC_Signal_Previous = IR_AC_Signal_Current;
  
  //This is good to view for debugging
  ESP_LOGD(TAG, "Signal_Current: %0d", IR_AC_Signal_Current);

  //  Process next data sample
  IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

  //  Detect positive zero crossing (rising edge)
  if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0)) {
    IR_AC_Max = IR_AC_Signal_max; //Adjust our AC max and min
    IR_AC_Min = IR_AC_Signal_min;

    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;

    //if ((IR_AC_Max - IR_AC_Min) > 100 & (IR_AC_Max - IR_AC_Min) < 1000)
    if (((IR_AC_Max - IR_AC_Min) > 20) && ((IR_AC_Max - IR_AC_Min) < 1000)) {
      //Heart beat!!!
      beatDetected = true;
    }
  }

  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) && (IR_AC_Signal_Current <= 0)) {
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }

  //  Find Maximum value in positive cycle
  if (positiveEdge && (IR_AC_Signal_Current > IR_AC_Signal_Previous)) {
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }

  //  Find Minimum value in negative cycle
  if (negativeEdge && (IR_AC_Signal_Current < IR_AC_Signal_Previous)) {
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }
  
  return beatDetected;
}

//  Average DC Estimator
int16_t MAX3010xComponent::averageDCEstimator(int32_t *p, uint16_t x)
{
  *p += ((((long) x << 15) - *p) >> 4);
  return (*p >> 15);
}

//  Integer multiplier
static int32_t mul16(int16_t x, int16_t y)
{
  return((long)x * (long)y);
}

static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

//  Low Pass FIR Filter
int16_t MAX3010xComponent::lowPassFIRFilter(int16_t din)
{  
  cbuf[offset] = din;

  int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);
  
  for (uint8_t i = 0 ; i < 11 ; i++) {
    z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
  }

  offset++;
  offset %= 32; //Wrap condition

  return(z >> 15);
}

}  // namespace max3010x
}  // namespace esphome
