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

static const char *const TAG = "max3010x.sensor";

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
static const uint8_t MAX3010X_INT_A_FULL_MASK = (byte) ~0b10000000;
static const uint8_t MAX3010X_INT_A_FULL_ENABLE = 0x80;
static const uint8_t MAX3010X_INT_A_FULL_DISABLE = 0x00;

static const uint8_t MAX3010X_INT_DATA_RDY_MASK = (byte) ~0b01000000;
static const uint8_t MAX3010X_INT_DATA_RDY_ENABLE = 0x40;
static const uint8_t MAX3010X_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX3010X_INT_ALC_OVF_MASK = (byte) ~0b00100000;
static const uint8_t MAX3010X_INT_ALC_OVF_ENABLE = 0x20;
static const uint8_t MAX3010X_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_MASK = (byte) ~0b00000010;
static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX3010X_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX3010X_SAMPLEAVG_MASK = (byte) ~0b11100000;
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

#define I2C_BUFFER_LENGTH 32

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
  if (readPartID() != MAX_3010X_EXPECTEDPARTID) {
    // Error -- Part ID read from MAX3010X does not match expected part ID.
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
    this->error_code_ = WRONG_CHIP_ID;
    return;
  }

  // Populate revision ID
  readRevisionID();

  return true;
}

//
// Configuration
//

// Begin Interrupt configuration
uint8_t MAX3010xComponent::getINT1(void) {
  uint8_t data;
  read_byte(MAX3010X_REGISTER_ISR1, &data);
  return data;
}
uint8_t MAX3010xComponent::getINT2(void) {
  uint8_t data;
  read_byte(MAX3010X_REGISTER_ISR2, &data);
  return data;
}

void MAX3010xComponent::enableAFULL(void) {
  bitMask(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_A_FULL_MASK, MAX3010X_INT_A_FULL_ENABLE);
}
void MAX3010xComponent::disableAFULL(void) {
  bitMask(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_A_FULL_MASK, MAX3010X_INT_A_FULL_DISABLE);
}

void MAX3010xComponent::enableDATARDY(void) {
  bitMask(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_DATA_RDY_MASK, MAX3010X_INT_DATA_RDY_ENABLE);
}
void MAX3010xComponent::disableDATARDY(void) {
  bitMask(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_DATA_RDY_MASK, MAX3010X_INT_DATA_RDY_DISABLE);
}

void MAX3010xComponent::enableALCOVF(void) {
  bitMask(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_ALC_OVF_MASK, MAX3010X_INT_ALC_OVF_ENABLE);
}
void MAX3010xComponent::disableALCOVF(void) {
  bitMask(MAX3010X_REGISTER_INTEN1, MAX3010X_INT_ALC_OVF_MASK, MAX3010X_INT_ALC_OVF_DISABLE);
}

void MAX3010xComponent::enableDIETEMPRDY(void) {
  bitMask(MAX3010X_REGISTER_INTEN2, MAX3010X_INT_DIE_TEMP_RDY_MASK, MAX3010X_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX3010xComponent::disableDIETEMPRDY(void) {
  bitMask(MAX3010X_REGISTER_INTEN2, MAX3010X_INT_DIE_TEMP_RDY_MASK, MAX3010X_INT_DIE_TEMP_RDY_DISABLE);
}

// End Interrupt configuration

void MAX3010xComponent::softReset(void) {
  bitMask(MAX3010X_REGISTER_MODE_CFG, MAX3010X_RESET_MASK, MAX3010X_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
    uint8_t response;
    read_byte(MAX3010X_REGISTER_MODE_CFG, &response);
    if ((response & MAX3010X_RESET) == 0)
      break;   // We're done!
    delay(1);  // Let's not over burden the I2C bus
  }
}

void MAX3010xComponent::shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX3010X_REGISTER_MODE_CFG, MAX3010X_SHUTDOWN_MASK, MAX3010X_SHUTDOWN);
}

void MAX3010xComponent::wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX3010X_REGISTER_MODE_CFG, MAX3010X_SHUTDOWN_MASK, MAX3010X_WAKEUP);
}

void MAX3010xComponent::setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX3010X_REGISTER_MODE_CFG, MAX3010X_MODE_MASK, mode);
}

void MAX3010xComponent::setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX3010X_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX3010X_REGISTER_SPO2_CFG, MAX3010X_ADCRANGE_MASK, adcRange);
}

void MAX3010xComponent::setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX3010X_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX3010X_REGISTER_SPO2_CFG, MAX3010X_SAMPLERATE_MASK, sampleRate);
}

void MAX3010xComponent::setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX3010X_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX3010X_REGISTER_SPO2_CFG, MAX3010X_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX3010xComponent::setPulseAmplitudeRed(uint8_t amplitude) {
  write_byte(MAX3010X_REGISTER_LED_PULSE_AMPL1_CFG, amplitude);
}

void MAX3010xComponent::setPulseAmplitudeIR(uint8_t amplitude) {
  write_byte(MAX3010X_REGISTER_LED_PULSE_AMPL2_CFG, amplitude);
}

// Given a slot number assign a thing to it
// Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
// Assigning a SLOT_RED_LED will pulse LED
// Assigning a SLOT_RED_PILOT will ??
void MAX3010xComponent::enableSlot(uint8_t slotNumber, uint8_t device) {
  uint8_t originalContents;

  switch (slotNumber) {
    case (1):
      bitMask(MAX3010X_REGISTER_MULTILED_MODE_CTRL1, MAX3010X_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX3010X_REGISTER_MULTILED_MODE_CTRL1, MAX3010X_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX3010X_REGISTER_MULTILED_MODE_CTRL2, MAX3010X_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX3010X_REGISTER_MULTILED_MODE_CTRL2, MAX3010X_SLOT4_MASK, device << 4);
      break;
    default:
      // Shouldn't be here!
      break;
  }
}

// Clears all slot assignments
void MAX3010xComponent::disableSlots(void) {
  write_byte(MAX3010X_REGISTER_MULTILED_MODE_CTRL1, 0);
  write_byte(MAX3010X_REGISTER_MULTILED_MODE_CTRL2, 0);
}

//
// FIFO Configuration
//

// Set sample average (Table 3, Page 18)
void MAX3010xComponent::setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_SAMPLEAVG_MASK, numberOfSamples);
}

// Resets all points to start in a known state
// Page 15 recommends clearing FIFO before beginning a read
void MAX3010xComponent::clearFIFO(void) {
  write_byte(MAX3010X_REGISTER_FIFO_WR_PTR, 0);
  // write_byte(MAX3010X_REGISTER_FIFO, 0);
  write_byte(MAX3010X_REGISTER_FIFO_RD_PTR, 0);
}

// Enable roll over if FIFO over flows
void MAX3010xComponent::enableFIFORollover(void) {
  bitMask(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_ROLLOVER_MASK, MAX3010X_ROLLOVER_ENABLE);
}

// Disable roll over if FIFO over flows
void MAX3010xComponent::disableFIFORollover(void) {
  bitMask(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_ROLLOVER_MASK, MAX3010X_ROLLOVER_DISABLE);
}

// Set number of samples to trigger the almost full interrupt (Page 18)
// Power on default is 32 samples
// Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX3010xComponent::setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX3010X_REGISTER_FIFO_CFG, MAX3010X_A_FULL_MASK, numberOfSamples);
}

// Read the FIFO Write Pointer
uint8_t MAX3010xComponent::getWritePointer(void) {
  uint8_t ptr;
  read_byte(MAX3010X_REGISTER_FIFO_WR_PTR, &ptr);
  return ptr;
}

// Read the FIFO Read Pointer
uint8_t MAX3010xComponent::getReadPointer(void) {
  uint8_t ptr;
  read_byte(MAX3010X_REGISTER_FIFO_RD_PTR, &ptr);
  return ptr;
}

// Die Temperature
// Returns temp in C
float MAX3010xComponent::readTemperature() {
  // DIE_TEMP_RDY interrupt must be enabled
  // See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19

  // Step 1: Config die temperature register to take 1 temperature sample
  write_byte(MAX3010X_REGISTER_DIE_TEMP_CFG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100) {
    // uint8_t response;
    // read_byte(MAX3010X_DIETEMPCONFIG, &response); //Original way
    // if ((response & 0x01) == 0) break; //We're done!

    // Check to see if DIE_TEMP_RDY interrupt is set
    uint8_t response;
    read_byte(MAX3010X_REGISTER_ISR2, &response);
    if ((response & MAX3010X_INT_DIE_TEMP_RDY_ENABLE) > 0)
      break;   // We're done!
    delay(1);  // Let's not over burden the I2C bus
  }
  // TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readRegister8(MAX3010X_REGISTER_DIE_TEMP_INT);
  uint8_t tempFrac = readRegister8(MAX3010X_REGISTER_DIE_TEMP_FRAC);  // Causes the clearing of the DIE_TEMP_RDY
                                                                      // interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float) tempInt + ((float) tempFrac * 0.0625);
}

// Returns die temp in F
float MAX3010xComponent::readTemperatureF() {
  float temp = readTemperature();

  if (temp != -999.0)
    temp = temp * 1.8 + 32.0;

  return (temp);
}

//
// Device ID and Revision
//
uint8_t MAX3010xComponent::readPartID() { return readRegister8(MAX3010X_REGISTER_PART_ID); }

void MAX3010xComponent::readRevisionID() { revisionID = readRegister8(MAX3010X_REGISTER_REV_ID); }

uint8_t MAX3010xComponent::getRevisionID() { return revisionID; }

// Setup the sensor
// The MAX3010X has many settings. By default we select:
//  Sample Average = 4
//  Mode = MultiLED
//  ADC Range = 16384 (62.5pA per LSB)
//  Sample rate = 50
// Use the default setup if you are just getting started with the MAX3010X sensor
void MAX3010xComponent::setup_sensor(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth,
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
uint16_t MAX3010xComponent::check(void) {
  // Read register FIDO_DATA in (3-byte * number of active LED) chunks
  // Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();

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

      while (toGet > 0) {
        sense.head++;                // Advance the head of the storage struct
        sense.head %= STORAGE_SIZE;  // Wrap condition

        byte temp[sizeof(uint32_t)];  // Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        // Burst read three bytes - RED
        read_bytes(MAX3010X_REGISTER_FIFO_DATA, temp, toGet);

        // Convert array to long
        tempLong = temp[2] | (temp[1] << 8) | ((temp[0] & 0x3) << 16);

        sense.red[sense.head] = tempLong;  // Store this reading into the sense array

        if (activeLEDs > 1) {
          // Burst read three more bytes - IR
          read_bytes(MAX3010X_REGISTER_FIFO_DATA, temp, toGet);
          tempLong = temp[2] | (temp[1] << 8) | ((temp[0] & 0x3) << 16);

          sense.IR[sense.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    }  // End while (bytesLeftToRead > 0)

  }  // End readPtr != writePtr

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
void MAX3010xComponent::bitMask(uint8_t reg, uint8_t mask, uint8_t thing) {
  // Grab current register context
  uint8_t originalContents;

  read_byte(reg, &originalContents);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  write_byte(reg, originalContents | thing);
}

//
// Low-level I2C Communication
//
uint16_t MAX3010xComponent::read_u16_le_(uint8_t a_register) {
  uint16_t data = 0;
  this->read_byte_16(a_register, &data);
  return (data >> 8) | (data << 8);
}
int16_t MAX3010xComponent::read_s16_le_(uint8_t a_register) { return this->read_u16_le_(a_register); }

uint8_t MAX3010xComponent::readRegister8(uint8_t a_register) {
  uint8_t data;
  read_byte(a_register, &data);
  return data;
}

bool MAX3010xComponent::read_byte(uint8_t a_register, uint8_t *data) { return I2CDevice::read_byte(a_register, data); };
bool MAX3010xComponent::write_byte(uint8_t a_register, uint8_t data) {
  return I2CDevice::write_byte(a_register, data);
};
bool MAX3010xComponent::read_bytes(uint8_t a_register, uint8_t *data, size_t len) {
  return I2CDevice::read_bytes(a_register, data, len);
};
bool MAX3010xComponent::read_byte_16(uint8_t a_register, uint16_t *data) {
  return I2CDevice::read_byte_16(a_register, data);
};

// Not updated from BME280:

#if 0

void BME280Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BME280...");
  uint8_t chip_id = 0;

  // Mark as not failed before initializing. Some devices will turn off sensors to save on batteries
  // and when they come back on, the COMPONENT_STATE_FAILED bit must be unset on the component.
  if ((this->component_state_ & COMPONENT_STATE_MASK) == COMPONENT_STATE_FAILED) {
    this->component_state_ &= ~COMPONENT_STATE_MASK;
    this->component_state_ |= COMPONENT_STATE_CONSTRUCTION;
  }

  if (!this->read_byte(BME280_REGISTER_CHIPID, &chip_id)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  if (chip_id != 0x60) {
    this->error_code_ = WRONG_CHIP_ID;
    this->mark_failed();
    return;
  }

  // Send a soft reset.
  if (!this->write_byte(BME280_REGISTER_RESET, BME280_SOFT_RESET)) {
    this->mark_failed();
    return;
  }
  // Wait until the NVM data has finished loading.
  uint8_t status;
  uint8_t retry = 5;
  do {  // NOLINT
    delay(2);
    if (!this->read_byte(BME280_REGISTER_STATUS, &status)) {
      ESP_LOGW(TAG, "Error reading status register.");
      this->mark_failed();
      return;
    }
  } while ((status & BME280_STATUS_IM_UPDATE) && (--retry));
  if (status & BME280_STATUS_IM_UPDATE) {
    ESP_LOGW(TAG, "Timeout loading NVM.");
    this->mark_failed();
    return;
  }

  // Read calibration
  this->calibration_.t1 = read_u16_le_(BME280_REGISTER_DIG_T1);
  this->calibration_.t2 = read_s16_le_(BME280_REGISTER_DIG_T2);
  this->calibration_.t3 = read_s16_le_(BME280_REGISTER_DIG_T3);

  this->calibration_.p1 = read_u16_le_(BME280_REGISTER_DIG_P1);
  this->calibration_.p2 = read_s16_le_(BME280_REGISTER_DIG_P2);
  this->calibration_.p3 = read_s16_le_(BME280_REGISTER_DIG_P3);
  this->calibration_.p4 = read_s16_le_(BME280_REGISTER_DIG_P4);
  this->calibration_.p5 = read_s16_le_(BME280_REGISTER_DIG_P5);
  this->calibration_.p6 = read_s16_le_(BME280_REGISTER_DIG_P6);
  this->calibration_.p7 = read_s16_le_(BME280_REGISTER_DIG_P7);
  this->calibration_.p8 = read_s16_le_(BME280_REGISTER_DIG_P8);
  this->calibration_.p9 = read_s16_le_(BME280_REGISTER_DIG_P9);

  this->calibration_.h1 = read_u8_(BME280_REGISTER_DIG_H1);
  this->calibration_.h2 = read_s16_le_(BME280_REGISTER_DIG_H2);
  this->calibration_.h3 = read_u8_(BME280_REGISTER_DIG_H3);
  this->calibration_.h4 = read_u8_(BME280_REGISTER_DIG_H4) << 4 | (read_u8_(BME280_REGISTER_DIG_H4 + 1) & 0x0F);
  this->calibration_.h5 = read_u8_(BME280_REGISTER_DIG_H5 + 1) << 4 | (read_u8_(BME280_REGISTER_DIG_H5) >> 4);
  this->calibration_.h6 = read_u8_(BME280_REGISTER_DIG_H6);

  uint8_t humid_control_val = 0;
  if (!this->read_byte(BME280_REGISTER_CONTROLHUMID, &humid_control_val)) {
    this->mark_failed();
    return;
  }
  humid_control_val &= ~0b00000111;
  humid_control_val |= this->humidity_oversampling_ & 0b111;
  if (!this->write_byte(BME280_REGISTER_CONTROLHUMID, humid_control_val)) {
    this->mark_failed();
    return;
  }

  uint8_t config_register = 0;
  if (!this->read_byte(BME280_REGISTER_CONFIG, &config_register)) {
    this->mark_failed();
    return;
  }
  config_register &= ~0b11111100;
  config_register |= 0b101 << 5;  // 1000 ms standby time
  config_register |= (this->iir_filter_ & 0b111) << 2;
  if (!this->write_byte(BME280_REGISTER_CONFIG, config_register)) {
    this->mark_failed();
    return;
  }
}
void BME280Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BME280:");
  switch (this->error_code_) {
    case COMMUNICATION_FAILED:
      ESP_LOGE(TAG, "Communication with BME280 failed!");
      break;
    case WRONG_CHIP_ID:
      ESP_LOGE(TAG, "BME280 has wrong chip ID! Is it a BME280?");
      break;
    case NONE:
    default:
      break;
  }
  ESP_LOGCONFIG(TAG, "  IIR Filter: %s", iir_filter_to_str(this->iir_filter_));
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->temperature_oversampling_));
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
  ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->pressure_oversampling_));
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->humidity_oversampling_));
}
float BME280Component::get_setup_priority() const { return setup_priority::DATA; }

inline uint8_t oversampling_to_time(BME280Oversampling over_sampling) { return (1 << uint8_t(over_sampling)) >> 1; }

void BME280Component::update() {
  // Enable sensor
  ESP_LOGV(TAG, "Sending conversion request...");
  uint8_t meas_value = 0;
  meas_value |= (this->temperature_oversampling_ & 0b111) << 5;
  meas_value |= (this->pressure_oversampling_ & 0b111) << 2;
  meas_value |= BME280_MODE_FORCED;
  if (!this->write_byte(BME280_REGISTER_CONTROL, meas_value)) {
    this->status_set_warning();
    return;
  }

  float meas_time = 1.5f;
  meas_time += 2.3f * oversampling_to_time(this->temperature_oversampling_);
  meas_time += 2.3f * oversampling_to_time(this->pressure_oversampling_) + 0.575f;
  meas_time += 2.3f * oversampling_to_time(this->humidity_oversampling_) + 0.575f;

  this->set_timeout("data", uint32_t(ceilf(meas_time)), [this]() {
    uint8_t data[8];
    if (!this->read_bytes(BME280_REGISTER_MEASUREMENTS, data, 8)) {
      ESP_LOGW(TAG, "Error reading registers.");
      this->status_set_warning();
      return;
    }
    int32_t t_fine = 0;
    float const temperature = this->read_temperature_(data, &t_fine);
    if (std::isnan(temperature)) {
      ESP_LOGW(TAG, "Invalid temperature, cannot read pressure & humidity values.");
      this->status_set_warning();
      return;
    }
    float const pressure = this->read_pressure_(data, t_fine);
    float const humidity = this->read_humidity_(data, t_fine);

    ESP_LOGV(TAG, "Got temperature=%.1f°C pressure=%.1fhPa humidity=%.1f%%", temperature, pressure, humidity);
    if (this->temperature_sensor_ != nullptr)
      this->temperature_sensor_->publish_state(temperature);
    if (this->pressure_sensor_ != nullptr)
      this->pressure_sensor_->publish_state(pressure);
    if (this->humidity_sensor_ != nullptr)
      this->humidity_sensor_->publish_state(humidity);
    this->status_clear_warning();
  });
}
float BME280Component::read_temperature_(const uint8_t *data, int32_t *t_fine) {
  int32_t adc = ((data[3] & 0xFF) << 16) | ((data[4] & 0xFF) << 8) | (data[5] & 0xFF);
  adc >>= 4;
  if (adc == 0x80000) {
    // temperature was disabled
    return NAN;
  }

  const int32_t t1 = this->calibration_.t1;
  const int32_t t2 = this->calibration_.t2;
  const int32_t t3 = this->calibration_.t3;

  int32_t const var1 = (((adc >> 3) - (t1 << 1)) * t2) >> 11;
  int32_t const var2 = (((((adc >> 4) - t1) * ((adc >> 4) - t1)) >> 12) * t3) >> 14;
  *t_fine = var1 + var2;

  float const temperature = (*t_fine * 5 + 128);
  return temperature / 25600.0f;
}

float BME280Component::read_pressure_(const uint8_t *data, int32_t t_fine) {
  int32_t adc = ((data[0] & 0xFF) << 16) | ((data[1] & 0xFF) << 8) | (data[2] & 0xFF);
  adc >>= 4;
  if (adc == 0x80000) {
    // pressure was disabled
    return NAN;
  }
  const int64_t p1 = this->calibration_.p1;
  const int64_t p2 = this->calibration_.p2;
  const int64_t p3 = this->calibration_.p3;
  const int64_t p4 = this->calibration_.p4;
  const int64_t p5 = this->calibration_.p5;
  const int64_t p6 = this->calibration_.p6;
  const int64_t p7 = this->calibration_.p7;
  const int64_t p8 = this->calibration_.p8;
  const int64_t p9 = this->calibration_.p9;

  int64_t var1, var2, p;
  var1 = int64_t(t_fine) - 128000;
  var2 = var1 * var1 * p6;
  var2 = var2 + ((var1 * p5) << 17);
  var2 = var2 + (p4 << 35);
  var1 = ((var1 * var1 * p3) >> 8) + ((var1 * p2) << 12);
  var1 = ((int64_t(1) << 47) + var1) * p1 >> 33;

  if (var1 == 0)
    return NAN;

  p = 1048576 - adc;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (p9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = (p8 * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (p7 << 4);
  return (p / 256.0f) / 100.0f;
}

float BME280Component::read_humidity_(const uint8_t *data, int32_t t_fine) {
  uint16_t const raw_adc = ((data[6] & 0xFF) << 8) | (data[7] & 0xFF);
  if (raw_adc == 0x8000)
    return NAN;

  int32_t const adc = raw_adc;

  const int32_t h1 = this->calibration_.h1;
  const int32_t h2 = this->calibration_.h2;
  const int32_t h3 = this->calibration_.h3;
  const int32_t h4 = this->calibration_.h4;
  const int32_t h5 = this->calibration_.h5;
  const int32_t h6 = this->calibration_.h6;

  int32_t v_x1_u32r = t_fine - 76800;

  v_x1_u32r = ((((adc << 14) - (h4 << 20) - (h5 * v_x1_u32r)) + 16384) >> 15) *
              (((((((v_x1_u32r * h6) >> 10) * (((v_x1_u32r * h3) >> 11) + 32768)) >> 10) + 2097152) * h2 + 8192) >> 14);

  v_x1_u32r = v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * h1) >> 4);

  v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
  v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;
  float const h = v_x1_u32r >> 12;

  return h / 1024.0f;
}
void BME280Component::set_temperature_oversampling(BME280Oversampling temperature_over_sampling) {
  this->temperature_oversampling_ = temperature_over_sampling;
}
void BME280Component::set_pressure_oversampling(BME280Oversampling pressure_over_sampling) {
  this->pressure_oversampling_ = pressure_over_sampling;
}
void BME280Component::set_humidity_oversampling(BME280Oversampling humidity_over_sampling) {
  this->humidity_oversampling_ = humidity_over_sampling;
}
void BME280Component::set_iir_filter(BME280IIRFilter iir_filter) { this->iir_filter_ = iir_filter; }
uint8_t BME280Component::read_u8_(uint8_t a_register) {
  uint8_t data = 0;
  this->read_byte(a_register, &data);
  return data;
}
uint16_t BME280Component::read_u16_le_(uint8_t a_register) {
  uint16_t data = 0;
  this->read_byte_16(a_register, &data);
  return (data >> 8) | (data << 8);
}
int16_t BME280Component::read_s16_le_(uint8_t a_register) { return this->read_u16_le_(a_register); }

bool BME280I2CComponent::read_byte(uint8_t a_register, uint8_t *data) {
  return I2CDevice::read_byte(a_register, data);
};
bool BME280I2CComponent::write_byte(uint8_t a_register, uint8_t data) {
  return I2CDevice::write_byte(a_register, data);
};
bool BME280I2CComponent::read_bytes(uint8_t a_register, uint8_t *data, size_t len) {
  return I2CDevice::read_bytes(a_register, data, len);
};
bool BME280I2CComponent::read_byte_16(uint8_t a_register, uint16_t *data) {
  return I2CDevice::read_byte_16(a_register, data);
};

void BME280I2CComponent::dump_config() {
  LOG_I2C_DEVICE(this);
  BME280Component::dump_config();
}

#endif

}  // namespace max3010x
}  // namespace esphome
