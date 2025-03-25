#include "dm556.h"
#include "esphome/core/log.h"

namespace esphome {
namespace dm556 {

static const char *const TAG = "dm556.stepper";

void DM556::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DM556...");
  if (this->enable_pin_ != nullptr) {
    this->enable_pin_->setup();
    this->enable_pin_->digital_write(false);
    this->enable_pin_state_ = false;
  }
  this->step_pin_->setup();
  this->step_pin_->digital_write(false);
  this->dir_pin_->setup();
  this->dir_pin_->digital_write(false);
}
void DM556::dump_config() {
  ESP_LOGCONFIG(TAG, "DM556:");
  LOG_PIN("  Step Pin: ", this->step_pin_);
  LOG_PIN("  Dir Pin: ", this->dir_pin_);
  LOG_PIN("  Enable Pin: ", this->enable_pin_);
  LOG_STEPPER(this);
}
void DM556::loop() {
  bool at_target = this->has_reached_target();
  if (this->enable_pin_ != nullptr) {
    bool enable_falling_edge = enable_pin_state_ && !at_target;
    this->enable_pin_->digital_write(!at_target);
    this->enable_pin_state_ = !at_target;
    if (enable_falling_edge) {
      delayMicroseconds(1000);
    }
  }
  if (at_target) {
    this->high_freq_.stop();
  } else {
    this->high_freq_.start();
  }

  int32_t dir = this->should_step_();
  if (dir == 0)
    return;

  this->dir_pin_->digital_write(dir == 1);

  // TODO: Should this instead use ledc (PWM)?
  delayMicroseconds(50); // 5 us setup time for the DIR signal is required --> this delay could be reduced
  this->step_pin_->digital_write(true);
  delayMicroseconds(5); // Minimum pulse width 2.5 us --> doubled
  this->step_pin_->digital_write(false);
}

}  // namespace dm556
}  // namespace esphome
