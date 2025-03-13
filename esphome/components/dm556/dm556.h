#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/stepper/stepper.h"

namespace esphome {
namespace dm556 {

class DM556 : public stepper::Stepper, public Component {
 public:
  void set_step_pin(GPIOPin *step_pin) { step_pin_ = step_pin; }
  void set_dir_pin(GPIOPin *dir_pin) { dir_pin_ = dir_pin; }
  void set_enable_pin(GPIOPin *enable_pin) { this->enable_pin_ = enable_pin; }
  void setup() override;
  void dump_config() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  GPIOPin *step_pin_;
  GPIOPin *dir_pin_;
  GPIOPin *enable_pin_{nullptr};
  bool enable_pin_state_;
  HighFrequencyLoopRequester high_freq_;
};

}  // namespace dm556
}  // namespace esphome
