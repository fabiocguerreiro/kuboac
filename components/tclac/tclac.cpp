#include "tclac.h"

namespace esphome {
namespace tclac {

tclacClimate::tclacClimate()
    : PollingComponent(5000),
      checksum_(0),
      poll_{0xBB,0x00,0x01,0x04,0x02,0x01,0x00,0xBD},
      beeper_status_(false),
      display_status_(false),
      force_mode_status_(false),
      module_display_status_(false),
      vertical_direction_(AirflowVerticalDirection::MAX_UP),
      vertical_swing_direction_(VerticalSwingDirection::UP_DOWN),
      rx_led_pin_(nullptr),
      tx_led_pin_(nullptr) { }

void tclacClimate::setup() { }
void tclacClimate::loop() { }
void tclacClimate::update() { }
void tclacClimate::control(const ClimateCall &call) { }

void tclacClimate::set_beeper_state(bool state) { beeper_status_ = state; }
void tclacClimate::set_display_state(bool state) { display_status_ = state; }
void tclacClimate::set_force_mode_state(bool state) { force_mode_status_ = state; }
void tclacClimate::set_module_display_state(bool state) { module_display_status_ = state; }
void tclacClimate::set_vertical_airflow(AirflowVerticalDirection direction) { vertical_direction_ = direction; }
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection direction) { vertical_swing_direction_ = direction; }

ClimateTraits tclacClimate::traits() {
    auto t = ClimateTraits();
    t.set_supports_current_temperature(true);
    t.set_supports_target_temperature(true);
    t.set_min_temperature(16.0);
    t.set_max_temperature(31.0);
    t.set_temperature_step(1.0);
    return t;
}

void tclacClimate::set_supported_presets(const std::set<ClimatePreset> &presets) { }
void tclacClimate::set_supported_modes(const std::set<ClimateMode> &modes) { }
void tclacClimate::set_supported_fan_modes(const std::set<ClimateFanMode> &modes) { }
void tclacClimate::set_supported_swing_modes(const std::set<ClimateSwingMode> &modes) { }

}  // namespace tclac
}  // namespace esphome
