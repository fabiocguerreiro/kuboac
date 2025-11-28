#include "tclac.h"

namespace esphome {
namespace tclac {

tclacClimate::tclacClimate()
    : PollingComponent(5000),
      checksum_(0),
      switch_preset_(0),
      switch_fan_mode_(0),
      switch_swing_mode_(0),
      target_temperature_set_(0),
      switch_climate_mode_(0),
      allow_take_control_(false),
      beeper_status_(false),
      display_status_(false),
      force_mode_status_(false),
      module_display_status_(false),
      vertical_direction_(AirflowVerticalDirection::MAX_UP),
      horizontal_direction_(AirflowHorizontalDirection::CENTER),
      vertical_swing_direction_(VerticalSwingDirection::UP_DOWN),
      horizontal_swing_direction_(HorizontalSwingDirection::CENTER)
{
    poll_[0]=0xBB; poll_[1]=0x00; poll_[2]=0x01; poll_[3]=0x04;
    poll_[4]=0x02; poll_[5]=0x01; poll_[6]=0x00; poll_[7]=0xBD;
}

// --- Implementação de métodos vazios (pode preencher depois) ---
void tclacClimate::setup() {}
void tclacClimate::loop() {}
void tclacClimate::update() {}
void tclacClimate::control(const ClimateCall &call) {}
void tclacClimate::readData() {}
void tclacClimate::takeControl() {}
void tclacClimate::sendData(byte *message, byte size) {}
byte tclacClimate::getChecksum(const byte *message, size_t size) { return 0; }
String tclacClimate::getHex(byte *message, byte size) { return ""; }

void tclacClimate::set_beeper_state(bool state) { beeper_status_ = state; }
void tclacClimate::set_display_state(bool state) { display_status_ = state; }
void tclacClimate::set_force_mode_state(bool state) { force_mode_status_ = state; }
void tclacClimate::set_module_display_state(bool state) { module_display_status_ = state; }

void tclacClimate::set_rx_led_pin(GPIOPin *rx_led_pin) { rx_led_pin_ = rx_led_pin; }
void tclacClimate::set_tx_led_pin(GPIOPin *tx_led_pin) { tx_led_pin_ = tx_led_pin; }

void tclacClimate::set_vertical_airflow(AirflowVerticalDirection direction) { vertical_direction_ = direction; }
void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection direction) { horizontal_direction_ = direction; }
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection direction) { vertical_swing_direction_ = direction; }
void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection direction) { horizontal_swing_direction_ = direction; }

void tclacClimate::set_supported_presets(const std::set<ClimatePreset> &presets) { supported_presets_ = presets; }
void tclacClimate::set_supported_modes(const std::set<ClimateMode> &modes) { supported_modes_ = modes; }
void tclacClimate::set_supported_fan_modes(const std::set<ClimateFanMode> &modes) { supported_fan_modes_ = modes; }
void tclacClimate::set_supported_swing_modes(const std::set<ClimateSwingMode> &modes) { supported_swing_modes_ = modes; }

ClimateTraits tclacClimate::traits() {
    return traits_;
}

}  // namespace tclac
}  // namespace esphome
