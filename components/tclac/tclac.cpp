#include "tclac.h"

namespace esphome {
namespace tclac {

void tclacClimate::setup() {
  // Inicializa variáveis
  beeper_status_ = false;
  display_status_ = true;
  force_mode_status_ = false;
  module_display_status_ = true;
  vertical_direction_ = AirflowVerticalDirection::MAX_UP;
  horizontal_direction_ = AirflowHorizontalDirection::CENTER;
  vertical_swing_direction_ = VerticalSwingDirection::UP_DOWN;
  horizontal_swing_direction_ = HorizontalSwingDirection::LEFT_RIGHT;

  // Polling inicial
  sendData(poll, sizeof(poll));
}

void tclacClimate::update() {
  // Atualiza a cada ciclo do PollingComponent
  readData();
}

void tclacClimate::loop() {
  // Pode ser usado para ler UART contínua, se necessário
}

void tclacClimate::readData() {
  // Exemplo simplificado: lê os bytes do UART
  if (this->available()) {
    int len = this->read_array(dataRX, sizeof(dataRX));
    if (len > 0) {
      // Aqui poderíamos atualizar estados internos se necessário
      ESP_LOGD("tclac", "Received %d bytes: %s", len, getHex(dataRX, len).c_str());
    }
  }
}

void tclacClimate::sendData(byte *message, byte size) {
  byte cs = getChecksum(message, size - 1);
  message[size - 1] = cs;
  this->write_array(message, size);
}

String tclacClimate::getHex(byte *message, byte size) {
  String res = "";
  for (int i = 0; i < size; i++) {
    if (message[i] < 0x10) res += "0";
    res += String(message[i], HEX);
    if (i < size - 1) res += " ";
  }
  return res;
}

byte tclacClimate::getChecksum(const byte *message, size_t size) {
  byte cs = 0;
  for (size_t i = 0; i < size; i++) cs += message[i];
  return cs;
}

// --- Control functions ---

void tclacClimate::set_beeper_state(bool state) {
  beeper_status_ = state;
  ESP_LOGD("tclac", "Beeper %s", state ? "ON" : "OFF");
}

void tclacClimate::set_display_state(bool state) {
  display_status_ = state;
  ESP_LOGD("tclac", "Display %s", state ? "ON" : "OFF");
}

void tclacClimate::set_force_mode_state(bool state) {
  force_mode_status_ = state;
  ESP_LOGD("tclac", "Force mode %s", state ? "ON" : "OFF");
}

void tclacClimate::set_module_display_state(bool state) {
  module_display_status_ = state;
  ESP_LOGD("tclac", "Module Display %s", state ? "ON" : "OFF");
}

void tclacClimate::set_vertical_airflow(AirflowVerticalDirection direction) {
  vertical_direction_ = direction;
  ESP_LOGD("tclac", "Vertical airflow set to %d", static_cast<int>(direction));
}

void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection direction) {
  horizontal_direction_ = direction;
  ESP_LOGD("tclac", "Horizontal airflow set to %d", static_cast<int>(direction));
}

void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection direction) {
  vertical_swing_direction_ = direction;
  ESP_LOGD("tclac", "Vertical swing set to %d", static_cast<int>(direction));
}

void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection direction) {
  horizontal_swing_direction_ = direction;
  ESP_LOGD("tclac", "Horizontal swing set to %d", static_cast<int>(direction));
}

// --- Traits ---
ClimateTraits tclacClimate::traits() {
  auto t = traits_;
  t.set_supported_modes(supported_modes_);
  t.set_supported_presets(supported_presets_);
  t.set_supported_fan_modes(supported_fan_modes_);
  t.set_supported_swing_modes(supported_swing_modes_);
  t.set_visual_min_temperature(TCLAC_MIN_TEMPERATURE);
  t.set_visual_max_temperature(TCLAC_MAX_TEMPERATURE);
  t.set_visual_temperature_step(TCLAC_TARGET_TEMPERATURE_STEP);
  return t;
}

// --- Control command from YAML ---
void tclacClimate::control(const ClimateCall &call) {
  if (call.get_target_temperature().has_value()) {
    target_temperature_set = *call.get_target_temperature();
    ESP_LOGD("tclac", "Target temperature set to %d", target_temperature_set);
  }
  if (call.get_mode().has_value()) {
    switch_climate_mode = static_cast<uint8_t>(*call.get_mode());
    ESP_LOGD("tclac", "Mode set to %d", switch_climate_mode);
  }
  if (call.get_fan_mode().has_value()) {
    switch_fan_mode = static_cast<uint8_t>(*call.get_fan_mode());
    ESP_LOGD("tclac", "Fan mode set to %d", switch_fan_mode);
  }
  if (call.get_swing_mode().has_value()) {
    switch_swing_mode = static_cast<uint8_t>(*call.get_swing_mode());
    ESP_LOGD("tclac", "Swing mode set to %d", switch_swing_mode);
  }
}

// --- Supported sets ---
void tclacClimate::set_supported_modes(const std::set<ClimateMode> &modes) {
  supported_modes_ = modes;
}

void tclacClimate::set_supported_presets(const std::set<ClimatePreset> &presets) {
  supported_presets_ = presets;
}

void tclacClimate::set_supported_fan_modes(const std::set<ClimateFanMode> &modes) {
  supported_fan_modes_ = modes;
}

void tclacClimate::set_supported_swing_modes(const std::set<ClimateSwingMode> &modes) {
  supported_swing_modes_ = modes;
}

}  // namespace tclac
}  // namespace esphome
