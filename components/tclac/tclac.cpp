namespace esphome {
namespace tclac {

ClimateTraits tclacClimate::traits() {
	auto traits = climate::ClimateTraits();

	traits.set_supports_action(false);
	traits.set_supports_current_temperature(true);
	traits.set_supports_two_point_target_temperature(false);

	// Substituição do deprecated: usar add_supported_* em vez de set_supported_*
	for (auto mode : supported_modes_)
		traits.add_supported_mode(mode);
	for (auto preset : supported_presets_)
		traits.add_supported_preset(preset);
	for (auto fan_mode : supported_fan_modes_)
		traits.add_supported_fan_mode(fan_mode);
	for (auto swing_mode : supported_swing_modes_)
		traits.add_supported_swing_mode(swing_mode);

	traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
	traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);
	traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
	traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
	traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);

	return traits;
}

// Mantemos os setters apenas para compatibilidade interna
void tclacClimate::set_supported_modes(const std::set<ClimateMode> &modes) {
	this->supported_modes_ = modes;
}
void tclacClimate::set_supported_presets(const std::set<ClimatePreset> &presets) {
	this->supported_presets_ = presets;
}
void tclacClimate::set_supported_fan_modes(const std::set<ClimateFanMode> &modes){
	this->supported_fan_modes_ = modes;
}
void tclacClimate::set_supported_swing_modes(const std::set<ClimateSwingMode> &modes) {
	this->supported_swing_modes_ = modes;
}

} // namespace tclac
} // namespace esphome
