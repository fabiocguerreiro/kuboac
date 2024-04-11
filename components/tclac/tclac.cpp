#include "esphome.h"
#include "esphome/core/defines.h"
#include "tclac.h"

namespace esphome{
namespace tclac{


ClimateTraits tclacClimate::traits() {
	auto traits = climate::ClimateTraits();

	traits.set_supports_action(false);
	traits.set_supports_current_temperature(true);
	traits.set_supports_two_point_target_temperature(false);

	traits.set_supported_modes(this->supported_modes_);
	traits.set_supported_fan_modes(this->supported_fan_modes_);
	traits.set_supported_swing_modes(this->supported_swing_modes_);

	traits.add_supported_mode(climate::CLIMATE_MODE_OFF);		// The air conditioner's off mode is always available
	traits.add_supported_mode(climate::CLIMATE_MODE_AUTO);		// Automatic mode of the air conditioner is also always available
	traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);	// The automatic mode of the fan is always available
	traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);    // The off mode of vent swinging is always available

	traits.set_visual_temperature_step(STEP_TEMPERATURE);
	traits.set_visual_min_temperature(MIN_SET_TEMPERATURE);
	traits.set_visual_max_temperature(MAX_SET_TEMPERATURE);

	return traits;
}


void tclacClimate::setup() {

#ifdef CONF_RX_LED
	this->rx_led_pin_->setup();
	this->rx_led_pin_->digital_write(false);
#endif
#ifdef CONF_TX_LED
	this->tx_led_pin_->setup();
	this->tx_led_pin_->digital_write(false);
#endif
}

void tclacClimate::loop()  {
	// If there is something in the UART buffer, then read that something
	if (esphome::uart::UARTDevice::available() > 0) {
		dataShow(0, true);
		dataRX[0] = esphome::uart::UARTDevice::read();
		// If the received byte is not a header (0xBB), then simply exit the loop
		if (dataRX[0] != 0xBB) {
			ESP_LOGD("TCL", "Wrong byte");
			dataShow(0,0);
			return;
		}
		// If the header (0xBB) matches, then start reading the next 4 bytes sequentially
		delay(5);
		dataRX[1] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[2] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[3] = esphome::uart::UARTDevice::read();
		delay(5);
		dataRX[4] = esphome::uart::UARTDevice::read();

		auto raw = getHex(dataRX, 5);
		ESP_LOGD("TCL", "first 5 byte : %s ", raw.c_str());

		// Of the first 5 bytes, we need the fifth one - it contains the message length
		esphome::uart::UARTDevice::read_array(dataRX+5, dataRX[4]+1);

		byte check = getChecksum(dataRX, sizeof(dataRX));

		raw = getHex(dataRX, sizeof(dataRX));
		ESP_LOGD("TCL", "RX full : %s ", raw.c_str());
		// Check the checksum
		if (check != dataRX[60]) {
			ESP_LOGD("TCL", "Invalid checksum %x", check);
			tclacClimate::dataShow(0,0);
			return;
		} else {
			ESP_LOGD("TCL", "checksum OK %x", check);
		}
		tclacClimate::dataShow(0,0);
		// After reading everything from the buffer, proceed to parse the data
		tclacClimate::readData();
	}
}

void tclacClimate::update() {
	
	tclacClimate::dataShow(1,1);
	//Serial.write(poll, sizeof(poll));
	this->esphome::uart::UARTDevice::write_array(poll, sizeof(poll));
	auto raw = tclacClimate::getHex(poll, sizeof(poll));
	ESP_LOGD("TCL", "chek status sended");
	tclacClimate::dataShow(1,0);
}

void tclacClimate::readData() {
	
	current_temperature = float((( (dataRX[17] << 8) | dataRX[18] ) / 374 - 32)/1.8);
	target_temperature = (dataRX[FAN_SPEED_POS] & SET_TEMP_MASK) + 16;

	ESP_LOGD("TCL", "TEMP: %f ", current_temperature);

	if (dataRX[MODE_POS] & ( 1 << 4)) {
		// If the air conditioner is turned on, then parse the data for display
		uint8_t modeswitch = MODE_MASK & dataRX[MODE_POS];
		uint8_t fanspeedswitch = FAN_SPEED_MASK & dataRX[FAN_SPEED_POS];
		uint8_t swingmodeswitch = SWING_MODE_MASK & dataRX[SWING_POS];

		switch (modeswitch) {
			case MODE_AUTO:
				mode = climate::CLIMATE_MODE_AUTO;
				break;
			case MODE_COOL:
				mode = climate::CLIMATE_MODE_COOL;
				break;
			case MODE_DRY:
				mode = climate::CLIMATE_MODE_DRY;
				break;
			case MODE_FAN_ONLY:
				mode = climate::CLIMATE_MODE_FAN_ONLY;
				break;
			case MODE_HEAT:
				mode = climate::CLIMATE_MODE_HEAT;
				break;
			default:
				mode = climate::CLIMATE_MODE_AUTO;
		}

		if ( dataRX[FAN_QUIET_POS] & FAN_QUIET) {
			fan_mode = climate::CLIMATE_FAN_QUIET;
		} else if (dataRX[MODE_POS] & FAN_DIFFUSE){
			fan_mode = climate::CLIMATE_FAN_DIFFUSE;
		} else {
			switch (fanspeedswitch) {
				case FAN_AUTO:
					fan_mode = climate::CLIMATE_FAN_AUTO;
					break;
				case FAN_LOW:
					fan_mode = climate::CLIMATE_FAN_LOW;
					break;
				case FAN_MIDDLE:
					fan_mode = climate::CLIMATE_FAN_MIDDLE;
					break;
				case FAN_MEDIUM:
					fan_mode = climate::CLIMATE_FAN_MEDIUM;
					break;
				case FAN_HIGH:
					fan_mode = climate::CLIMATE_FAN_HIGH;
					break;
				case FAN_FOCUS:
					fan_mode = climate::CLIMATE_FAN_FOCUS;
					break;
				default:
					fan_mode = climate::CLIMATE_FAN_AUTO;
			}
		}

		switch (swingmodeswitch) {
			case SWING_OFF: 
				swing_mode = climate::CLIMATE_SWING_OFF;
				break;
			case SWING_HORIZONTAL:
				swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
				break;
			case SWING_VERTICAL:
				swing_mode = climate::CLIMATE_SWING_VERTICAL;
				break;
			case SWING_BOTH:
				swing_mode = climate::CLIMATE_SWING_BOTH;
				break;
		}
	} else {
		// If the air conditioner is turned off, then all modes are displayed as off
		mode = climate::CLIMATE_MODE_OFF;
		fan_mode = climate::CLIMATE_FAN_AUTO;
		swing_mode = climate::CLIMATE_SWING_OFF;
	}
	// Publish the data
	this->publish_state();
    }

// Climate control
void tclacClimate::control(const ClimateCall &call) {
	
	uint8_t switchvar = 0;
	
	dataTX[7]  = 0b00000000;//eco,display,beep,ontimerenable, offtimerenable,power,0,0
	dataTX[8]  = 0b00000000;//mute,0,turbo,health,mode(4)  0=cool 1=fan  2=dry 3=heat 4=auto 
	dataTX[9]  = 0b00000000;//[9] = 0,0,0,0,temp(4) 31 - value
	dataTX[10] = 0b00000000;//[10] = 0,timerindicator,swingv(3),fan(3) 0=auto 1=low 2=med 3=high
							//																{0,2,3,5,0};
	dataTX[11] = 0b00000000;
	dataTX[32] = 0b00000000;
	dataTX[33] = 0b00000000;
	
	if (call.get_mode().has_value()){
		switchvar = call.get_mode().value();
	} else {
		switchvar = mode;
	}

	// Turn the buzzer on or off depending on the switch setting
	if (beeper_status_){
		ESP_LOGD("TCL", "Beep mode ON");
		dataTX[7] += 0b00100000;
	} else {
		ESP_LOGD("TCL", "Beep mode OFF");
		dataTX[7] += 0b00000000;
	}
	
	// Turn the air conditioner display on or off depending on the switch setting
	// Turn on the display only if the air conditioner is in one of the operational modes
	
	// CAUTION! When turning off the display, the air conditioner forcibly switches to automatic mode!
	
	if (display_status_ && switchvar != climate::CLIMATE_MODE_OFF) {
    ESP_LOGD("TCL", "Display turn ON");
    // Turn on the display
    dataTX[7] |= 0b01000000;
} else {
    ESP_LOGD("TCL", "Display turn OFF");
    // Turn off the display
    dataTX[7] &= ~0b01000000;
}
		
	// Configure the operation mode of the air conditioner
	switch (switchvar) {
		case climate::CLIMATE_MODE_OFF:
			dataTX[7] += 0b00000000;
			dataTX[8] += 0b00000000;
			break;
		case climate::CLIMATE_MODE_AUTO:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00001000;
			break;
		case climate::CLIMATE_MODE_COOL:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000011;	
			break;
		case climate::CLIMATE_MODE_DRY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000010;	
			break;
		case climate::CLIMATE_MODE_FAN_ONLY:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000111;	
			break;
		case climate::CLIMATE_MODE_HEAT:
			dataTX[7] += 0b00000100;
			dataTX[8] += 0b00000001;	
			break;
	}

	// Configure the operation mode of the fan
	if (call.get_fan_mode().has_value()){
		switchvar = call.get_fan_mode().value();
		switch(switchvar) {
			case climate::CLIMATE_FAN_AUTO:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000000;
				break;
			case climate::CLIMATE_FAN_QUIET:
				dataTX[8]	+= 0b10000000;
				dataTX[10]	+= 0b00000000;
				break;
			case climate::CLIMATE_FAN_LOW:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000001;
				break;
			case climate::CLIMATE_FAN_MIDDLE:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000110;
				break;
			case climate::CLIMATE_FAN_MEDIUM:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000011;
				break;
			case climate::CLIMATE_FAN_HIGH:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000111;
				break;
			case climate::CLIMATE_FAN_FOCUS:
				dataTX[8]	+= 0b00000000;
				dataTX[10]	+= 0b00000101;
				break;
			case climate::CLIMATE_FAN_DIFFUSE:
				dataTX[8]	+= 0b01000000;
				dataTX[10]	+= 0b00000000;
				break;
		}		
	} else {
		if(fan_mode == climate::CLIMATE_FAN_AUTO){
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000000;
		} else if(fan_mode == climate::CLIMATE_FAN_QUIET){
			dataTX[8]	+= 0b10000000;
			dataTX[10]	+= 0b00000000;
		} else if(fan_mode == climate::CLIMATE_FAN_LOW){
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000001;
		} else if(fan_mode == climate::CLIMATE_FAN_MIDDLE){
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000110;
		} else if(fan_mode == climate::CLIMATE_FAN_MEDIUM){
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000011;
		} else if(fan_mode == climate::CLIMATE_FAN_HIGH){
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000111;
		} else if(fan_mode == climate::CLIMATE_FAN_FOCUS){
			dataTX[8]	+= 0b00000000;
			dataTX[10]	+= 0b00000101;
		} else if(fan_mode == climate::CLIMATE_FAN_DIFFUSE){
			dataTX[8]	+= 0b01000000;
			dataTX[10]	+= 0b00000000;
		}
	}

// Louver Mode
//  Vertical Louver
//      Vertical Louver Swing [10 bytes, mask 00111000]:
//          000 - Swing disabled, louver in last position or in fixation
//        111 - Swing enabled in the selected mode
//       Vertical Louver Swing Mode (louver fixation mode doesn't matter if swing is enabled) [32 bytes, mask 00011000]:
//           01 - swing from top to bottom, DEFAULT
//           10 - swing in the upper half
//           11 - swing in the lower half
//       Louver Fixation Mode (louver swing mode doesn't matter if swing is disabled) [32 bytes, mask 00000111]:
//           000 - no fixation, DEFAULT
//           001 - fixation at the top
//           010 - fixation between top and middle
//           011 - fixation at the middle
//           100 - fixation between middle and bottom
//           101 - fixation at the bottom
//   Horizontal Louvers
//       Horizontal Louver Swing [11 bytes, mask 00001000]:
//           0 - Swing disabled, louvers in last position or in fixation
//           1 - Swing enabled in the selected mode
//       Horizontal Louver Swing Mode (louver fixation mode doesn't matter if swing is enabled) [33 bytes, mask 00111000]:
//           001 - swing from left to right, DEFAULT
//           010 - swing from left
//           011 - swing in the middle
//           100 - swing from right
//       Horizontal Louver Fixation Mode (louver swing mode doesn't matter if swing is disabled) [33 bytes, mask 00000111]:
//           000 - no fixation, DEFAULT
//           001 - fixation on the left
//           010 - fixation between left and middle
//           011 - fixation in the middle
//           100 - fixation between middle and right
//           101 - fixation on the right


// Request data from the louvers swing mode switch
	if (call.get_swing_mode().has_value()){
		switchvar = call.get_swing_mode().value();
	} else {
		// If the switch is empty, fill it with the value from the last state polling. It's like nothing has changed.
		switchvar = swing_mode;
	}
	// Set the vent swing mode
	switch(switchvar) {
		case climate::CLIMATE_SWING_OFF:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_VERTICAL:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00000000;
			break;
		case climate::CLIMATE_SWING_HORIZONTAL:
			dataTX[10]	+= 0b00000000;
			dataTX[11]	+= 0b00001000;
			break;
		case climate::CLIMATE_SWING_BOTH:
			dataTX[10]	+= 0b00111000;
			dataTX[11]	+= 0b00001000;  
			break;
	}
	// Set the mode for vertical vent swing
	switch(vertical_swing_direction_) {
		case VerticalSwingDirection::UP_DOWN:
			dataTX[32]	+= 0b00001000;
			ESP_LOGD("TCL", "Vertical swing: up-down");
			break;
		case VerticalSwingDirection::UPSIDE:
			dataTX[32]	+= 0b00010000;
			ESP_LOGD("TCL", "Vertical swing: upper");
			break;
		case VerticalSwingDirection::DOWNSIDE:
			dataTX[32]	+= 0b00011000;
			ESP_LOGD("TCL", "Vertical swing: downer");
			break;
	}
	// Set the mode for vertical vent swing
	switch(horizontal_swing_direction_) {
		case HorizontalSwingDirection::LEFT_RIGHT:
			dataTX[33]	+= 0b00001000;
			ESP_LOGD("TCL", "Horizontal swing: left-right");
			break;
		case HorizontalSwingDirection::LEFTSIDE:
			dataTX[33]	+= 0b00010000;
			ESP_LOGD("TCL", "Horizontal swing: lefter");
			break;
		case HorizontalSwingDirection::CENTER:
			dataTX[33]	+= 0b00011000;
			ESP_LOGD("TCL", "Horizontal swing: center");
			break;
		case HorizontalSwingDirection::RIGHTSIDE:
			dataTX[33]	+= 0b00100000;
			ESP_LOGD("TCL", "Horizontal swing: righter");
			break;
	}
	// Set the position fixation of the vertical vent
	switch(this->vertical_direction_) {
		case AirflowVerticalDirection::LAST:
			dataTX[32]	+= 0b00000000;
			ESP_LOGD("TCL", "Vertical fix: last position");
			break;
		case AirflowVerticalDirection::MAX_UP:
			dataTX[32]	+= 0b00000001;
			ESP_LOGD("TCL", "Vertical fix: up");
			break;
		case AirflowVerticalDirection::UP:
			dataTX[32]	+= 0b00000010;
			ESP_LOGD("TCL", "Vertical fix: upper");
			break;
		case AirflowVerticalDirection::CENTER:
			dataTX[32]	+= 0b00000011;
			ESP_LOGD("TCL", "Vertical fix: center");
			break;
		case AirflowVerticalDirection::DOWN:
			dataTX[32]	+= 0b00000100;
			ESP_LOGD("TCL", "Vertical fix: downer");
			break;
		case AirflowVerticalDirection::MAX_DOWN:
			dataTX[32]	+= 0b00000101;
			ESP_LOGD("TCL", "Vertical fix: down");
			break;
	}
	// Set the position fixation of the horizontal vents
	switch(this->horizontal_direction_) {
		case AirflowHorizontalDirection::LAST:
			dataTX[33]	+= 0b00000000;
			ESP_LOGD("TCL", "Horizontal fix: last position");
			break;
		case AirflowHorizontalDirection::MAX_LEFT:
			dataTX[33]	+= 0b00000001;
			ESP_LOGD("TCL", "Horizontal fix: left");
			break;
		case AirflowHorizontalDirection::LEFT:
			dataTX[33]	+= 0b00000010;
			ESP_LOGD("TCL", "Horizontal fix: lefter");
			break;
		case AirflowHorizontalDirection::CENTER:
			dataTX[33]	+= 0b00000011;
			ESP_LOGD("TCL", "Horizontal fix: center");
			break;
		case AirflowHorizontalDirection::RIGHT:
			dataTX[33]	+= 0b00000100;
			ESP_LOGD("TCL", "Horizontal fix: righter");
			break;
		case AirflowHorizontalDirection::MAX_RIGHT:
			dataTX[33]	+= 0b00000101;
			ESP_LOGD("TCL", "Horizontal fix: right");
			break;
	}

	// Calculation and setting of the temperature
	if (call.get_target_temperature().has_value()) {
		dataTX[9] = 31-(int)call.get_target_temperature().value();		//0,0,0,0, temp(4)
	} else {
		dataTX[9] = 31-(int)target_temperature;
	}

	// Constructing an array of bytes for transmission to the air conditioner
	dataTX[0] = 0xBB;	//Start byte of the header
	dataTX[1] = 0x00;	//Start byte of the header
	dataTX[2] = 0x01;	//Start byte of the header
	dataTX[3] = 0x03;	//0x03 - control, 0x04 - polling
	dataTX[4] = 0x20;	//0x20 - control, 0x19 - polling
	dataTX[5] = 0x03;	//??
	dataTX[6] = 0x01;	//??
	//dataTX[7] = 0x64;	//eco,display,beep,ontimerenable, offtimerenable,power,0,0
	//dataTX[8] = 0x08;	//mute,0,turbo,health, mode(4) mode 01 heat, 02 dry, 03 cool, 07 fan, 08 auto, health(+16), 41=turbo-heat 43=turbo-cool (turbo = 0x40+ 0x01..0x08)
	//dataTX[9] = 0x0f;	//0 -31 ;    15 - 16 0,0,0,0, temp(4) settemp 31 - x
	//dataTX[10] = 0x00;	//0,timerindicator,swingv(3),fan(3) fan+swing modes //0=auto 1=low 2=med 3=high
	//dataTX[11] = 0x00;	//0,offtimer(6),0
	dataTX[12] = 0x00;	//fahrenheit,ontimer(6),0 cf 80=f 0=c
	dataTX[13] = 0x01;	//??
	dataTX[14] = 0x00;	//0,0,halfdegree,0,0,0,0,0
	dataTX[15] = 0x00;	//??
	dataTX[16] = 0x00;	//??
	dataTX[17] = 0x00;	//??
	dataTX[18] = 0x00;	//??
	dataTX[19] = 0x00;	//sleep on = 1 off=0
	dataTX[20] = 0x00;	//??
	dataTX[21] = 0x00;	//??
	dataTX[22] = 0x00;	//??
	dataTX[23] = 0x00;	//??
	dataTX[24] = 0x00;	//??
	dataTX[25] = 0x00;	//??
	dataTX[26] = 0x00;	//??
	dataTX[27] = 0x00;	//??
	dataTX[28] = 0x00;	//??
	dataTX[30] = 0x00;	//??
	dataTX[31] = 0x00;	//??
	//dataTX[32] = 0x00;	//0,0,0,vertical swing mode(2), vertical fixation mode(3)
	//dataTX[33] = 0x00;	//0,0, horizontal swing mode(3), horizontal fixation mode(3)
	dataTX[34] = 0x00;	//??
	dataTX[35] = 0x00;	//??
	dataTX[36] = 0x00;	//??
	dataTX[37] = 0xFF;	//Checksum
	dataTX[37] = tclacClimate::getChecksum(dataTX, sizeof(dataTX));

	tclacClimate::sendData(dataTX, sizeof(dataTX));
}
// Sending data to the air conditioner
void tclacClimate::sendData(byte * message, byte size) {
	tclacClimate::dataShow(1,1);
	//Serial.write(message, size);
	this->esphome::uart::UARTDevice::write_array(message, size);
	auto raw = getHex(message, size);
	ESP_LOGD("TCL", "Message to TCL sended...");
	tclacClimate::dataShow(1,0);
}
// Byte conversion to readable format
String tclacClimate::getHex(byte *message, byte size) {
	String raw;
	for (int i = 0; i < size; i++) {
		raw += "\n" + String(message[i]);
	}
	raw.toUpperCase();
	return raw;
}
// Checksum calculation
byte tclacClimate::getChecksum(const byte * message, size_t size) {
	byte position = size - 1;
	byte crc = 0;
	for (int i = 0; i < position; i++)
		crc ^= message[i];
	return crc;
}
// Flashing LEDs
void tclacClimate::dataShow(bool flow, bool shine) {
	if (module_display_status_){
		if (flow == 0){
			if (shine == 1){
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_RX_LED
				this->rx_led_pin_->digital_write(false);
#endif
			}
		}
		if (flow == 1) {
			if (shine == 1){
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(true);
#endif
			} else {
#ifdef CONF_TX_LED
				this->tx_led_pin_->digital_write(false);
#endif
			}
		}
	}
}

// Actions with data from the configuration

// Obtaining the buzzer state
void tclacClimate::set_beeper_state(bool state) {
	this->beeper_status_ = state;
}
// Obtaining the air conditioner display state
void tclacClimate::set_display_state(bool state) {
	this->display_status_ = state;
}
// Obtaining the data reception LED pin
#ifdef CONF_RX_LED
void tclacClimate::set_rx_led_pin(GPIOPin *rx_led_pin) {
	this->rx_led_pin_ = rx_led_pin;
}
#endif
// Obtaining the data transmission LED pin
#ifdef CONF_TX_LED
void tclacClimate::set_tx_led_pin(GPIOPin *tx_led_pin) {
	this->tx_led_pin_ = tx_led_pin;
}
#endif
// Obtaining the status of the module communication LEDs
void tclacClimate::set_module_display_state(bool state) {
	this->module_display_status_ = state;
}
// Obtaining the vertical vent fixation mode
void tclacClimate::set_vertical_airflow(AirflowVerticalDirection direction) {
	this->vertical_direction_ = direction;
}
// Obtaining the horizontal vent fixation mode
void tclacClimate::set_horizontal_airflow(AirflowHorizontalDirection direction) {
	this->horizontal_direction_ = direction;
}
// Obtaining the vertical vent swing mode
void tclacClimate::set_vertical_swing_direction(VerticalSwingDirection direction) {
	this->vertical_swing_direction_ = direction;
}
// Obtaining the available operation modes of the air conditioner
void tclacClimate::set_supported_modes(const std::set<climate::ClimateMode> &modes) {
	this->supported_modes_ = modes;
}
// Obtaining the horizontal vent swing mode
void tclacClimate::set_horizontal_swing_direction(HorizontalSwingDirection direction) {
	this->horizontal_swing_direction_ = direction;
}
// Obtaining the available fan speeds
void tclacClimate::set_supported_fan_modes(const std::set<climate::ClimateFanMode> &modes){
	this->supported_fan_modes_ = modes;
}
// Obtaining the available vent swing modes
void tclacClimate::set_supported_swing_modes(const std::set<climate::ClimateSwingMode> &modes) {
	this->supported_swing_modes_ = modes;
}

// Function templates for querying state, might be useful in the future if feedback is implemented. Really don't want to, it will be very cumbersome.

//bool tclacClimate::get_beeper_state() const { return this->beeper_status_; }
//bool tclacClimate::get_display_state() const { return this->display_status_; }
//bool tclacClimate::get_module_display_state() const { return this->module_display_status_; }
//AirflowVerticalDirection tclacClimate::get_vertical_airflow() const { return this->vertical_direction_; };
//AirflowHorizontalDirection tclacClimate::get_horizontal_airflow() const { return this->horizontal_direction_; }
//VerticalSwingDirection tclacClimate::get_vertical_swing_direction() const { return this->vertical_swing_direction_; }
//HorizontalSwingDirection tclacClimate::get_horizontal_swing_direction() const { return this->horizontal_swing_direction_; }



}
}
