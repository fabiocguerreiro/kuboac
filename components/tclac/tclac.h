#ifndef TCL_ESP_TCL_H
#define TCL_ESP_TCL_H

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"

namespace esphome {
namespace tclac {

// --- MACROS ---
#define SET_TEMP_MASK	0b00001111
#define MODE_POS		7
#define MODE_MASK		0b00111111

#define MODE_AUTO		0b00110101
#define MODE_COOL		0b00110001
#define MODE_DRY		0b00110011
#define MODE_FAN_ONLY	0b00110010
#define MODE_HEAT		0b00110100

#define FAN_SPEED_MASK	0b11110000
#define FAN_AUTO		0b10000000
#define FAN_QUIET		0x80
#define FAN_LOW			0b10010000
#define FAN_MIDDLE		0b11000000
#define FAN_MEDIUM		0b10100000
#define FAN_HIGH		0b11010000
#define FAN_FOCUS		0b10110000
#define FAN_DIFFUSE		0b10000000

#define SWING_MODE_MASK	0b01100000
#define SWING_OFF			0b00000000
#define SWING_HORIZONTAL	0b00100000
#define SWING_VERTICAL		0b01000000
#define SWING_BOTH			0b01100000

using climate::ClimateCall;
using climate::ClimateMode;
using climate::ClimatePreset;
using climate::ClimateTraits;
using climate::ClimateFanMode;
using climate::ClimateSwingMode;

// --- ENUMS ---
enum class VerticalSwingDirection : uint8_t { UP_DOWN=0, UPSIDE=1, DOWNSIDE=2 };
enum class HorizontalSwingDirection : uint8_t { LEFT_RIGHT=0, LEFTSIDE=1, CENTER=2, RIGHTSIDE=3 };
enum class AirflowVerticalDirection : uint8_t { LAST=0, MAX_UP=1, UP=2, CENTER=3, DOWN=4, MAX_DOWN=5 };
enum class AirflowHorizontalDirection : uint8_t { LAST=0, MAX_LEFT=1, LEFT=2, CENTER=3, RIGHT=4, MAX_RIGHT=5 };

// --- CLASSE PRINCIPAL ---
class tclacClimate : public climate::Climate, public esphome::uart::UARTDevice, public PollingComponent {
public:
    tclacClimate();

    void setup() override;
    void loop() override;
    void update() override;
    void control(const ClimateCall &call) override;

    void set_beeper_state(bool state);
    void set_display_state(bool state);
    void set_force_mode_state(bool state);
    void set_module_display_state(bool state);

    void set_vertical_airflow(AirflowVerticalDirection direction);
    void set_vertical_swing_direction(VerticalSwingDirection direction);

    void set_supported_presets(const std::set<ClimatePreset> &presets);
    void set_supported_modes(const std::set<ClimateMode> &modes);
    void set_supported_fan_modes(const std::set<ClimateFanMode> &modes);
    void set_supported_swing_modes(const std::set<ClimateSwingMode> &modes);

protected:
    ClimateTraits traits() override;

private:
    byte checksum_;
    byte dataTX_[38];
    byte dataRX_[61];
    byte poll_[8];

    bool beeper_status_;
    bool display_status_;
    bool force_mode_status_;
    bool module_display_status_;
    AirflowVerticalDirection vertical_direction_;
    VerticalSwingDirection vertical_swing_direction_;

    GPIOPin *rx_led_pin_;
    GPIOPin *tx_led_pin_;
};

}  // namespace tclac
}  // namespace esphome

#endif  // TCL_ESP_TCL_H
