#ifndef TCL_ESP_TCL_H
#define TCL_ESP_TCL_H

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"

namespace esphome {
namespace tclac {

// --- MACROS ---
#define SET_TEMP_MASK 0x0F
#define MODE_POS 7
#define MODE_MASK 0x3F

#define MODE_AUTO    0x35
#define MODE_COOL    0x31
#define MODE_DRY     0x33
#define MODE_FAN_ONLY 0x32
#define MODE_HEAT    0x34

#define FAN_SPEED_MASK 0xF0
#define FAN_AUTO       0x80
#define FAN_QUIET      0x80
#define FAN_LOW        0x90
#define FAN_MIDDLE     0xC0
#define FAN_MEDIUM     0xA0
#define FAN_HIGH       0xD0
#define FAN_FOCUS      0xB0
#define FAN_DIFFUSE    0x80

#define SWING_MODE_MASK 0x60
#define SWING_OFF      0x00
#define SWING_HORIZONTAL 0x20
#define SWING_VERTICAL   0x40
#define SWING_BOTH       0x60

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

// --- DECLARAÇÃO DA CLASSE ---
class tclacClimate : public climate::Climate, public esphome::uart::UARTDevice, public PollingComponent {
public:
    tclacClimate();

    void readData();
    void takeControl();
    void loop() override;
    void setup() override;
    void update() override;

    void set_beeper_state(bool state);
    void set_display_state(bool state);
    void set_force_mode_state(bool state);
    void set_module_display_state(bool state);
    void set_rx_led_pin(GPIOPin *rx_led_pin);
    void set_tx_led_pin(GPIOPin *tx_led_pin);

    void sendData(byte *message, byte size);
    static String getHex(byte *message, byte size);
    static byte getChecksum(const byte *message, size_t size);

    void control(const ClimateCall &call) override;

    void set_vertical_airflow(AirflowVerticalDirection direction);
    void set_horizontal_airflow(AirflowHorizontalDirection direction);
    void set_vertical_swing_direction(VerticalSwingDirection direction);
    void set_horizontal_swing_direction(HorizontalSwingDirection direction);

    void set_supported_presets(const std::set<ClimatePreset> &presets);
    void set_supported_modes(const std::set<ClimateMode> &modes);
    void set_supported_fan_modes(const std::set<ClimateFanMode> &modes);
    void set_supported_swing_modes(const std::set<ClimateSwingMode> &modes);

protected:
    byte checksum;
    byte dataTX[38];
    byte dataRX[61];
    byte poll[8];

    bool beeper_status_;
    bool display_status_;
    bool force_mode_status_;
    bool module_display_status_;
    int target_temperature_set;
    uint8_t switch_preset;
    uint8_t switch_fan_mode;
    uint8_t switch_swing_mode;
    uint8_t switch_climate_mode;
    bool allow_take_control;

    ClimateTraits traits_;
    GPIOPin *rx_led_pin_;
    GPIOPin *tx_led_pin_;

    std::set<ClimateMode> supported_modes_;
    std::set<ClimatePreset> supported_presets_;
    std::set<ClimateFanMode> supported_fan_modes_;
    std::set<ClimateSwingMode> supported_swing_modes_;

    VerticalSwingDirection vertical_swing_direction_;
    HorizontalSwingDirection horizontal_swing_direction_;
    AirflowVerticalDirection vertical_direction_;
    AirflowHorizontalDirection horizontal_direction_;
};

}  // namespace tclac
}  // namespace esphome

#endif  // TCL_ESP_TCL_H
