from esphome import automation, pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart
from esphome.const import (
    CONF_ID, CONF_BEEPER, CONF_VISUAL, CONF_MAX_TEMPERATURE, CONF_MIN_TEMPERATURE,
    CONF_SUPPORTED_MODES, CONF_TEMPERATURE_STEP, CONF_SUPPORTED_PRESETS,
    CONF_TARGET_TEMPERATURE, CONF_SUPPORTED_FAN_MODES, CONF_SUPPORTED_SWING_MODES,
)

from esphome.components.climate import ClimateMode, ClimatePreset, ClimateSwingMode, CONF_CURRENT_TEMPERATURE

AUTO_LOAD = ["climate"]
DEPENDENCIES = ["climate", "uart"]

TCLAC_MIN_TEMPERATURE = 16.0
TCLAC_MAX_TEMPERATURE = 31.0
TCLAC_TARGET_TEMPERATURE_STEP = 1.0
TCLAC_CURRENT_TEMPERATURE_STEP = 1.0

CONF_DISPLAY = "show_display"
CONF_FORCE_MODE = "force_mode"
CONF_VERTICAL_AIRFLOW = "vertical_airflow"
CONF_MODULE_DISPLAY = "show_module_display"
CONF_VERTICAL_SWING_MODE = "vertical_swing_mode"

tclac_ns = cg.esphome_ns.namespace("tclac")
tclacClimate = tclac_ns.class_("tclacClimate", uart.UARTDevice, climate.Climate, cg.PollingComponent)

# Opções de enum
SUPPORTED_FAN_MODES_OPTIONS = {
    "AUTO": ClimateMode.CLIMATE_FAN_AUTO,
    "QUIET": ClimateMode.CLIMATE_FAN_QUIET,
    "LOW": ClimateMode.CLIMATE_FAN_LOW,
    "MIDDLE": ClimateMode.CLIMATE_FAN_MIDDLE,
    "MEDIUM": ClimateMode.CLIMATE_FAN_MEDIUM,
    "HIGH": ClimateMode.CLIMATE_FAN_HIGH,
    "FOCUS": ClimateMode.CLIMATE_FAN_FOCUS,
    "DIFFUSE": ClimateMode.CLIMATE_FAN_DIFFUSE,
}

SUPPORTED_SWING_MODES_OPTIONS = {
    "OFF": ClimateSwingMode.CLIMATE_SWING_OFF,
    "VERTICAL": ClimateSwingMode.CLIMATE_SWING_VERTICAL,
}

SUPPORTED_CLIMATE_MODES_OPTIONS = {
    "OFF": ClimateMode.CLIMATE_MODE_OFF,
    "AUTO": ClimateMode.CLIMATE_MODE_AUTO,
    "COOL": ClimateMode.CLIMATE_MODE_COOL,
    "HEAT": ClimateMode.CLIMATE_MODE_HEAT,
    "DRY": ClimateMode.CLIMATE_MODE_DRY,
    "FAN_ONLY": ClimateMode.CLIMATE_MODE_FAN_ONLY,
}

SUPPORTED_CLIMATE_PRESETS_OPTIONS = {
    "NONE": ClimatePreset.CLIMATE_PRESET_NONE,
    "ECO": ClimatePreset.CLIMATE_PRESET_ECO,
    "SLEEP": ClimatePreset.CLIMATE_PRESET_SLEEP,
}

VerticalSwingDirection = tclac_ns.enum("VerticalSwingDirection", True)
VERTICAL_SWING_DIRECTION_OPTIONS = {"UP_DOWN": VerticalSwingDirection.UP_DOWN, "UPSIDE": VerticalSwingDirection.UPSIDE, "DOWNSIDE": VerticalSwingDirection.DOWNSIDE}

AirflowVerticalDirection = tclac_ns.enum("AirflowVerticalDirection", True)
AIRFLOW_VERTICAL_DIRECTION_OPTIONS = {"LAST": AirflowVerticalDirection.LAST, "MAX_UP": AirflowVerticalDirection.MAX_UP, "UP": AirflowVerticalDirection.UP, "CENTER": AirflowVerticalDirection.CENTER, "DOWN": AirflowVerticalDirection.DOWN, "MAX_DOWN": AirflowVerticalDirection.MAX_DOWN}

# Validação visual
def validate_visual(config):
    if CONF_VISUAL not in config:
        config[CONF_VISUAL] = {}
    visual_config = config[CONF_VISUAL]
    visual_config.setdefault(CONF_MIN_TEMPERATURE, TCLAC_MIN_TEMPERATURE)
    visual_config.setdefault(CONF_MAX_TEMPERATURE, TCLAC_MAX_TEMPERATURE)
    visual_config.setdefault(CONF_TEMPERATURE_STEP, {CONF_TARGET_TEMPERATURE: TCLAC_TARGET_TEMPERATURE_STEP, CONF_CURRENT_TEMPERATURE: TCLAC_CURRENT_TEMPERATURE_STEP})
    return config

CONFIG_SCHEMA = cv.All(
    climate.climate_schema(tclacClimate)
    .extend({
        cv.Optional(CONF_BEEPER, default=True): cv.boolean,
        cv.Optional(CONF_DISPLAY, default=True): cv.boolean,
        cv.Optional(CONF_FORCE_MODE, default=True): cv.boolean,
        cv.Optional(CONF_MODULE_DISPLAY, default=True): cv.boolean,
        cv.Optional(CONF_VERTICAL_AIRFLOW, default="MAX_UP"): cv.ensure_list(cv.enum(AIRFLOW_VERTICAL_DIRECTION_OPTIONS, upper=True)),
        cv.Optional(CONF_VERTICAL_SWING_MODE, default="UP_DOWN"): cv.ensure_list(cv.enum(VERTICAL_SWING_DIRECTION_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_PRESETS, default=["NONE", "ECO", "SLEEP"]): cv.ensure_list(cv.enum(SUPPORTED_CLIMATE_PRESETS_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_SWING_MODES, default=["OFF", "VERTICAL"]): cv.ensure_list(cv.enum(SUPPORTED_SWING_MODES_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_MODES, default=["OFF","AUTO","COOL","HEAT","DRY","FAN_ONLY"]): cv.ensure_list(cv.enum(SUPPORTED_CLIMATE_MODES_OPTIONS, upper=True)),
        cv.Optional(CONF_SUPPORTED_FAN_MODES, default=["AUTO","QUIET","LOW","MIDDLE","MEDIUM","HIGH","FOCUS","DIFFUSE"]): cv.ensure_list(cv.enum(SUPPORTED_FAN_MODES_OPTIONS, upper=True)),
    })
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    validate_visual,
)
