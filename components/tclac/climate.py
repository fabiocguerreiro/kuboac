from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, uart
from esphome.const import CONF_ID

tclac_ns = cg.esphome_ns.namespace("tclac")
tclacClimate = tclac_ns.class_("tclacClimate", uart.UARTDevice, climate.Climate, cg.PollingComponent)

# Actions
DisplayOnAction = tclac_ns.class_("DisplayOnAction", automation.Action)
DisplayOffAction = tclac_ns.class_("DisplayOffAction", automation.Action)
BeeperOnAction = tclac_ns.class_("BeeperOnAction", automation.Action)
BeeperOffAction = tclac_ns.class_("BeeperOffAction", automation.Action)
VerticalAirflowAction = tclac_ns.class_("VerticalAirflowAction", automation.Action)
VerticalSwingDirectionAction = tclac_ns.class_("VerticalSwingDirectionAction", automation.Action)
