import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID

# H310D: the panel's WiFi STA / AP mode toggles, reported by the MCU. Repurpose them as feature switches.
CONF_STA_MODE = "sta_mode"
CONF_AP_MODE = "ap_mode"

CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_STA_MODE): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_AP_MODE): binary_sensor.binary_sensor_schema(),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    if conf := config.get(CONF_STA_MODE):
        cg.add(hub.set_sta_binary_sensor(await binary_sensor.new_binary_sensor(conf)))
    if conf := config.get(CONF_AP_MODE):
        cg.add(hub.set_ap_binary_sensor(await binary_sensor.new_binary_sensor(conf)))
