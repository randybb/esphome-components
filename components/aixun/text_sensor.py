import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID

CONF_DEVICE_MODEL = "device_model"
CONF_FIRMWARE_VERSION = "firmware_version"
# H310D only (ESPHome on the station's ESP32-C3, binary link to the GD32 MCU):
CONF_SERIAL_NUMBER = "serial_number"
CONF_HEATER_STATUS = "heater_status"  # H310D: Heating / Cooling / Standby

_DIAG = {"entity_category": ENTITY_CATEGORY_DIAGNOSTIC}

CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_DEVICE_MODEL): text_sensor.text_sensor_schema(
            **_DIAG, icon="mdi:information-outline"
        ),
        cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
            **_DIAG, icon="mdi:chip"
        ),
        cv.Optional(CONF_SERIAL_NUMBER): text_sensor.text_sensor_schema(
            **_DIAG, icon="mdi:barcode"
        ),
        cv.Optional(CONF_HEATER_STATUS): text_sensor.text_sensor_schema(icon="mdi:heat-wave"),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    for key, setter in (
        (CONF_DEVICE_MODEL, hub.set_device_model_text_sensor),
        (CONF_FIRMWARE_VERSION, hub.set_firmware_version_text_sensor),
        (CONF_SERIAL_NUMBER, hub.set_serial_number_text_sensor),
        (CONF_HEATER_STATUS, hub.set_heater_status_text_sensor),
    ):
        if conf := config.get(key):
            cg.add(setter(await text_sensor.new_text_sensor(conf)))
