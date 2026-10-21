import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID

CONF_DEVICE_MODEL = "device_model"
CONF_FIRMWARE_VERSION = "firmware_version"

CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_DEVICE_MODEL): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:information-outline"
        ),
        cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:chip"
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    text_sensors = text_sensor.sub_text_sensors(config)
    await text_sensors(CONF_DEVICE_MODEL, hub.set_device_model_text_sensor)
    await text_sensors(CONF_FIRMWARE_VERSION, hub.set_firmware_version_text_sensor)
