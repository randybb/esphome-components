import esphome.codegen as cg
from esphome.components import text_sensor

from . import CONF_PN5180_ID, PN5180_CHILD_SCHEMA

# The UID of the tag on the reader, empty without one
CONFIG_SCHEMA = text_sensor.text_sensor_schema(icon="mdi:nfc-variant").extend(PN5180_CHILD_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_PN5180_ID])
    cg.add(parent.set_uid_text_sensor(await text_sensor.new_text_sensor(config)))
