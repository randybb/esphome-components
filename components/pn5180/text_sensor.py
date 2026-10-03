import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_TYPE

from . import CONF_PN5180_ID, PN5180_CHILD_SCHEMA, FieldSource

# type: (source, key); keys from the OpenPrintTag spec
TYPES = {
    "uid": ("SOURCE_UID", 0),
    "material_type": ("SOURCE_MAIN", 9),
    "material_name": ("SOURCE_MAIN", 10),
    "brand_name": ("SOURCE_MAIN", 11),
    "primary_color": ("SOURCE_MAIN", 19),
    "gtin": ("SOURCE_MAIN", 4),
    "storage_location": ("SOURCE_AUX", 4),
}

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema()
    .extend(PN5180_CHILD_SCHEMA)
    .extend({cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True)})
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_PN5180_ID])
    var = await text_sensor.new_text_sensor(config)
    source, key = TYPES[config[CONF_TYPE]]
    cg.add(parent.add_text_sensor(var, getattr(FieldSource, source), key))
