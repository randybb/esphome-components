import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_TYPE

from . import CONF_CREALITY_CFS_ID, CREALITY_CFS_CHILD_SCHEMA

# type: (offset, length, is the color) in the record, as DnG-Crafts/K2-RFID lays it out
TYPES = {
    "date": (0, 5, False),  # month (1 hex digit), day, year
    "vendor": (5, 4, False),
    "batch": (9, 2, False),
    "material_id": (11, 6, False),  # the printer's material database ID
    "color": (17, 7, True),  # 0RRGGBB, published as #RRGGBB
    "serial": (28, 6, False),
}

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema()
    .extend(CREALITY_CFS_CHILD_SCHEMA)
    .extend({cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True)})
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CREALITY_CFS_ID])
    var = await text_sensor.new_text_sensor(config)
    cg.add(parent.add_text_sensor(var, *TYPES[config[CONF_TYPE]]))
