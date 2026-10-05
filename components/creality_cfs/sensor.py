import esphome.codegen as cg
from esphome.components import sensor
from esphome.const import UNIT_METER

from . import CONF_CREALITY_CFS_ID, CREALITY_CFS_CHILD_SCHEMA

# The filament length the tag tells
CONFIG_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER, icon="mdi:printer-3d-nozzle", accuracy_decimals=0
).extend(CREALITY_CFS_CHILD_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_CREALITY_CFS_ID])
    cg.add(parent.set_length_sensor(await sensor.new_sensor(config)))
