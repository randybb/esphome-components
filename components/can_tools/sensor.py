import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import CONF_LAMBDA
from esphome.types import ConfigType

from . import CAN_TOOLS_CHILD_SCHEMA, CONF_REQUEST_ID, DATA_ARGS

CONFIG_SCHEMA = sensor.sensor_schema().extend(CAN_TOOLS_CHILD_SCHEMA).extend(
    {cv.Required(CONF_LAMBDA): cv.returning_lambda}
)


async def to_code(config: ConfigType) -> None:
    req = await cg.get_variable(config[CONF_REQUEST_ID])
    var = await sensor.new_sensor(config)
    decode = await cg.process_lambda(
        config[CONF_LAMBDA], DATA_ARGS, return_type=cg.optional.template(cg.float_)
    )
    cg.add(req.add_sensor(var, decode))
