import esphome.codegen as cg
from esphome.components import output
import esphome.config_validation as cv
from esphome.const import CONF_CHANNEL, CONF_ID
from esphome.types import ConfigType

from . import P9813

DEPENDENCIES = ["p9813"]

Channel = P9813.class_("Channel", output.FloatOutput)

CONF_P9813_ID = "p9813_id"
CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_P9813_ID): cv.use_id(P9813),
        cv.Required(CONF_ID): cv.declare_id(Channel),
        # chip * 3 + 0 (R), 1 (G), 2 (B)
        cv.Required(CONF_CHANNEL): cv.int_range(min=0, max=254),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await output.register_output(var, config)

    parent = await cg.get_variable(config[CONF_P9813_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_channel(config[CONF_CHANNEL]))
