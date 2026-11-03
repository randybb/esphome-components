import esphome.codegen as cg
from esphome.components import display, i2c
import esphome.config_validation as cv
from esphome.const import CONF_BRIGHTNESS, CONF_ID, CONF_LAMBDA, CONF_PAGES
from esphome.types import ConfigType

from . import m5stack_st7789v2_ns

DEPENDENCIES = ["i2c"]

M5StackST7789V2 = m5stack_st7789v2_ns.class_(
    "M5StackST7789V2", display.DisplayBuffer, i2c.I2CDevice
)

CONFIG_SCHEMA = cv.All(
    display.FULL_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(M5StackST7789V2),
            cv.Optional(CONF_BRIGHTNESS, default="100%"): cv.percentage,
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(i2c.i2c_device_schema(0x3E)),
    cv.has_at_most_one_key(CONF_PAGES, CONF_LAMBDA),
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await display.register_display(var, config)
    await i2c.register_i2c_device(var, config)
    await display.add_writer(var, config)
    cg.add(var.set_brightness(config[CONF_BRIGHTNESS]))
