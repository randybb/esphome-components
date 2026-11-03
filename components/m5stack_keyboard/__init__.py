import esphome.codegen as cg
from esphome.components import i2c, key_provider
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["key_provider"]
MULTI_CONF = True

m5stack_keyboard_ns = cg.esphome_ns.namespace("m5stack_keyboard")
M5StackKeyboard = m5stack_keyboard_ns.class_(
    "M5StackKeyboard", cg.PollingComponent, i2c.I2CDevice, key_provider.KeyProvider
)

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(M5StackKeyboard)})
    .extend(cv.polling_component_schema("20ms"))
    .extend(i2c.i2c_device_schema(0x08))  # Faces QWERTY; CardKB is 0x5F
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
