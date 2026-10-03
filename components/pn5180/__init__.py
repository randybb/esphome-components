from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import spi
import esphome.config_validation as cv
from esphome.const import (
    CONF_BUSY_PIN,
    CONF_ID,
    CONF_INVERTED,
    CONF_IRQ_PIN,
    CONF_ON_TAG,
    CONF_ON_TAG_REMOVED,
    CONF_RESET_PIN,
)

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["spi"]
MULTI_CONF = True

CONF_PN5180_ID = "pn5180_id"

pn5180_ns = cg.esphome_ns.namespace("pn5180")
PN5180 = pn5180_ns.class_("PN5180", cg.PollingComponent, spi.SPIDevice)


def _irq_pin(value):
    value = pins.internal_gpio_input_pin_schema(value)
    if value.get(CONF_INVERTED):
        raise cv.Invalid("the IRQ polarity is read from the PN5180 EEPROM, 'inverted' is not supported")
    return value


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(PN5180),
            cv.Required(CONF_BUSY_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_IRQ_PIN): _irq_pin,
            cv.Optional(CONF_ON_TAG): automation.validate_automation({}),
            cv.Optional(CONF_ON_TAG_REMOVED): automation.validate_automation({}),
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(spi.spi_device_schema(cs_pin_required=True))
)

FINAL_VALIDATE_SCHEMA = spi.final_validate_device_schema(
    "pn5180", require_miso=True, require_mosi=True
)

PN5180_CHILD_SCHEMA = cv.Schema({cv.GenerateID(CONF_PN5180_ID): cv.use_id(PN5180)})


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)
    cg.add(var.set_busy_pin(await cg.gpio_pin_expression(config[CONF_BUSY_PIN])))
    if CONF_RESET_PIN in config:
        cg.add(var.set_reset_pin(await cg.gpio_pin_expression(config[CONF_RESET_PIN])))
    if CONF_IRQ_PIN in config:
        cg.add(var.set_irq_pin(await cg.gpio_pin_expression(config[CONF_IRQ_PIN])))
    for conf in config.get(CONF_ON_TAG, []):
        await automation.build_callback_automation(
            var, "add_on_tag_callback", [(cg.std_string, "x")], conf
        )
    for conf in config.get(CONF_ON_TAG_REMOVED, []):
        await automation.build_callback_automation(
            var, "add_on_tag_removed_callback", [(cg.std_string, "x")], conf
        )
