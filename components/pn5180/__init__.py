from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import nfc, spi
import esphome.config_validation as cv
from esphome.const import (
    CONF_BUSY_PIN,
    CONF_INVERTED,
    CONF_IRQ_PIN,
    CONF_ON_TAG,
    CONF_ON_TAG_REMOVED,
    CONF_RESET_PIN,
)
from esphome.types import ConfigType

AUTO_LOAD = ["nfc"]
CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["spi"]
DOMAIN = "pn5180"
MULTI_CONF = True

CONF_PN5180_ID = "pn5180_id"

pn5180_ns = cg.esphome_ns.namespace("pn5180")
PN5180 = pn5180_ns.class_("PN5180", cg.PollingComponent, spi.SPIDevice, nfc.Nfcc)


def _irq_pin(value):
    value = pins.internal_gpio_input_pin_schema(value)
    if value.get(CONF_INVERTED):
        raise cv.Invalid(
            "the IRQ polarity is read from the PN5180 EEPROM, 'inverted' is not supported"
        )
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
    DOMAIN, require_miso=True, require_mosi=True
)

PN5180_CHILD_SCHEMA = cv.Schema({cv.GenerateID(CONF_PN5180_ID): cv.use_id(PN5180)})

_request_listener_slot = cg.slot_counter("PN5180_LISTENER_COUNT")


async def register_listener(config: ConfigType, var: cg.MockObj) -> None:
    """Connect a decoding component (a TagListener) to the reader in its PN5180_CHILD_SCHEMA config."""
    reader = await cg.get_variable(config[CONF_PN5180_ID])
    _request_listener_slot(str(reader))
    cg.add(var.set_reader(reader))


_TAG_ARGS = [(cg.std_string, "x"), (nfc.NfcTagConstRef, "tag")]

_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(CONF_ON_TAG, "add_on_tag_callback", _TAG_ARGS),
    automation.CallbackAutomation(
        CONF_ON_TAG_REMOVED, "add_on_tag_removed_callback", _TAG_ARGS
    ),
)


async def to_code(config: ConfigType) -> None:
    var = await spi.new_spi_component(config)
    await cg.add_pin(var.set_busy_pin, config[CONF_BUSY_PIN])
    await cg.add_pin(var.set_reset_pin, config.get(CONF_RESET_PIN))
    await cg.add_pin(var.set_irq_pin, config.get(CONF_IRQ_PIN))
    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)
