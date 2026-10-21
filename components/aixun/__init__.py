import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_MODEL
import esphome.final_validate as fv
from esphome.types import ConfigType

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["uart"]
DOMAIN = "aixun"

CONF_AIXUN_ID = "aixun_id"
# Models with their own entities, the version text sensors work on any Aixun device.
MODELS = ["P2408"]

aixun_ns = cg.esphome_ns.namespace("aixun")
Aixun = aixun_ns.class_("Aixun", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Aixun),
            cv.Optional(CONF_MODEL): cv.one_of(*MODELS, upper=True),
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

AIXUN_CHILD_SCHEMA = cv.Schema({cv.GenerateID(CONF_AIXUN_ID): cv.use_id(Aixun)})


def validate_p2408(config: ConfigType) -> ConfigType:
    """Final validation of the platforms whose entities exist only on a P2408."""
    if fv.get_declaring_config(config[CONF_AIXUN_ID]).get(CONF_MODEL) != "P2408":
        raise cv.Invalid("these entities need model: P2408 on the aixun hub")
    return config


async def to_code(config: ConfigType) -> None:
    var = await uart.new_uart_component(config)
    cg.add(var.set_p2408(config.get(CONF_MODEL) == "P2408"))
