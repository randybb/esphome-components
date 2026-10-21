import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import DEVICE_CLASS_CURRENT, UNIT_AMPERE
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID, Aixun, aixun_ns, validate_p2408

CONF_DC_CURRENT_LIMIT = "dc_current_limit"

DCCurrentLimitNumber = aixun_ns.class_(
    "DCCurrentLimitNumber", number.Number, cg.Parented.template(Aixun)
)

CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_DC_CURRENT_LIMIT): number.number_schema(
            DCCurrentLimitNumber,
            unit_of_measurement=UNIT_AMPERE,
            device_class=DEVICE_CLASS_CURRENT,
        ),
    }
)
FINAL_VALIDATE_SCHEMA = validate_p2408


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    numbers = number.sub_numbers(config, parent=hub)
    await numbers(
        CONF_DC_CURRENT_LIMIT,
        hub.set_dc_limit_number,
        min_value=0,
        max_value=8,
        step=0.01,
    )
