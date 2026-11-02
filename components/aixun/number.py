import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_MODEL,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)
import esphome.final_validate as fv
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID, Aixun, aixun_ns

CONF_DC_CURRENT_LIMIT = "dc_current_limit"
# H310D
CONF_TARGET_TEMPERATURE = "target_temperature"
CONF_FAN_SPEED = "fan_speed"

DCCurrentLimitNumber = aixun_ns.class_(
    "DCCurrentLimitNumber", number.Number, cg.Parented.template(Aixun)
)
H310DTemperatureNumber = aixun_ns.class_(
    "H310DTemperatureNumber", number.Number, cg.Parented.template(Aixun)
)
H310DFanNumber = aixun_ns.class_(
    "H310DFanNumber", number.Number, cg.Parented.template(Aixun)
)

CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_DC_CURRENT_LIMIT): number.number_schema(
            DCCurrentLimitNumber,
            unit_of_measurement=UNIT_AMPERE,
            device_class=DEVICE_CLASS_CURRENT,
        ),
        cv.Optional(CONF_TARGET_TEMPERATURE): number.number_schema(
            H310DTemperatureNumber,
            unit_of_measurement=UNIT_CELSIUS,
            device_class=DEVICE_CLASS_TEMPERATURE,
            icon="mdi:heat-wave",
        ),
        cv.Optional(CONF_FAN_SPEED): number.number_schema(
            H310DFanNumber, unit_of_measurement=UNIT_PERCENT, icon="mdi:fan"
        ),
    }
)


def validate_models(config: ConfigType) -> ConfigType:
    model = fv.get_declaring_config(config[CONF_AIXUN_ID]).get(CONF_MODEL)
    if CONF_DC_CURRENT_LIMIT in config and model != "P2408":
        raise cv.Invalid(f"{CONF_DC_CURRENT_LIMIT} needs model: P2408 on the aixun hub")
    if (CONF_TARGET_TEMPERATURE in config or CONF_FAN_SPEED in config) and model != "H310D":
        raise cv.Invalid("target_temperature/fan_speed need model: H310D on the aixun hub")
    return config


FINAL_VALIDATE_SCHEMA = validate_models

# (key, setter on the hub, min, max, step)
NUMBERS = (
    (CONF_DC_CURRENT_LIMIT, "set_dc_limit_number", 0, 8, 0.01),
    (CONF_TARGET_TEMPERATURE, "set_target_temperature_number", 100, 500, 1),
    (CONF_FAN_SPEED, "set_fan_speed_number", 1, 100, 1),
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    for key, setter, lo, hi, step in NUMBERS:
        if conf := config.get(key):
            n = await number.new_number(conf, min_value=lo, max_value=hi, step=step)
            await cg.register_parented(n, hub)
            if setter:
                cg.add(getattr(hub, setter)(n))
