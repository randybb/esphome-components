import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_MODEL,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)
import esphome.final_validate as fv
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID

OUTPUTS = ("pd_qc", "boot", "dc")  # order of the V/I pairs in the status line
KINDS = {
    "voltage": sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "current": sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "power": sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

# H310D status, index = slot in the hub
H310D_SENSORS = {
    "current_temperature": sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "current_fan_speed": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        icon="mdi:fan",
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "heating_power": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        icon="mdi:flash",
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}
P2408_KEYS = [f"{out}_{kind}" for out in OUTPUTS for kind in KINDS]

CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {
        **{
            cv.Optional(f"{out}_{kind}"): schema
            for out in OUTPUTS
            for kind, schema in KINDS.items()
        },
        **{cv.Optional(key): schema for key, schema in H310D_SENSORS.items()},
    }
)


def validate_models(config: ConfigType) -> ConfigType:
    model = fv.get_declaring_config(config[CONF_AIXUN_ID]).get(CONF_MODEL)
    if any(k in config for k in P2408_KEYS) and model != "P2408":
        raise cv.Invalid("the output sensors need model: P2408 on the aixun hub")
    if any(k in config for k in H310D_SENSORS) and model != "H310D":
        raise cv.Invalid("the status sensors need model: H310D on the aixun hub")
    return config


FINAL_VALIDATE_SCHEMA = validate_models


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    for i, out in enumerate(OUTPUTS):
        for j, kind in enumerate(KINDS):
            if conf := config.get(f"{out}_{kind}"):
                cg.add(hub.set_sensor(i, j, await sensor.new_sensor(conf)))
    for idx, key in enumerate(H310D_SENSORS):
        if conf := config.get(key):
            cg.add(hub.set_h310d_sensor(idx, await sensor.new_sensor(conf)))
