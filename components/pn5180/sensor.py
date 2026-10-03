import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_TYPE,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WEIGHT,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_MILLIMETER,
    UNIT_MINUTE,
)

from . import CONF_PN5180_ID, PN5180_CHILD_SCHEMA, FieldSource

UNIT_GRAM = "g"
UNIT_GRAMS_PER_CUBIC_CENTIMETER = "g/cm³"

# type: (source, key, unit, device class, decimals); keys from the OpenPrintTag spec
_WEIGHT = (UNIT_GRAM, DEVICE_CLASS_WEIGHT, 0)
_TEMPERATURE = (UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, 0)
TYPES = {
    "remaining_weight": ("SOURCE_REMAINING_WEIGHT", 0, *_WEIGHT),
    "consumed_weight": ("SOURCE_AUX", 0, *_WEIGHT),
    "nominal_weight": ("SOURCE_MAIN", 16, *_WEIGHT),
    "actual_weight": ("SOURCE_MAIN", 17, *_WEIGHT),
    "empty_container_weight": ("SOURCE_MAIN", 18, *_WEIGHT),
    "density": ("SOURCE_MAIN", 29, UNIT_GRAMS_PER_CUBIC_CENTIMETER, None, 2),
    "filament_diameter": ("SOURCE_MAIN", 30, UNIT_MILLIMETER, None, 2),
    "min_print_temperature": ("SOURCE_MAIN", 34, *_TEMPERATURE),
    "max_print_temperature": ("SOURCE_MAIN", 35, *_TEMPERATURE),
    "preheat_temperature": ("SOURCE_MAIN", 36, *_TEMPERATURE),
    "min_bed_temperature": ("SOURCE_MAIN", 37, *_TEMPERATURE),
    "max_bed_temperature": ("SOURCE_MAIN", 38, *_TEMPERATURE),
    "chamber_temperature": ("SOURCE_MAIN", 41, *_TEMPERATURE),
    "drying_temperature": ("SOURCE_MAIN", 57, *_TEMPERATURE),
    "drying_time": ("SOURCE_MAIN", 58, UNIT_MINUTE, DEVICE_CLASS_DURATION, 0),
}


def _schema(config):
    _, _, unit, device_class, decimals = TYPES[config[CONF_TYPE]]
    kwargs = {"unit_of_measurement": unit, "accuracy_decimals": decimals}
    if device_class:
        kwargs["device_class"] = device_class
    if config[CONF_TYPE] in ("remaining_weight", "consumed_weight"):
        kwargs["state_class"] = STATE_CLASS_MEASUREMENT
    return sensor.sensor_schema(**kwargs).extend(PN5180_CHILD_SCHEMA, _TYPE_SCHEMA)(config)


_TYPE_SCHEMA = cv.Schema({cv.Required(CONF_TYPE): cv.one_of(*TYPES, lower=True)})
CONFIG_SCHEMA = cv.All(_TYPE_SCHEMA.extend({}, extra=cv.ALLOW_EXTRA), _schema)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_PN5180_ID])
    var = await sensor.new_sensor(config)
    source, key, *_ = TYPES[config[CONF_TYPE]]
    cg.add(parent.add_sensor(var, getattr(FieldSource, source), key))
