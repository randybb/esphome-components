from pathlib import Path

import esphome.codegen as cg
from esphome import git
from esphome.components import binary_sensor, sensor
from esphome.components.zephyr import (
    zephyr_add_overlay,
    zephyr_add_prj_conf,
    zephyr_set_module_override,
)
from esphome.components.zephyr.variants import ZephyrModule
import esphome.config_validation as cv
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    CONF_DISTANCE,
    CONF_ID,
    CONF_POWER,
    CONF_SPEED,
    CONF_TEMPERATURE,
    CONF_TYPE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_RUNNING,
    DEVICE_CLASS_SPEED,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_HEART_PULSE,
    PLATFORM_ZEPHYR,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_BEATS_PER_MINUTE,
    UNIT_CELSIUS,
    UNIT_METER,
    UNIT_METER_PER_SECOND,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)
from esphome.core import CORE

CODEOWNERS = ["@randybb"]
AUTO_LOAD = ["sensor", "binary_sensor"]

CONF_CADENCE = "cadence"
CONF_CONNECTED = "connected"
CONF_DEVICE_NUMBER = "device_number"
CONF_DEVICES = "devices"
CONF_HEART_RATE = "heart_rate"
CONF_IN_USE = "in_use"
CONF_BATTERY_LOW = "battery_low"
CONF_NETWORK_KEY = "network_key"
CONF_TEMPERATURE_MAX = "temperature_max"
CONF_UNKNOWN_DEVICE = "unknown_device"
CONF_TEMPERATURE_MIN = "temperature_min"

# RadiANT: clean-room ANT+ compatible link layer for Zephyr (Apache-2.0)
RADIANT_URL = "https://github.com/winedarksea/RadiANT"
RADIANT_REF = "6979eb9ae9417dee5d9de43df5f816765a901048"

ant_plus_ns = cg.esphome_ns.namespace("ant_plus")
AntPlus = ant_plus_ns.class_("AntPlus", cg.Component)
AntPlusDevice = ant_plus_ns.class_("AntPlusDevice")
DeviceType = ant_plus_ns.enum("DeviceType", is_class=True)

TYPE_HEART_RATE = "heart_rate"
TYPE_FITNESS_EQUIPMENT = "fitness_equipment"
TYPE_TEMPERATURE = "temperature"
TYPE_BIKE_RADAR = "bike_radar"
DEVICE_TYPES = {
    TYPE_HEART_RATE: DeviceType.HEART_RATE,
    TYPE_FITNESS_EQUIPMENT: DeviceType.FITNESS_EQUIPMENT,
    TYPE_TEMPERATURE: DeviceType.TEMPERATURE,
    TYPE_BIKE_RADAR: DeviceType.BIKE_RADAR,
}


def network_key(value):
    value = cv.string_strict(value).replace(":", "").replace(" ", "")
    try:
        key = bytes.fromhex(value)
    except ValueError as err:
        raise cv.Invalid("network_key must be hex") from err
    if len(key) != 8:
        raise cv.Invalid("network_key must be 8 bytes (16 hex digits)")
    return list(key)


HEART_RATE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_BEATS_PER_MINUTE,
    icon=ICON_HEART_PULSE,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)
BATTERY_VOLTAGE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_VOLTAGE,
    state_class=STATE_CLASS_MEASUREMENT,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

TEMPERATURE_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

DEVICE_BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AntPlusDevice),
        # the 20-bit ANT ID Garmin shows; 0 pairs with the first sensor found and logs its ID
        cv.Required(CONF_DEVICE_NUMBER): cv.int_range(0, 0xFFFFF),
        cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        # ANT+ common page 82, or HRM page 7
        cv.Optional(CONF_BATTERY_VOLTAGE): BATTERY_VOLTAGE_SCHEMA,
        # battery status low or critical
        cv.Optional(CONF_BATTERY_LOW): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

DEVICE_SCHEMA = cv.typed_schema(
    {
        TYPE_HEART_RATE: DEVICE_BASE_SCHEMA.extend(
            {
                cv.Optional(CONF_HEART_RATE): HEART_RATE_SCHEMA,
                cv.Optional(CONF_BATTERY_LEVEL): sensor.sensor_schema(
                    unit_of_measurement=UNIT_PERCENT,
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_BATTERY,
                    state_class=STATE_CLASS_MEASUREMENT,
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),
            }
        ),
        TYPE_FITNESS_EQUIPMENT: DEVICE_BASE_SCHEMA.extend(
            {
                cv.Optional(CONF_POWER): sensor.sensor_schema(
                    unit_of_measurement=UNIT_WATT,
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_POWER,
                    state_class=STATE_CLASS_MEASUREMENT,
                ),
                # strokes/min on a rower
                cv.Optional(CONF_CADENCE): sensor.sensor_schema(
                    unit_of_measurement="spm",
                    icon="mdi:rowing",
                    accuracy_decimals=0,
                    state_class=STATE_CLASS_MEASUREMENT,
                ),
                cv.Optional(CONF_SPEED): sensor.sensor_schema(
                    unit_of_measurement=UNIT_METER_PER_SECOND,
                    accuracy_decimals=2,
                    device_class=DEVICE_CLASS_SPEED,
                    state_class=STATE_CLASS_MEASUREMENT,
                ),
                # accumulated since boot
                cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
                    unit_of_measurement=UNIT_METER,
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_DISTANCE,
                    state_class=STATE_CLASS_TOTAL_INCREASING,
                ),
                cv.Optional(CONF_HEART_RATE): HEART_RATE_SCHEMA,
                # FE state IN_USE
                cv.Optional(CONF_IN_USE): binary_sensor.binary_sensor_schema(
                    device_class=DEVICE_CLASS_RUNNING,
                ),
            }
        ),
        TYPE_TEMPERATURE: DEVICE_BASE_SCHEMA.extend(
            {
                cv.Optional(CONF_TEMPERATURE): TEMPERATURE_SCHEMA,
                # the sensor's own low/high over the last 24 hours
                cv.Optional(CONF_TEMPERATURE_MIN): TEMPERATURE_SCHEMA,
                cv.Optional(CONF_TEMPERATURE_MAX): TEMPERATURE_SCHEMA,
            }
        ),
        # Garmin Varia and the like: only the battery is read
        TYPE_BIKE_RADAR: DEVICE_BASE_SCHEMA,
    },
    key=CONF_TYPE,
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AntPlus),
            # the ANT+ network key, from thisisant.com (ANT+ Adopter)
            cv.Required(CONF_NETWORK_KEY): network_key,
            cv.Required(CONF_DEVICES): cv.All(
                cv.ensure_list(DEVICE_SCHEMA), cv.Length(min=1, max=31)
            ),
            # any configured device is being received
            cv.Optional(CONF_CONNECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
            ),
            # an ANT+ device that isn't configured is in range; its ANT ID is logged.
            # Takes one more channel.
            cv.Optional(CONF_UNKNOWN_DEVICE): binary_sensor.binary_sensor_schema(
                icon="mdi:access-point",
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ZEPHYR]),
)

SENSORS = (
    CONF_HEART_RATE,
    CONF_BATTERY_LEVEL,
    CONF_BATTERY_VOLTAGE,
    CONF_POWER,
    CONF_CADENCE,
    CONF_SPEED,
    CONF_DISTANCE,
    CONF_TEMPERATURE,
    CONF_TEMPERATURE_MIN,
    CONF_TEMPERATURE_MAX,
)
BINARY_SENSORS = (CONF_CONNECTED, CONF_IN_USE, CONF_BATTERY_LOW)


def _antr_module(radiant: Path) -> Path:
    """Zephyr module compiling RadiANT's antr_* adapter (apps/common), which the
    radiant module itself leaves to the application."""
    common = (radiant / "apps" / "common").as_posix()
    module = Path(CORE.relative_build_path("ant_plus_antr"))
    (module / "zephyr").mkdir(parents=True, exist_ok=True)
    (module / "zephyr" / "module.yml").write_text(
        "name: ant_plus_antr\nbuild:\n  cmake: .\n  kconfig: Kconfig\n"
    )
    (module / "Kconfig").write_text(f'source "{common}/Kconfig.antr_api"\n')
    (module / "CMakeLists.txt").write_text(
        "if(CONFIG_RADIANT_ANTR_API)\n"
        "  zephyr_library()\n"
        f'  zephyr_library_sources("{common}/ant_radio_radiant.c")\n'
        f'  zephyr_include_directories("{common}" "{common}/ant")\n'
        "endif()\n"
    )
    return module


async def to_code(config):
    radiant, _ = git.clone_or_update(
        url=RADIANT_URL, ref=RADIANT_REF, refresh=None, domain="ant_plus"
    )
    zephyr_set_module_override(
        "radiant", ZephyrModule(name="radiant", local_path=radiant / "radiant")
    )
    zephyr_set_module_override(
        "ant_plus_antr",
        ZephyrModule(name="ant_plus_antr", local_path=_antr_module(radiant)),
    )
    zephyr_add_prj_conf("RADIANT", True)
    zephyr_add_prj_conf("RADIANT_BACKEND_NRF", True)
    # the radio backend owns a 1 MHz TIMER with 5 CC channels (TIMER3/4 on nRF52)
    zephyr_add_overlay(
        "/ { chosen { radiant,radio-timer = &timer4; }; };\n"
        '&timer4 { status = "okay"; };\n'
    )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_network_key(config[CONF_NETWORK_KEY]))
    for key in (CONF_CONNECTED, CONF_UNKNOWN_DEVICE):
        if sensor_config := config.get(key):
            s = await binary_sensor.new_binary_sensor(sensor_config)
            cg.add(getattr(var, f"set_{key}_binary_sensor")(s))

    for device_config in config[CONF_DEVICES]:
        device = cg.new_Pvariable(
            device_config[CONF_ID],
            DEVICE_TYPES[device_config[CONF_TYPE]],
            device_config[CONF_DEVICE_NUMBER],
        )
        cg.add(var.add_device(device))
        for key in SENSORS:
            if sensor_config := device_config.get(key):
                s = await sensor.new_sensor(sensor_config)
                cg.add(getattr(device, f"set_{key}_sensor")(s))
        for key in BINARY_SENSORS:
            if sensor_config := device_config.get(key):
                s = await binary_sensor.new_binary_sensor(sensor_config)
                cg.add(getattr(device, f"set_{key}_binary_sensor")(s))
