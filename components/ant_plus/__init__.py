from pathlib import Path

import esphome.codegen as cg
from esphome import git
from esphome.components import sensor
from esphome.components.zephyr import (
    zephyr_add_overlay,
    zephyr_add_prj_conf,
    zephyr_set_module_override,
)
from esphome.components.zephyr.variants import ZephyrModule
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ICON_HEART_PULSE,
    PLATFORM_ZEPHYR,
    STATE_CLASS_MEASUREMENT,
    UNIT_BEATS_PER_MINUTE,
)
from esphome.core import CORE

CODEOWNERS = ["@randybb"]
AUTO_LOAD = ["sensor"]

CONF_HEART_RATE = "heart_rate"
CONF_DEVICE_NUMBER = "device_number"
CONF_NETWORK_KEY = "network_key"

# RadiANT: clean-room ANT+ compatible link layer for Zephyr (Apache-2.0)
RADIANT_URL = "https://github.com/winedarksea/RadiANT"
RADIANT_REF = "6979eb9ae9417dee5d9de43df5f816765a901048"

ant_plus_ns = cg.esphome_ns.namespace("ant_plus")
AntPlus = ant_plus_ns.class_("AntPlus", cg.Component)


def network_key(value):
    value = cv.string_strict(value).replace(":", "").replace(" ", "")
    try:
        key = bytes.fromhex(value)
    except ValueError as err:
        raise cv.Invalid("network_key must be hex") from err
    if len(key) != 8:
        raise cv.Invalid("network_key must be 8 bytes (16 hex digits)")
    return list(key)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AntPlus),
            # the ANT+ network key, from thisisant.com (ANT+ Adopter)
            cv.Required(CONF_NETWORK_KEY): network_key,
            # the 20-bit ANT ID Garmin shows; 0 pairs with the first monitor found
            cv.Optional(CONF_DEVICE_NUMBER, default=0): cv.int_range(0, 0xFFFFF),
            cv.Optional(CONF_HEART_RATE): sensor.sensor_schema(
                unit_of_measurement=UNIT_BEATS_PER_MINUTE,
                icon=ICON_HEART_PULSE,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ZEPHYR]),
)


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
    cg.add(var.set_device_number(config[CONF_DEVICE_NUMBER]))
    if heart_rate := config.get(CONF_HEART_RATE):
        cg.add(var.set_heart_rate_sensor(await sensor.new_sensor(heart_rate)))
