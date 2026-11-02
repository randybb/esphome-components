import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID, Aixun, aixun_ns, validate_h310d

H310DPresetButton = aixun_ns.class_(
    "H310DPresetButton", button.Button, cg.Parented.template(Aixun)
)

# H310D: recall the M1..M3 presets (short press, never saves).
PRESETS = {"preset_m1": 1, "preset_m2": 2, "preset_m3": 3}

CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {cv.Optional(key): button.button_schema(H310DPresetButton) for key in PRESETS}
)
FINAL_VALIDATE_SCHEMA = validate_h310d


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    for key, m in PRESETS.items():
        if conf := config.get(key):
            b = await button.new_button(conf)
            await cg.register_parented(b, hub)
            cg.add(b.set_preset(m))
