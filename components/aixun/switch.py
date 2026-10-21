import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.types import ConfigType

from . import AIXUN_CHILD_SCHEMA, CONF_AIXUN_ID, Aixun, aixun_ns, validate_p2408

CONF_DC_OUTPUT = "dc_output"

DCOutputSwitch = aixun_ns.class_(
    "DCOutputSwitch", switch.Switch, cg.Parented.template(Aixun)
)

# The status line's out flags do not follow the real output, so the switch is optimistic and
# does not touch the output when the ESP boots.
CONFIG_SCHEMA = AIXUN_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_DC_OUTPUT): switch.switch_schema(
            DCOutputSwitch, default_restore_mode="DISABLED"
        ),
    }
)
FINAL_VALIDATE_SCHEMA = validate_p2408


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_AIXUN_ID])
    switches = switch.sub_switches(config, parent=hub)
    await switches(CONF_DC_OUTPUT, None)
