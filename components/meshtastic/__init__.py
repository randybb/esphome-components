import base64

from esphome import automation
import esphome.codegen as cg
from esphome.components.sx127x import CONF_SX127X_ID, SX127x, SX127xListener
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_TEXT, CONF_TO
from esphome.types import ConfigType

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["sx127x"]

CONF_CHANNEL = "channel"
CONF_HOP_LIMIT = "hop_limit"
CONF_KEY = "key"
CONF_LONG_NAME = "long_name"
CONF_NODE_ID = "node_id"
CONF_ON_MESSAGE = "on_message"
CONF_ON_NODE = "on_node"
CONF_SHORT_NAME = "short_name"

BROADCAST = 0xFFFFFFFF

meshtastic_ns = cg.esphome_ns.namespace("meshtastic")
Meshtastic = meshtastic_ns.class_("Meshtastic", cg.Component, SX127xListener)
Node = meshtastic_ns.struct("Node")


def channel_key(value):
    """The channel PSK as the Meshtastic app shows it, base64: 1 byte (AQ== is the default key), 16 or 32."""
    value = cv.string_strict(value)
    try:
        key = base64.b64decode(value, validate=True)
    except ValueError as err:
        raise cv.Invalid("key must be base64, as shown in the Meshtastic app") from err
    if len(key) not in (1, 16, 32):
        raise cv.Invalid("key must be 1, 16 or 32 bytes")
    return list(key)


def node_id(value):
    if isinstance(value, str) and value.startswith("!"):
        value = "0x" + value[1:]
    return cv.hex_uint32_t(value)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Meshtastic),
            cv.GenerateID(CONF_SX127X_ID): cv.use_id(SX127x),
            cv.Optional(CONF_CHANNEL, default="LongFast"): cv.All(
                cv.string_strict, cv.Length(min=1, max=11)
            ),
            cv.Optional(CONF_KEY, default="AQ=="): channel_key,
            cv.Optional(CONF_NODE_ID): node_id,
            cv.Optional(CONF_LONG_NAME): cv.All(cv.string_strict, cv.Length(min=1, max=39)),
            cv.Optional(CONF_SHORT_NAME): cv.All(cv.string_strict, cv.Length(min=1, max=4)),
            cv.Optional(CONF_HOP_LIMIT, default=3): cv.int_range(min=0, max=7),
            cv.Optional(CONF_ON_MESSAGE): automation.validate_automation(single=True),
            cv.Optional(CONF_ON_NODE): automation.validate_automation(single=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_parented(var, config[CONF_SX127X_ID])
    cg.add(var.set_channel(config[CONF_CHANNEL], config[CONF_KEY]))
    cg.add(var.set_hop_limit(config[CONF_HOP_LIMIT]))
    if CONF_NODE_ID in config:
        cg.add(var.set_node_id(config[CONF_NODE_ID]))
    if CONF_LONG_NAME in config:
        short = config.get(CONF_SHORT_NAME, config[CONF_LONG_NAME][:4])
        cg.add(var.set_names(config[CONF_LONG_NAME], short))
    if CONF_ON_MESSAGE in config:
        await automation.build_automation(
            var.get_message_trigger(),
            [
                (cg.uint32, "from"),
                (cg.uint32, "to"),
                (cg.std_string, "from_name"),
                (cg.std_string, "text"),
                (cg.float_, "rssi"),
                (cg.float_, "snr"),
            ],
            config[CONF_ON_MESSAGE],
        )
    if CONF_ON_NODE in config:
        await automation.build_automation(
            var.get_node_trigger(), [(Node, "node")], config[CONF_ON_NODE]
        )


automation.register_apply_action(
    "meshtastic.send_text",
    cv.maybe_simple_value(
        {
            cv.GenerateID(): cv.use_id(Meshtastic),
            cv.Required(CONF_TEXT): cv.templatable(cv.string),
            cv.Optional(CONF_TO, default=BROADCAST): cv.templatable(node_id),
        },
        key=CONF_TEXT,
    ),
    automation.ApplyCall(
        "send_text({}, {})",
        ((CONF_TEXT, cg.std_string), (CONF_TO, cg.uint32)),
    ),
)
