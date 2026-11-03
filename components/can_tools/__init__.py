from esphome import automation
import esphome.codegen as cg
from esphome.components import canbus
import esphome.config_validation as cv
from esphome.const import (
    CONF_DATA,
    CONF_ID,
    CONF_INTERVAL,
    CONF_TIMEOUT,
    CONF_TRIGGER_ID,
)
from esphome.types import ConfigType

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["canbus"]

CONF_CAN_ID = canbus.CONF_CAN_ID
CONF_REQUEST_ID = "request_id"
CONF_REQUESTS = "requests"
CONF_RESPONSE_CAN_ID = "response_can_id"
CONF_ON_RESPONSE = "on_response"
OBD_FUNCTIONAL_ID = 0x7DF

can_tools_ns = cg.esphome_ns.namespace("can_tools")
CanTools = can_tools_ns.class_("CanTools", cg.PollingComponent)
Request = can_tools_ns.class_("Request")
ResponseTrigger = can_tools_ns.class_(
    "ResponseTrigger", automation.Trigger.template(cg.std_vector.template(cg.uint8))
)
# Argument of the decoding lambdas: the ISO-TP payload of a positive response
DATA_ARGS = [(cg.std_vector.template(cg.uint8).operator("ref").operator("const"), "data")]


def default_response_id(config: ConfigType) -> ConfigType:
    if CONF_RESPONSE_CAN_ID not in config:
        tx = config[CONF_CAN_ID]
        config[CONF_RESPONSE_CAN_ID] = 0x7E8 if tx == OBD_FUNCTIONAL_ID else tx + 8
    return config


REQUEST_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_ID): cv.declare_id(Request),
            cv.Required(CONF_CAN_ID): cv.int_range(min=0, max=0x7FF),
            cv.Optional(CONF_RESPONSE_CAN_ID): cv.int_range(min=0, max=0x7FF),
            # single frame request: service and its parameters, up to 7 bytes
            cv.Required(CONF_DATA): cv.All(
                [cv.hex_uint8_t], cv.Length(min=1, max=7)
            ),
            cv.Optional(CONF_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TIMEOUT, default="500ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ON_RESPONSE): automation.validate_automation(
                {cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(ResponseTrigger)}
            ),
        }
    ),
    default_response_id,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CanTools),
        cv.GenerateID(canbus.CONF_CANBUS_ID): cv.use_id(canbus.CanbusComponent),
        cv.Optional(CONF_REQUESTS, default=[]): cv.ensure_list(REQUEST_SCHEMA),
    }
).extend(cv.polling_component_schema("100ms"))

CAN_TOOLS_CHILD_SCHEMA = cv.Schema({cv.Required(CONF_REQUEST_ID): cv.use_id(Request)})


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_canbus(await cg.get_variable(config[canbus.CONF_CANBUS_ID])))
    for conf in config[CONF_REQUESTS]:
        req = cg.new_Pvariable(
            conf[CONF_ID],
            conf[CONF_CAN_ID],
            conf[CONF_RESPONSE_CAN_ID],
            conf[CONF_DATA],
            conf[CONF_INTERVAL],
            conf[CONF_TIMEOUT],
        )
        cg.add(var.add_request(req))
        for trigger_conf in conf.get(CONF_ON_RESPONSE, []):
            trigger = cg.new_Pvariable(trigger_conf[CONF_TRIGGER_ID], req)
            await automation.build_automation(
                trigger, [(cg.std_vector.template(cg.uint8), "data")], trigger_conf
            )
