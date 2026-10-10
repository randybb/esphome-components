from esphome import automation
import esphome.codegen as cg
from esphome.components.pn5180 import CONF_PN5180_ID, PN5180
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["pn5180"]
DOMAIN = "openprinttag"
MULTI_CONF = True

CONF_OPENPRINTTAG_ID = "openprinttag_id"
CONF_ON_OPENPRINTTAG = "on_openprinttag"

openprinttag_ns = cg.esphome_ns.namespace("openprinttag")
OpenPrintTagComponent = openprinttag_ns.class_("OpenPrintTagComponent", cg.Component)
FieldSource = openprinttag_ns.enum("FieldSource")

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OpenPrintTagComponent),
        cv.GenerateID(CONF_PN5180_ID): cv.use_id(PN5180),
        cv.Optional(CONF_ON_OPENPRINTTAG): automation.validate_automation({}),
    }
).extend(cv.COMPONENT_SCHEMA)

OPENPRINTTAG_CHILD_SCHEMA = cv.Schema(
    {cv.GenerateID(CONF_OPENPRINTTAG_ID): cv.use_id(OpenPrintTagComponent)}
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_reader(await cg.get_variable(config[CONF_PN5180_ID])))
    for conf in config.get(CONF_ON_OPENPRINTTAG, []):
        await automation.build_callback_automation(
            var,
            "add_on_openprinttag_callback",
            [(cg.std_string, "uid"), (cg.std_string, "payload")],
            conf,
        )
