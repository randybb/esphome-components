from esphome import automation
import esphome.codegen as cg
from esphome.components.pn5180 import CONF_PN5180_ID, PN5180
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["pn5180"]
MULTI_CONF = True

CONF_CREALITY_CFS_ID = "creality_cfs_id"
CONF_ON_CREALITY = "on_creality"

creality_cfs_ns = cg.esphome_ns.namespace("creality_cfs")
CrealityCfsComponent = creality_cfs_ns.class_("CrealityCfsComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CrealityCfsComponent),
        cv.GenerateID(CONF_PN5180_ID): cv.use_id(PN5180),
        cv.Optional(CONF_ON_CREALITY): automation.validate_automation({}),
    }
).extend(cv.COMPONENT_SCHEMA)

CREALITY_CFS_CHILD_SCHEMA = cv.Schema(
    {cv.GenerateID(CONF_CREALITY_CFS_ID): cv.use_id(CrealityCfsComponent)}
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_reader(await cg.get_variable(config[CONF_PN5180_ID])))
    for conf in config.get(CONF_ON_CREALITY, []):
        await automation.build_callback_automation(
            var,
            "add_on_creality_callback",
            [(cg.std_string, "uid"), (cg.std_string, "data")],
            conf,
        )
