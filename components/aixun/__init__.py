import esphome.codegen as cg
from esphome.components import number, sensor, switch, text_sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MODEL,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_VOLT,
    UNIT_WATT,
)

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["uart"]
DOMAIN = "aixun"
AUTO_LOAD = ["number", "sensor", "switch", "text_sensor"]

CONF_DC_OUTPUT = "dc_output"
CONF_DC_CURRENT_LIMIT = "dc_current_limit"
CONF_DEVICE_MODEL = "device_model"
CONF_FIRMWARE_VERSION = "firmware_version"
# Models with their own entities, the version text sensors work on any Aixun device.
MODELS = ["P2408"]
OUTPUTS = ("pd_qc", "boot", "dc")  # order of the V/I pairs in the status line

aixun_ns = cg.esphome_ns.namespace("aixun")
Aixun = aixun_ns.class_("Aixun", cg.PollingComponent, uart.UARTDevice)
DCOutputSwitch = aixun_ns.class_(
    "DCOutputSwitch", switch.Switch, cg.Parented.template(Aixun)
)
DCCurrentLimitNumber = aixun_ns.class_(
    "DCCurrentLimitNumber", number.Number, cg.Parented.template(Aixun)
)

SENSORS = {}
for out in OUTPUTS:
    SENSORS[f"{out}_voltage"] = sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    SENSORS[f"{out}_current"] = sensor.sensor_schema(
        unit_of_measurement=UNIT_AMPERE,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_CURRENT,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    SENSORS[f"{out}_power"] = sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    )

P2408_KEYS = [*SENSORS, CONF_DC_OUTPUT, CONF_DC_CURRENT_LIMIT]


def validate_model(config):
    if config.get(CONF_MODEL) != "P2408":
        for key in P2408_KEYS:
            if key in config:
                raise cv.Invalid(f"'{key}' needs model: P2408", [key])
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Aixun),
            cv.Optional(CONF_MODEL): cv.one_of(*MODELS, upper=True),
            cv.Optional(CONF_DEVICE_MODEL): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:information-outline",
            ),
            cv.Optional(CONF_FIRMWARE_VERSION): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:chip"
            ),
            **{cv.Optional(k): v for k, v in SENSORS.items()},
            # The status line's out flags do not follow the real output, so the switch is optimistic and
            # does not touch the output when the ESP boots.
            cv.Optional(CONF_DC_OUTPUT): switch.switch_schema(
                DCOutputSwitch, default_restore_mode="DISABLED"
            ),
            cv.Optional(CONF_DC_CURRENT_LIMIT): number.number_schema(
                DCCurrentLimitNumber,
                unit_of_measurement=UNIT_AMPERE,
                device_class=DEVICE_CLASS_CURRENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(uart.UART_DEVICE_SCHEMA),
    validate_model,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_p2408(config.get(CONF_MODEL) == "P2408"))

    if conf := config.get(CONF_DEVICE_MODEL):
        cg.add(
            var.set_device_model_text_sensor(await text_sensor.new_text_sensor(conf))
        )
    if conf := config.get(CONF_FIRMWARE_VERSION):
        cg.add(
            var.set_firmware_version_text_sensor(
                await text_sensor.new_text_sensor(conf)
            )
        )

    for i, out in enumerate(OUTPUTS):
        for j, kind in enumerate(("voltage", "current", "power")):
            if conf := config.get(f"{out}_{kind}"):
                cg.add(var.set_sensor(i, j, await sensor.new_sensor(conf)))

    if conf := config.get(CONF_DC_OUTPUT):
        sw = await switch.new_switch(conf)
        await cg.register_parented(sw, var)
    if conf := config.get(CONF_DC_CURRENT_LIMIT):
        num = await number.new_number(conf, min_value=0, max_value=8, step=0.01)
        await cg.register_parented(num, var)
        cg.add(var.set_dc_limit_number(num))
