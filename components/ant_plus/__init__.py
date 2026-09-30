import esphome.codegen as cg
from esphome.components import sensor, uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ICON_HEART_PULSE,
    STATE_CLASS_MEASUREMENT,
    UNIT_BEATS_PER_MINUTE,
)

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]

CONF_HEART_RATE = "heart_rate"
CONF_DEVICE_NUMBER = "device_number"

ant_plus_ns = cg.esphome_ns.namespace("ant_plus")
AntPlus = ant_plus_ns.class_("AntPlus", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AntPlus),
            # the 20-bit ANT ID Garmin shows; 0 takes whichever monitor the bridge found
            cv.Optional(CONF_DEVICE_NUMBER, default=0): cv.int_range(0, 0xFFFFF),
            cv.Optional(CONF_HEART_RATE): sensor.sensor_schema(
                unit_of_measurement=UNIT_BEATS_PER_MINUTE,
                icon=ICON_HEART_PULSE,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "ant_plus", baud_rate=115200, require_rx=True
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_device_number(config[CONF_DEVICE_NUMBER]))
    if heart_rate := config.get(CONF_HEART_RATE):
        cg.add(var.set_heart_rate_sensor(await sensor.new_sensor(heart_rate)))
