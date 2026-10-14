from esphome import pins
import esphome.codegen as cg
from esphome.components import http_request, uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_RESET_PIN

CODEOWNERS = ["@randybb"]
DEPENDENCIES = ["uart", "http_request"]
DOMAIN = "stm32_bootloader"

CONF_BOOT0_PIN = "boot0_pin"
CONF_HTTP_REQUEST_ID = "http_request_id"

stm32_bootloader_ns = cg.esphome_ns.namespace("stm32_bootloader")
STM32Bootloader = stm32_bootloader_ns.class_(
    "STM32Bootloader", cg.Component, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(STM32Bootloader),
            cv.Required(CONF_BOOT0_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.GenerateID(CONF_HTTP_REQUEST_ID): cv.use_id(
                http_request.HttpRequestComponent
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_boot0_pin(await cg.gpio_pin_expression(config[CONF_BOOT0_PIN])))
    cg.add(var.set_reset_pin(await cg.gpio_pin_expression(config[CONF_RESET_PIN])))
    cg.add(var.set_http_request(await cg.get_variable(config[CONF_HTTP_REQUEST_ID])))
