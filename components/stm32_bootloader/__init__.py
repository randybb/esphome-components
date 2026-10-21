from esphome import pins
import esphome.codegen as cg
from esphome.components import http_request, uart
import esphome.config_validation as cv
from esphome.const import CONF_RESET_PIN
from esphome.types import ConfigType

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


async def to_code(config: ConfigType) -> None:
    var = await uart.new_uart_component(config)
    await cg.add_pin(var.set_boot0_pin, config[CONF_BOOT0_PIN])
    await cg.add_pin(var.set_reset_pin, config[CONF_RESET_PIN])
    await cg.add_variable(var.set_http_request, config[CONF_HTTP_REQUEST_ID])
