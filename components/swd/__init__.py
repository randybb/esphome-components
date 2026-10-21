from esphome import pins
import esphome.codegen as cg
from esphome.components import http_request
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.types import ConfigType

CODEOWNERS = ["@randybb"]
DOMAIN = "swd"

CONF_SWCLK_PIN = "swclk_pin"
CONF_SWDIO_PIN = "swdio_pin"
CONF_RAM_START = "ram_start"
CONF_RAM_SIZE = "ram_size"
CONF_CLOCK_DELAY = "clock_delay"
CONF_HTTP_REQUEST_ID = "http_request_id"

swd_ns = cg.esphome_ns.namespace("swd")
SWD = swd_ns.class_("SWD", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(SWD),
        cv.Required(CONF_SWCLK_PIN): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_SWDIO_PIN): pins.internal_gpio_output_pin_schema,
        # STM32F103C8: 20 kB SRAM
        cv.Optional(CONF_RAM_START, default=0x20000000): cv.hex_uint32_t,
        cv.Optional(CONF_RAM_SIZE, default=0x5000): cv.hex_uint32_t,
        cv.Optional(
            CONF_CLOCK_DELAY, default="1us"
        ): cv.positive_time_period_microseconds,
        # Enables flash(url, address)
        cv.Optional(CONF_HTTP_REQUEST_ID): cv.use_id(http_request.HttpRequestComponent),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.add_pin(var.set_swclk_pin, config[CONF_SWCLK_PIN])
    await cg.add_pin(var.set_swdio_pin, config[CONF_SWDIO_PIN])
    cg.add(var.set_ram(config[CONF_RAM_START], config[CONF_RAM_SIZE]))
    cg.add(var.set_clock_delay(config[CONF_CLOCK_DELAY].total_microseconds))
    if await cg.add_variable(var.set_http_request, config.get(CONF_HTTP_REQUEST_ID)):
        cg.add_define("USE_SWD_FLASH")
