import esphome.codegen as cg
from esphome.components.zephyr import zephyr_add_overlay, zephyr_add_prj_conf
from esphome.components.zephyr_rtos import zephyr_rtos_data
from esphome.components.zephyr_rtos.const import KEY_BOARD
from esphome.components.zephyr_rtos.dts_lookup import get_nfct_pins_node_label
import esphome.config_validation as cv
from esphome.const import CONF_FRAMEWORK, CONF_ID, CONF_TYPE, CONF_URL, PLATFORM_NRF
import esphome.final_validate as fv

CODEOWNERS = ["@randybb"]

nfc_tag_ns = cg.esphome_ns.namespace("nfc_tag")
NfcTag = nfc_tag_ns.class_("NfcTag", cg.Component)

# NFC Forum URI identifier codes, longest first
URI_PREFIXES = (
    ("https://www.", 0x02),
    ("http://www.", 0x01),
    ("https://", 0x04),
    ("http://", 0x03),
    ("tel:", 0x05),
    ("mailto:", 0x06),
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NfcTag),
            cv.Required(CONF_URL): cv.All(cv.string_strict, cv.Length(min=1, max=200)),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_NRF]),
)


def _final_validate(config):
    framework = fv.full_config.get()[PLATFORM_NRF][CONF_FRAMEWORK]
    if framework.get(CONF_TYPE) != "ncs":
        raise cv.Invalid("nfc_tag needs the NCS T2T library: nrf: framework: type: ncs")
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    zephyr_add_prj_conf("NFC_T2T_NRFXLIB", True)
    zephyr_add_prj_conf("NFC_NDEF", True)
    zephyr_add_prj_conf("NFC_NDEF_MSG", True)
    zephyr_add_prj_conf("NFC_NDEF_RECORD", True)
    zephyr_add_prj_conf("NFC_NDEF_URI_REC", True)
    zephyr_add_prj_conf("NFC_NDEF_URI_MSG", True)
    # the nrf platform turns the antenna pins into GPIOs; undo it (runs after it)
    overlay = '&nfct { status = "okay"; };\n'
    if label := get_nfct_pins_node_label(zephyr_rtos_data()[KEY_BOARD]):
        overlay += f"&{label} {{ /delete-property/ nfct-pins-as-gpios; }};\n"
    zephyr_add_overlay(overlay)

    url = config[CONF_URL]
    code = 0
    for prefix, prefix_code in URI_PREFIXES:
        if url.startswith(prefix):
            code, url = prefix_code, url[len(prefix) :]
            break

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_uri(code, url))
