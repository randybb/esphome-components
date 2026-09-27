import esphome.codegen as cg
from esphome.components import audio, speaker
import esphome.config_validation as cv
from esphome.const import (
    CONF_BITS_PER_SAMPLE,
    CONF_ID,
    CONF_NUM_CHANNELS,
    CONF_OUTPUT_SPEAKER,
    CONF_SAMPLE_RATE,
    PLATFORM_ESP32,
)
from esphome.types import ConfigType

from . import audio_spectrum_ns

AUTO_LOAD = ["audio"]

AudioSpectrum = audio_spectrum_ns.class_(
    "AudioSpectrum", cg.Component, speaker.Speaker
)

CONF_BANDS = "bands"
CONF_MIN_FREQUENCY = "min_frequency"
CONF_MAX_FREQUENCY = "max_frequency"


def _set_stream_limits(config: ConfigType) -> ConfigType:
    # pass-through: accepts exactly the stream it forwards to the output speaker
    config.setdefault(CONF_BITS_PER_SAMPLE, 16)
    config.setdefault(CONF_NUM_CHANNELS, 1)
    config.setdefault(CONF_SAMPLE_RATE, 48000)
    audio.set_stream_limits(
        min_bits_per_sample=config[CONF_BITS_PER_SAMPLE],
        max_bits_per_sample=config[CONF_BITS_PER_SAMPLE],
        min_channels=config[CONF_NUM_CHANNELS],
        max_channels=config[CONF_NUM_CHANNELS],
        min_sample_rate=config[CONF_SAMPLE_RATE],
        max_sample_rate=config[CONF_SAMPLE_RATE],
    )(config)
    return config


def _validate_frequency_range(config: ConfigType) -> ConfigType:
    if config[CONF_MIN_FREQUENCY] >= config[CONF_MAX_FREQUENCY]:
        raise cv.Invalid(
            f"{CONF_MIN_FREQUENCY} must be lower than {CONF_MAX_FREQUENCY}"
        )
    return config


CONFIG_SCHEMA = cv.All(
    speaker.SPEAKER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(AudioSpectrum),
            cv.Required(CONF_OUTPUT_SPEAKER): cv.use_id(speaker.Speaker),
            cv.Optional(CONF_BANDS, default=20): cv.int_range(min=4, max=32),
            cv.Optional(CONF_MIN_FREQUENCY, default="60Hz"): cv.frequency,
            cv.Optional(CONF_MAX_FREQUENCY, default="16kHz"): cv.frequency,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ESP32]),
    _validate_frequency_range,
    _set_stream_limits,
)


def _final_validate(config: ConfigType) -> ConfigType:
    audio.final_validate_audio_schema(
        "audio_spectrum",
        audio_device=CONF_OUTPUT_SPEAKER,
        bits_per_sample=config[CONF_BITS_PER_SAMPLE],
        channels=config[CONF_NUM_CHANNELS],
        sample_rate=config[CONF_SAMPLE_RATE],
    )(config)
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)

    output = await cg.get_variable(config[CONF_OUTPUT_SPEAKER])
    cg.add(var.set_output_speaker(output))
    cg.add(var.set_bands(config[CONF_BANDS]))
    cg.add(
        var.set_frequency_range(
            config[CONF_MIN_FREQUENCY], config[CONF_MAX_FREQUENCY]
        )
    )
