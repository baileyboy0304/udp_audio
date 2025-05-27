import esphome.codegen as cg
from esphome.components import speaker
import esphome.config_validation as cv
from esphome.const import (
    CONF_BITS_PER_SAMPLE,
    CONF_ID,
    CONF_SAMPLE_RATE,
    PLATFORM_ESP32
)
from esphome.components import switch

CONF_IP_ADDRESS = "ip_address"
CONF_PORT = "port"
CONF_TAG_ENABLE_SWITCH = "tag_enable"

AUTO_LOAD = ["audio"]
CODEOWNERS = ["@jesserockz", "@kahrendt"]

udp_audio_ns = cg.esphome_ns.namespace("udp_audio")
UDPAudio = udp_audio_ns.class_(
    "UDPAudio", cg.Component, speaker.Speaker
)


CONFIG_SCHEMA = cv.All(
    speaker.SPEAKER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(UDPAudio),
            cv.Optional(CONF_SAMPLE_RATE, default=16000): cv.int_,
            cv.Optional(CONF_BITS_PER_SAMPLE, default=16): cv.int_,
            cv.Optional(CONF_IP_ADDRESS, default="192.168.1.137"): cv.string,
            cv.Optional(CONF_PORT, default=6056): cv.port,
            cv.Optional(CONF_TAG_ENABLE_SWITCH): cv.use_id(switch.Switch),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ESP32]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await speaker.register_speaker(var, config)

    # Convert IP address to string before passing to C++
    ip_address_str = str(config[CONF_IP_ADDRESS])
    cg.add(var.set_ip_address(ip_address_str))
    cg.add(var.set_port(config[CONF_PORT]))

    #cg.add(var.set_tag_enable(config[CONF_TAG_ENABLE]))
    if CONF_TAG_ENABLE_SWITCH in config:
        tag_switch = await cg.get_variable(config[CONF_TAG_ENABLE_SWITCH])
        cg.add(var.set_tag_enable_switch(tag_switch))
