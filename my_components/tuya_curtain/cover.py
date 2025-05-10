import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import cover, uart, text_sensor
from esphome import automation, pins
from esphome.automation import maybe_simple_id
from esphome.const import (
    CONF_ID,
    CONF_UART_ID,
    CONF_DEVICE_CLASS,
    CONF_ICON,
    CONF_NAME,
    CONF_TEXT_SENSORS,
)

CODEOWNERS = ["@Brokly"]
DEPENDENCIES = ["cover", "uart"]
AUTO_LOAD = ["cover","text_sensor"]

CONF_PRODUCT_ID = 'product_id'
ICON_PRODUCT_ID = 'mdi:cog'
CONF_SHOW_PROTCOL = 'protocol_debug_level'
CONF_RESTORE = 'restore_position'
CONF_REFRESH_POS = 'refresh_position'
CONF_REVERSED = 'motor_reversed'
CONF_SAVE_TIMEOUT = 'save_timeout'
CONF_CALIBRATE_TIMEOUT = 'clibrate_timeout'
CONF_SILENT = 'silent'

tuya_curtain_ns = cg.esphome_ns.namespace("tuya_curtain")
TuyaCurtain = tuya_curtain_ns.class_("TuyaCurtain", cover.Cover, cg.Component)

LOG_LEVEL_SHOW_PROTO = {
    "NONE": 0,
    "ERROR": 1,
    "WARN": 2,
    "INFO": 3,
    "CONFIG": 4,
    "DEBUG": 5,
    "VERBOSE": 6,
    "VERY_VERBOSE": 7,
}



CONFIG_SCHEMA = cv.All(
    cover.COVER_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(TuyaCurtain),
            cv.Optional(CONF_REVERSED, default=False): cv.boolean,
            cv.Optional(CONF_RESTORE): cv.Schema(
              {
                cv.Optional(CONF_SAVE_TIMEOUT, default=20000): cv.positive_time_period_milliseconds,
                cv.Optional(CONF_CALIBRATE_TIMEOUT, default=20000): cv.positive_time_period_milliseconds,
                cv.Optional(CONF_SILENT, default=False): cv.boolean,
              }
            ),
            cv.Optional(CONF_REFRESH_POS): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SHOW_PROTCOL): cv.enum(LOG_LEVEL_SHOW_PROTO, upper=True),
            cv.Optional(CONF_PRODUCT_ID): text_sensor.text_sensor_schema(icon=ICON_PRODUCT_ID),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
)

async def to_code(config):

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    if CONF_UART_ID in config:
       parent = await cg.get_variable(config[CONF_UART_ID])
       cg.add(var.initCurtain(parent))
    else:
       raise cv.Invalid(
          f"Setting 'uart_id' is required !"
       )

    if CONF_PRODUCT_ID in config:
        cg.add(var.set_product_id_text(await text_sensor.new_text_sensor(config[CONF_PRODUCT_ID])))

    if config[CONF_RESTORE]: 
        settings = config[CONF_RESTORE]
        cg.add_define("TCURT_RESTORE",settings[CONF_SAVE_TIMEOUT])
        cg.add_define("TCURT_CALIBRATE_TIMEOT",settings[CONF_CALIBRATE_TIMEOUT])
        if settings[CONF_SILENT]==True:
           cg.add_define("TCURT_SILENT_CALIBRATE")

    if config[CONF_REFRESH_POS]: 
        cg.add_define("TCURT_VIRTUAL_POS",config[CONF_REFRESH_POS])

    if CONF_REVERSED in config:
        if config[CONF_REVERSED]:
           cg.add_define("TCURT_MOTOR_REVERS",1)
        else:
           cg.add_define("TCURT_MOTOR_REVERS",0)
  
    if CONF_SHOW_PROTCOL in config:
        cg.add_define("TCURT_PRINT_RAW_PROTO", config[CONF_SHOW_PROTCOL])
        
        
