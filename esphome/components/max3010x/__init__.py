import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

CODEOWNERS = ["@topeju"]

CONF_HEARTRATE = "heart_rate"
CONF_OXIMETER = "oximeter"

max3010x_ns = cg.esphome_ns.namespace("max3010x")

CONFIG_SCHEMA_BASE = cv.Schema(
    {
        cv.Optional(CONF_HEARTRATE): sensor.sensor_schema(
            #unit_of_measurement=,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_OXIMETER): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
    }
).extend(cv.polling_component_schema("60s"))


async def to_code_base(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if heart_rate_config := config.get(CONF_HEARTRATE):
        sens = await sensor.new_sensor(heart_rate_config)
        cg.add(var.set_heart_rate_sensor(sens))

    if oximeter_config := config.get(CONF_OXIMETER):
        sens = await sensor.new_sensor(oximeter_config)
        cg.add(var.set_oximeter_sensor(sens))

    if temperature_config := config.get(CONF_TEMPERATURE):
        sens = await sensor.new_sensor(temperature_config)
        cg.add(var.set_temperature_sensor(sens))

    return var
