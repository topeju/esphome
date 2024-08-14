import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
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

DEPENDENCIES = ["i2c"]

CONF_HEARTRATE = "heart_rate"
CONF_OXIMETER = "oximeter"

CONF_MAX3010x_ID = "max3010x_id"

max3010x_ns = cg.esphome_ns.namespace("max3010x")

MAX3010xComponent = max3010x_ns.class_("MAX3010xComponent", cg.Component, i2c.I2CDevice)

MAX3010X_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_MAX3010x_ID): cv.use_id(MAX3010xComponent),
    }
)

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MAX3010xComponent),
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
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x57)),
    cv.has_exactly_one_key(CONF_HEARTRATE, CONF_OXIMETER),
)


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
