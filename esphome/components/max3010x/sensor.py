import esphome.codegen as cg
import esphome.config_validation as cv
#from esphome import pins
from esphome.components import i2c, sensor#, binary_sensor
from esphome.const import (
    CONF_ID,
    CONF_PIN,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    #ENTITY_CATEGORY_DIAGNOSTIC,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

CODEOWNERS = ["@topeju"]

DEPENDENCIES = ["i2c"]

CONF_HEARTRATE = "heart_rate"
CONF_OXIMETER = "oximeter"
CONF_IRQ = "irq"

CONF_MAX3010x_ID = "max3010x"

max3010x_ns = cg.esphome_ns.namespace("max3010x")

MAX3010xComponent = max3010x_ns.class_("MAX3010xComponent", cg.PollingComponent, i2c.I2CDevice)#, binary_sensor.BinarySensor)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MAX3010xComponent),
            #cv.Optional(CONF_IRQ): binary_sensor.binary_sensor_schema().extend({cv.Required(CONF_PIN): pins.gpio_input_pin_schema}),
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
                #entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
        }
    )
    .extend(cv.polling_component_schema("0.1s"))
    .extend(i2c.i2c_device_schema(0x57))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    #pin = await cg.gpio_pin_expression(config[CONF_IRQ])
    #cg.add(var.set_interrupt_pin(pin))

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
