import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import lock
from esphome.const import CONF_ID

# define YAML parameters for the retrofit lock
CONF_STEPPER_PINS = "stepper_pins"
CONF_STEPPER_DELAY = "delay_per_step"
CONF_STEPPER_STEPS_PER_REVOLUTION = "steps_per_revolution"
CONF_STEPPER_TURNS_PER_OPERATION = "turns_per_operation"
CONF_MFRC522_PINS = "mfrc522_pins"

retrofit_lock_ns = cg.esphome_ns.namespace('retrofit_lock')
RetrofitLock = retrofit_lock_ns.class_('RetrofitLock', lock.Lock, cg.Component)

# add parameters to the lock schema
CONFIG_SCHEMA = lock.LOCK_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(RetrofitLock),
    cv.Optional(CONF_STEPPER_PINS) : [cv.int_, cv.int_, cv.int_, cv.int_],
    cv.Optional(CONF_STEPPER_DELAY) : [cv.int_, cv.int_],
    cv.Optional(CONF_STEPPER_STEPS_PER_REVOLUTION) : cv.int_,
    cv.Optional(CONF_STEPPER_TURNS_PER_OPERATION) : cv.int_,
    cv.Optional(CONF_MFRC522_PINS) : [cv.int_, cv.int_, cv.int_, cv.int_, cv.int_]
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await lock.register_lock(var, config)

    # convert the retrofit lock parameters to code
    if CONF_STEPPER_PINS in config:
        cg.add(var.set_stepper_pins(config[CONF_STEPPER_PINS]))
    if CONF_STEPPER_DELAY in config:
        cg.add(var.set_stepper_delays(config[CONF_STEPPER_DELAY]))
    if CONF_STEPPER_STEPS_PER_REVOLUTION in config:
        cg.add(var.set_stepper_steps_per_revolution(config[CONF_STEPPER_STEPS_PER_REVOLUTION]))
    if CONF_STEPPER_TURNS_PER_OPERATION in config:
        cg.add(var.set_stepper_turns_per_operation(config[CONF_STEPPER_TURNS_PER_OPERATION]))
    if CONF_MFRC522_PINS in config:
        cg.add(var.set_rfid_pins(config[CONF_MFRC522_PINS]))
