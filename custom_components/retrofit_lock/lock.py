import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import lock
from esphome.const import CONF_ID

retrofit_lock_ns = cg.esphome_ns.namespace('retrofit_lock')
RetrofitLock = retrofit_lock_ns.class_('RetrofitLock', lock.Lock, cg.Component)

CONFIG_SCHEMA = lock.LOCK_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(RetrofitLock)
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await lock.register_lock(var, config)
