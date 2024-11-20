#include "esphome/core/log.h"
#include "retrofit_lock.h"

namespace esphome {
namespace retrofit_lock {

static const char *TAG = "retrofit_lock.lock";

void RetrofitLock::setup() {

}

void RetrofitLock::loop() {

}

void RetrofitLock::control(const lock::LockCall &call) {

}

void RetrofitLock::dump_config() {
   ESP_LOGCONFIG(TAG, "ESPHome Retrofit Electronic Lock V1.0");
}

void RetrofitLock::motorMove(int steps, uint16_t turns) {

}

}  // namespace retrofit_lock
}  // namespace esphome
