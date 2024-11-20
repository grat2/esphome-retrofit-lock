#pragma once

#include "esphome/core/component.h"
#include "esphome/components/lock/lock.h"
#include <SHA256.h>
#include <SPI.h>
#include <MFRC522.h>

namespace esphome {
namespace retrofit_lock {

class RetrofitLock : public lock::Lock, public Component {
   public:
      void setup() override;
      void loop() override;
      void control(const lock::LockCall &call);
      void dump_config() override;
      void motorMove(int steps, uint16_t turns);

   protected:
      MFRC522 rfid;
      MFRC522::MIFARE_Key mfKey;
      
};

} //namespace retrofit_lock
} //namespace esphome
