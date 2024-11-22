#pragma once

#include "esphome/core/component.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/lock/lock.h"
#include <Crypto.h>
#include <SHA256.h>
#include <SPI.h>
#include <MFRC522.h>
#include "retrofit_lock_vars.h"


namespace esphome {
namespace retrofit_lock {

class RetrofitLock : public lock::Lock, public Component, public api::CustomAPIDevice {
   public:
      void setup() override;
      void loop() override;
      void control(const lock::LockCall &call);
      void dump_config() override;
      void dump_byte_array(byte *data, uint32_t size);      // used for debugging only
      void mfrc522Int();            // main interrupt handler

   protected:
      // Elegoo stepper motor properties
      uint8_t stepperPins[4] = {STEP_IN1, STEP_IN2, STEP_IN3, STEP_IN4};
      void motorMove();             // single motor action for 10ms, called in loop()
      void motorInit(bool isUnlock);      // setup motor parameters
      uint32_t totalSteps[2];       // step number queue
      int8_t direction[2];          // direction value queue
      bool motorBusy;               // flag to make sure motor is free for operations
      uint8_t currentStep;          // step sequence number from 0-7, 8 = disengage motor
      uint32_t stepDelayMicrosec;         // variable step delay

      // MFRC522 RFID properties/functions
      MFRC522 rfid;
      MFRC522::MIFARE_Key mfKey;
      bool isCardInitOp = false;
      byte uids[TOTAL_CARDS][4];       // UID values from Mifare 1K tags
      byte hashes[TOTAL_CARDS][32];    // SHA256 hashes from secrets stored on RFID tags
      byte rfBuf[SECRET_SIZE];
      void readData();              // will write to rfBuf
      void writeData();             // requires data to write to be stored in rfBuf
      void addCard(byte *uid, byte *hash);         // save UID and SHA256 hash to database of active RFID tags
      void activateReception();                    // commands for MFRC522 to activate reception
      void initCard();                 // write data to a card and call addCard()
      void checkCardAccess();          // checks if the current RFID card has access to unlock; unlocks if yes
      // checks if two blocks of data are equal
      bool dataEqual(byte *data1, byte *data2, size_t data1Size, size_t data2Size); 
      void clearIntFlags();                        // clear internal MFRC522 interrupt flags
      // check status code for current operation
      bool checkMFRC522Error(const char *command, MFRC522::StatusCode status);

      // other properties
      uint32_t lastUnlockTime;
};

} //namespace retrofit_lock
} //namespace esphome
