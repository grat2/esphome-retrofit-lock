#include "esphome/core/log.h"
#include "retrofit_lock.h"
#include "retrofit_lock_secrets.h"

namespace esphome {
namespace retrofit_lock {

#define DEBUG

static const char *TAG = "retrofit_lock.lock";

void RetrofitLock::setup() {
   byte mfrcRegVal; 
   #ifdef DEBUG
      ESP_LOGD(TAG, "Init LED");
   #endif
   pinMode(LED_PIN, OUTPUT);

   #ifdef DEBUG
      ESP_LOGD(TAG, "Init stepper");
   #endif
   for(byte i = 0; i < 4; i++) {
      pinMode(this->stepperPins[i], OUTPUT);
   }
   this->totalSteps[0] = 0;
   this->totalSteps[1] = 0;
   this->motorBusy = false;
   this->currentStep = 0;
   this->lastUnlockTime = 0;
   this->stepDelayMicrosec = MAX_STEP_DELAY;
   
   #ifdef DEBUG
      ESP_LOGD(TAG, "Init MFRC522 RFID tag reader");
   #endif
   SPI.begin();
   this->rfid = MFRC522(SPI_CS_PIN, RC522_RST_PIN);
   this->rfid.PCD_Init();
   // read out the MFRC522 version
   mfrcRegVal = this->rfid.PCD_ReadRegister(this->rfid.VersionReg);
   #ifdef DEBUG
      ESP_LOGD(TAG, "Ver: 0x%x", mfrcRegVal);
   #endif
   // set up interrupts for the MFRC522
   pinMode(RC522_IRQ_PIN, INPUT_PULLUP);
   mfrcRegVal = 0xa0; // receive IRQ enable value
   this->rfid.PCD_WriteRegister(this->rfid.ComIEnReg, mfrcRegVal);
   // set the key A for RFID comms
   for(byte i = 0; i < 6; i++) 
      this->mfKey.keyByte[i] = 0xff;
   // interrupt is activated by ESPHome, so no attachInterrupt() call here

   #ifdef DEBUG
      ESP_LOGD(TAG, "Finished RetrofitLock setup!");
   #endif
}

void RetrofitLock::loop() {
   this->activateReception(); // I guess this needs to be regularly called
   if(this->motorBusy) 
      this->motorMove();
   else if(millis() - AUTO_LOCK_TIME > this->lastUnlockTime && this->state == lock::LOCK_STATE_UNLOCKED)
      this->lock();
}

void RetrofitLock::control(const lock::LockCall &call) {
   lock::LockState type = *call.get_state();
   if(this->motorBusy) {
      ESP_LOGW(TAG, "Motor is currently busy, please wait to execute your command!");
      return;
   }
   switch(type) {
      case lock::LOCK_STATE_LOCKED:
         this->motorInit(false);
         this->publish_state(type);
         break;
      case lock::LOCK_STATE_UNLOCKED:
         this->motorInit(true);
         this->lastUnlockTime = millis();
         this->publish_state(type);
         break;
      default:
         this->publish_state(lock::LOCK_STATE_NONE);
   }
}

void RetrofitLock::dump_config() {
   ESP_LOGCONFIG(TAG, "ESPHome Retrofit Electronic Lock V1.0");
}

void RetrofitLock::motorMove() {
   uint8_t queueStep;
   // give the motorMove function a certain period of time to do movement until we pass on priority to other
   // tasks/components
   uint32_t time = millis();
   while(millis() - time < MOTOR_MOVE_PERIOD_MS) {
      queueStep = this->direction[0] == 0 ? 1 : 0;
      if(this->stepDelayMicrosec > MIN_STEP_DELAY && this->totalSteps[queueStep] % 10 == 0)
         this->stepDelayMicrosec--;
      this->currentStep += this->direction[queueStep];
      for(uint8_t pin = 0; pin < 4; pin++) {
         digitalWrite(this->stepperPins[pin], 
            ELEGOO_STEP_SEQ_HALF[currentStep % 8][pin]);
      }
      this->totalSteps[queueStep]--;
      // if done with all the steps for this queue position, move to the next index by 
      // setting this index's direction to 0
      if(this->totalSteps[queueStep] == 0) {
         this->direction[queueStep] = 0;
         this->stepDelayMicrosec = MAX_STEP_DELAY;
      }
      // if done with all queue indexes, then set the motor to free and disengage motor
      if(this->direction[1] == 0) {
         this->motorBusy = false;
         for(uint8_t pin = 0; pin < 4; pin++) {
            digitalWrite(this->stepperPins[pin], 
               ELEGOO_STEP_SEQ_HALF[8][pin]);
         }
      }
      delayMicroseconds(this->stepDelayMicrosec);
   }
   /*this->totalSteps[0] = 0;
   this->totalSteps[1] = 0;
   this->motorBusy = false;*/
}

void RetrofitLock::motorInit(bool isUnlock) {
   /* clockwise = 1, counterclockwise = -1
    * Unlock steps: 
    *    counterclockwise rotate wheel from default position to move lever to "unlocked"
    *    clockwise rotate wheel to move back to default position
    * 
    * Lock steps: 
    *    clockwise rotate wheel from default position to move lever to "locked"
    *    counterclockwise rotate wheel to move back to default position
    */
   #ifdef DEBUG
      ESP_LOGD(TAG, "Initialize motor, isUnlock = %d", isUnlock);
   #endif
   if(this->motorBusy) {
      ESP_LOGW(TAG, "Motor is currently busy, wait to send another command!");
      return;
   }
   this->totalSteps[0] = LOCK_MOVE_RANGE_TURNS * STEPS_PER_REV;
   this->direction[0] = 1 - (2 * isUnlock);
   this->totalSteps[1] = LOCK_MOVE_RANGE_TURNS * STEPS_PER_REV;
   this->direction[1] = -1 + (2 * isUnlock);
   this->motorBusy = true;
}

void RetrofitLock::initCard() {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Init new card");
   #endif
}

void RetrofitLock::addCard(byte *uid, byte *hash) {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Add new card to database");
   #endif
}

void RetrofitLock::checkCardAccess() {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Check card access");
   #endif
   SHA256 secretHash;
   byte secretHashOut[32];
   
   // read in data from RFID card
   this->readData();

   // start hashing the data & salt with SHA256
   secretHash.reset();
   secretHash.update((void *)this->rfBuf, (size_t)SECRET_SIZE);
   secretHash.update((void *)SALT, sizeof(SALT));
   secretHash.finalize(secretHashOut, 32);

   // compare the fresh read data to our secrets database
   for(int card = 0; card < TOTAL_CARDS; card++) {
      if(dataEqual((byte *)this->rfid.uid.uidByte, (byte *)SECRET_UIDS[card], 4, 4) &&
         dataEqual(secretHashOut, (byte *)SECRET_HASHES[card], 32, 32))
         return;
   }
   
   // if we got here then the card we read doesn't match with anything in the database
   this->unlock();
   return;
}

void RetrofitLock::mfrc522Int() {
   this->rfid.PICC_ReadCardSerial();
   #ifdef DEBUG
      ESP_LOGD(TAG, "RFID ISR");
      this->dump_byte_array((byte *)this->rfid.uid.uidByte, (uint32_t)this->rfid.uid.size);
   #endif
   if(this->isCardInitOp) { // need to initialize a card, do those steps
      this->initCard();
   }
   else {
      this->checkCardAccess();
   }
   this->clearIntFlags();
   this->rfid.PICC_HaltA();
   this->rfid.PCD_StopCrypto1();
   this->activateReception();
}

bool RetrofitLock::checkMFRC522Error(const char *command, MFRC522::StatusCode status) {
   if(status != MFRC522::STATUS_OK) {
      ESP_LOGW(TAG, "%s() failed: %s", command, this->rfid.GetStatusCodeName(status));
      return true;
   }
   return false;
}

void RetrofitLock::readData() {
   byte buf[18];
   byte bufSize = sizeof(buf);
   MFRC522::StatusCode status;
   uint16_t currentByte = 0;

   // iterate through each sector
   for(byte sectorAddr = 2; sectorAddr < 16; sectorAddr++) {
      // authenticate using trailer block
      status = (MFRC522::StatusCode)this->rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 
         (sectorAddr*4) + 3, &this->mfKey, &this->rfid.uid);
      if(checkMFRC522Error("Authenticate", status))
         return;
      
      // iterate through user data blocks for the current sector
      for(byte blockAddr = sectorAddr * 4; blockAddr < (sectorAddr*4) + 3; blockAddr++) {
         status = (MFRC522::StatusCode)this->rfid.MIFARE_Read(blockAddr, buf, &bufSize);
         if(checkMFRC522Error("MIFARE_Read", status))
            return;

         // write from small buffer to main RFID buffer
         for(byte byteOffset = 0; byteOffset < 16; byteOffset++) {
            this->rfBuf[currentByte] = buf[byteOffset];
            currentByte++;
            if(currentByte >= SECRET_SIZE)
               return;
         }
      }
   }
}

void RetrofitLock::writeData() {
   byte buf[18];
   MFRC522::StatusCode status;
   uint16_t currentByte = 0;

   // iterate through each sector
   for(byte sectorAddr = 2; sectorAddr < 16; sectorAddr++) {
      // authenticate using trailer block
      status = (MFRC522::StatusCode)this->rfid.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 
         (sectorAddr*4) + 3, &this->mfKey, &this->rfid.uid);
      if(checkMFRC522Error("Authenticate", status))
         return;
      
      // iterate through user data blocks for the current sector
      for(byte blockAddr = sectorAddr * 4; blockAddr < (sectorAddr*4) + 3; blockAddr++) {
         // write from small buffer to main RFID buffer
         for(byte byteOffset = 0; byteOffset < 16; byteOffset++) {
            if(currentByte + byteOffset >= SECRET_SIZE)
               buf[byteOffset] = 0;
            else 
               buf[byteOffset] = this->rfBuf[currentByte + byteOffset];
         }

         status = (MFRC522::StatusCode)this->rfid.MIFARE_Write(blockAddr, buf, 16);
         if(checkMFRC522Error("MIFARE_Write", status))
            return;

         // increment currentByte counter by block size and check if we're done writing
         currentByte += 16;
         if(currentByte >= SECRET_SIZE)
            return;
      }
   }
}

void RetrofitLock::activateReception() {
   this->rfid.PCD_WriteRegister(this->rfid.FIFODataReg, this->rfid.PICC_CMD_REQA);
   this->rfid.PCD_WriteRegister(this->rfid.CommandReg, this->rfid.PCD_Transceive);
   this->rfid.PCD_WriteRegister(this->rfid.BitFramingReg, 0x87);
}

void RetrofitLock::clearIntFlags() {
   this->rfid.PCD_WriteRegister(this->rfid.ComIrqReg, 0x7F);
}

void RetrofitLock::dump_byte_array(byte *data, uint32_t size) {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Byte dump:");
      for(uint32_t i = 0; i < size; i++)
         ESP_LOGD(TAG, "data[%d] = %x", i, data[i]);
   #endif
}

bool RetrofitLock::dataEqual(byte *data1, byte *data2, size_t data1Size, size_t data2Size) {
   // if data blocks are different size, then they can't be equal
   if(data1Size != data2Size)
      return false;

   // compare each byte for equivalence, if just one doesn't match then they're not equal
   for(uint32_t i = 0; i < data1Size; i++) {
      if(data1[i] != data2[i])
         return false;
   }

   // if we've made it here, then the data has to be equal!
   return true;
}

}  // namespace retrofit_lock
}  // namespace esphome
