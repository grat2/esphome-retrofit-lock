#include "esphome/core/log.h"
#include "retrofit_lock.h"

namespace esphome {
namespace retrofit_lock {

#define DEBUG

static const char *TAG = "retrofit_lock.lock";

void RetrofitLock::setup() {
   byte mfrc_register_value; 
   #ifdef DEBUG
      ESP_LOGD(TAG, "Init LED");
   #endif
   pinMode(this->led_pin_, OUTPUT);

   #ifdef DEBUG
      ESP_LOGD(TAG, "Init stepper");
   #endif
   for(byte i = 0; i < 4; i++) {
      pinMode(this->stepper_pins_[i], OUTPUT);
   }
   this->current_step_delay_us_ = this->step_delays_us_[1]; // initialize to max delay
   
   #ifdef DEBUG
      ESP_LOGD(TAG, "Init MFRC522 RFID tag reader");
   #endif
   SPI.begin();
   this->rfid_ = MFRC522(this->rfid_mfrc522_pins_[0], this->rfid_mfrc522_pins_[4]);
   this->rfid_.PCD_Init();
   // read out the MFRC522 version
   mfrc_register_value = this->rfid_.PCD_ReadRegister(this->rfid_.VersionReg);
   #ifdef DEBUG
      ESP_LOGD(TAG, "Ver: 0x%x", mfrc_register_value);
   #endif
   mfrc_register_value = 0xa0; // receive IRQ enable value
   this->rfid_.PCD_WriteRegister(this->rfid_.ComIEnReg, mfrc_register_value);
   // set the key A for RFID comms
   for(byte i = 0; i < 6; i++) 
      this->mifare_key_.keyByte[i] = 0xff;

   // ESPHome binary_sensor handles interrupt pin init/detect/ISR-call, so no code for that here

   #ifdef DEBUG
      ESP_LOGD(TAG, "Finished RetrofitLock setup!");
   #endif
}

void RetrofitLock::loop() {
   this->activate_reception(); // I guess this needs to be regularly called
   if(this->motor_busy_) 
      this->motor_move();
   else if(millis() - this->auto_lock_delay_ms_ > this->last_unlock_time_ && 
      this->state == lock::LOCK_STATE_UNLOCKED)
      this->lock();
}

void RetrofitLock::control(const lock::LockCall &call) {
   lock::LockState type = *call.get_state();
   if(this->motor_busy_) {
      ESP_LOGW(TAG, "Motor is currently busy, please wait to execute your command!");
      return;
   }
   switch(type) {
      case lock::LOCK_STATE_LOCKED:
         this->motor_initialize(false);
         this->publish_state(type);
         break;
      case lock::LOCK_STATE_UNLOCKED:
         this->motor_initialize(true);
         this->publish_state(type);
         break;
      default:
         this->publish_state(lock::LOCK_STATE_NONE);
   }
}

void RetrofitLock::dump_config() {
   ESP_LOGCONFIG(TAG, "ESPHome Retrofit Electronic Lock V1.0");
}

void RetrofitLock::motor_move() {
   uint8_t queue_step;
   // give the motor_move function a certain period of time to do movement until we pass on priority to other
   // tasks/components
   uint32_t time = millis();
   while(millis() - time < this->motor_dedicated_move_time_ms_) {
      // get current spot in the queue
      queue_step = this->direction_[0] == 0 ? 1 : 0;

      // if current step delay is greater than the minimum step delay, decrement the current step delay
      if(this->current_step_delay_us_ > this->step_delays_us_[0] && this->total_steps_[queue_step] % 10 == 0)
         this->current_step_delay_us_--;

      // set the next step values on the pins then decrement the steps left for the current queue spot
      this->current_step_ += this->direction_[queue_step];
      for(uint8_t pin = 0; pin < 4; pin++) {
         digitalWrite(this->stepper_pins_[pin], 
            ELEGOO_STEP_SEQ_HALF[this->current_step_ % 8][pin]);
      }
      this->total_steps_[queue_step]--;

      // if done with all the steps for this queue position, move to the next index by 
      // setting this index's direction to 0
      if(this->total_steps_[queue_step] == 0) {
         // set unlock time here since this is where we finish the actual movement that unlocks the lever
         // (lever will be unlocked when the first half of the total "unlock" motor movement is done)
         if(this->direction_[0] == -1)
            this->last_unlock_time_ = millis();
         this->direction_[queue_step] = 0;
         this->current_step_delay_us_ = this->step_delays_us_[1];
      }

      // if done with all queue indexes, then set the motor to free and disengage motor
      if(this->direction_[1] == 0) {
         this->motor_busy_ = false;
         for(uint8_t pin = 0; pin < 4; pin++) {
            digitalWrite(this->stepper_pins_[pin], 
               ELEGOO_STEP_SEQ_HALF[8][pin]);
         }
      }

      delayMicroseconds(this->current_step_delay_us_);
   }
}

void RetrofitLock::motor_initialize(bool is_unlock) {
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
      ESP_LOGD(TAG, "Initialize motor, is_unlock = %d", is_unlock);
   #endif
   if(this->motor_busy_) {
      ESP_LOGW(TAG, "Motor is currently busy, wait to send another command!");
      return;
   }
   this->total_steps_[0] = this->turns_per_operation_ * this->steps_per_revolution_;
   this->direction_[0] = 1 - (2 * is_unlock);
   this->total_steps_[1] = this->turns_per_operation_ * this->steps_per_revolution_;
   this->direction_[1] = -1 + (2 * is_unlock);
   this->motor_busy_ = true;
}

void RetrofitLock::init_card() {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Init new card");
   #endif
   SHA256 hash;
   byte hashOut[32];
   byte tempBuf[SECRET_SIZE];

   // check to make sure you have a compatible card
   MFRC522::PICC_Type piccType = this->rfid_.PICC_GetType(this->rfid_.uid.sak);
   if(piccType != MFRC522::PICC_TYPE_MIFARE_1K) {
      ESP_LOGW(TAG, "This code specifically works on MIFARE Classic 1K cards only.");
      return;
   }

   // initialize random data in RF data buffer and in secondary temp buffer
   for(uint16_t i = 0; i < SECRET_SIZE; i++) {
      this->rf_buffer_[i] = random(256);
      tempBuf[i] = this->rf_buffer_[i];
   }
   
   // write data to the card and read it back to make sure it was written correctly
   // this->dump_byte_array(this->rf_buffer_, SECRET_SIZE);
   if(!this->write_data())
      return;
   if(!this->read_data())    // rf_buffer_ will have been modified with the read-back data here
      return;

   // verify read back data
   if(!this->data_equal(this->rf_buffer_, tempBuf, SECRET_SIZE, SECRET_SIZE)) {
      ESP_LOGW(TAG, "Data read back during init doesn't match data written to card!");
      return;
   }

   // hash the data with SHA256
   hash.reset();
   hash.update((void *)this->rf_buffer_, (size_t)SECRET_SIZE);
   hash.update((void *)SALT, sizeof(SALT));
   hash.finalize(hashOut, 32);

   // add card info to database
   #ifdef DEBUG
      ESP_LOGD(TAG, "Card initialized successfully! Adding to database...");
   #endif
   add_card(this->rfid_.uid.uidByte, hashOut);

   // clear card initialization flag
   this->is_card_init_operation = false;
}

void RetrofitLock::add_card(byte *uid, byte *hash) {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Add new card #%d to database", this->cards_stored_);
   #endif
   if(this->cards_stored_ >= TOTAL_CARDS) {
      ESP_LOGW(TAG, "Card database is full!");
      return;
   }

   // store card info into database
   for(uint8_t i = 0; i < 32; i++) {
      if(i < 4)
         this->uids_[this->cards_stored_][i] = uid[i];
      this->hashes_[this->cards_stored_][i] = hash[i];
   }
   this->cards_stored_++;
}

void RetrofitLock::check_card_access() {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Check card access");
   #endif
   SHA256 secretHash;
   byte secretHashOut[32];
   
   // read in data from RFID card
   if(!this->read_data())
      return;

   // start hashing the data & salt with SHA256
   secretHash.reset();
   secretHash.update((void *)this->rf_buffer_, (size_t)SECRET_SIZE);
   secretHash.update((void *)SALT, sizeof(SALT));
   secretHash.finalize(secretHashOut, 32);

   // compare the fresh read data to our secrets database
   for(int card = 0; card < TOTAL_CARDS; card++) {
      if(data_equal(this->rfid_.uid.uidByte, this->uids_[card], 4, 4) &&
         data_equal(secretHashOut, this->hashes_[card], 32, 32))
         this->unlock();
   }
   
   // if we got here then the card we read doesn't match with anything in the database
   return;
}

void RetrofitLock::mfrc522_interrupt() {
   this->rfid_.PICC_ReadCardSerial();
   #ifdef DEBUG
      ESP_LOGD(TAG, "RFID ISR");
      this->dump_byte_array((byte *)this->rfid_.uid.uidByte, (uint32_t)this->rfid_.uid.size);
   #endif
   if(this->is_card_init_operation) { // need to initialize a card, do those steps
      this->init_card();
   }
   else {
      this->check_card_access();
   }
   this->clear_interrupt_flags();
   this->rfid_.PICC_HaltA();
   this->rfid_.PCD_StopCrypto1();
   this->activate_reception();
}

bool RetrofitLock::check_mfrc522_error(const char *command, MFRC522::StatusCode status) {
   if(status != MFRC522::STATUS_OK) {
      ESP_LOGW(TAG, "%s() failed: %s", command, this->rfid_.GetStatusCodeName(status));
      return true;
   }
   return false;
}

bool RetrofitLock::read_data() {
   byte buf[18];
   byte bufSize = sizeof(buf);
   MFRC522::StatusCode status;
   uint16_t currentByte = 0;

   // iterate through each sector
   for(byte sectorAddr = 2; sectorAddr < 16; sectorAddr++) {
      // authenticate using trailer block
      status = (MFRC522::StatusCode)this->rfid_.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 
         (sectorAddr*4) + 3, &this->mifare_key_, &this->rfid_.uid);
      if(check_mfrc522_error("Authenticate", status))
         return false;
      
      // iterate through user data blocks for the current sector
      for(byte blockAddr = sectorAddr * 4; blockAddr < (sectorAddr*4) + 3; blockAddr++) {
         status = (MFRC522::StatusCode)this->rfid_.MIFARE_Read(blockAddr, buf, &bufSize);
         if(check_mfrc522_error("MIFARE_Read", status))
            return false;

         // write from small buffer to main RFID buffer
         for(byte byteOffset = 0; byteOffset < 16; byteOffset++) {
            this->rf_buffer_[currentByte] = buf[byteOffset];
            currentByte++;
            if(currentByte >= SECRET_SIZE)
               return true;
         }
      }
   }
   return true;
}

bool RetrofitLock::write_data() {
   byte buf[18];
   MFRC522::StatusCode status;
   uint16_t currentByte = 0;

   // iterate through each sector
   for(byte sectorAddr = 2; sectorAddr < 16; sectorAddr++) {
      // authenticate using trailer block
      status = (MFRC522::StatusCode)this->rfid_.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 
         (sectorAddr*4) + 3, &this->mifare_key_, &this->rfid_.uid);
      if(check_mfrc522_error("Authenticate", status))
         return false;
      
      // iterate through user data blocks for the current sector
      for(byte blockAddr = sectorAddr * 4; blockAddr < (sectorAddr*4) + 3; blockAddr++) {
         // write from small buffer to main RFID buffer
         for(byte byteOffset = 0; byteOffset < 16; byteOffset++) {
            if(currentByte + byteOffset >= SECRET_SIZE)
               buf[byteOffset] = 0;
            else 
               buf[byteOffset] = this->rf_buffer_[currentByte + byteOffset];
         }

         status = (MFRC522::StatusCode)this->rfid_.MIFARE_Write(blockAddr, buf, 16);
         if(check_mfrc522_error("MIFARE_Write", status))
            return false;

         // increment currentByte counter by block size and check if we're done writing
         currentByte += 16;
         if(currentByte >= SECRET_SIZE)
            return true;
      }
   }
   return true;
}

void RetrofitLock::activate_reception() {
   this->rfid_.PCD_WriteRegister(this->rfid_.FIFODataReg, this->rfid_.PICC_CMD_REQA);
   this->rfid_.PCD_WriteRegister(this->rfid_.CommandReg, this->rfid_.PCD_Transceive);
   this->rfid_.PCD_WriteRegister(this->rfid_.BitFramingReg, 0x87);
}

void RetrofitLock::clear_interrupt_flags() {
   this->rfid_.PCD_WriteRegister(this->rfid_.ComIrqReg, 0x7F);
}

void RetrofitLock::dump_byte_array(byte *data, uint32_t size) {
   #ifdef DEBUG
      ESP_LOGD(TAG, "Byte dump:");
      for(uint32_t i = 0; i < size; i++)
         ESP_LOGD(TAG, "data[%d] = %x", i, data[i]);
   #endif
}

bool RetrofitLock::data_equal(byte *data1, byte *data2, size_t data1Size, size_t data2Size) {
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

void RetrofitLock::set_stepper_pins(std::vector<int> pins) {
   this->stepper_pins_[0] = pins[0];
   this->stepper_pins_[1] = pins[1];
   this->stepper_pins_[2] = pins[2];
   this->stepper_pins_[3] = pins[3];
}

void RetrofitLock::set_stepper_delays(std::vector<int> delays) {
   this->step_delays_us_[0] = delays[0];
   this->step_delays_us_[1] = delays[1];
}

void RetrofitLock::set_stepper_steps_per_revolution(int steps_per_revolution) {
   this->steps_per_revolution_ = steps_per_revolution;
}

void RetrofitLock::set_stepper_turns_per_operation(int turns_per_operation) {
   this->turns_per_operation_ = turns_per_operation;
}

void RetrofitLock::set_rfid_pins(std::vector<int> pins) {
   this->rfid_mfrc522_pins_[0] = pins[0];
   this->rfid_mfrc522_pins_[1] = pins[1];
   this->rfid_mfrc522_pins_[2] = pins[2];
   this->rfid_mfrc522_pins_[3] = pins[3];
   this->rfid_mfrc522_pins_[4] = pins[4];
}

}  // namespace retrofit_lock
}  // namespace esphome
