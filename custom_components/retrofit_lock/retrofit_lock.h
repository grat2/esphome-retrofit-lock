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
      void mfrc522_interrupt();            // main interrupt handler
      bool is_card_init_operation = false;      // flag set by an external button to initialize RFID cards

      // custom initializers for the component to take inputs from YAML config
      void set_stepper_pins(std::vector<int> pins);         // 4 integer values for the 4 stepper pins
      void set_stepper_delays(std::vector<int> delays);     // 2 delays for a range of delay/step, min and max
      void set_stepper_steps_per_revolution(int steps_per_revolution);
      void set_stepper_turns_per_operation(int turns_per_operation);
      void set_rfid_pins(std::vector<int> pins);            // 5 pins: {CS, SCLK, MOSI, MISO, MFRC522 RST}

   protected:
      // Elegoo stepper motor properties
      uint8_t stepper_pins_[4] = {16, 17, 21, 22};
      void motor_move();                  // single motor action for 10ms, called in loop()
      void motor_initialize(bool isUnlock);           // setup motor parameters
      uint32_t total_steps_[2] = {0, 0};  // step number queue
      int8_t direction_[2] = {0, 0};      // direction value queue
      bool motor_busy_ = false;           // flag to make sure motor is free for operations
      uint8_t current_step_ = 0;          // step sequence number from 0-7, 8 = disengage motor
      uint32_t step_delays_us_[2] = {720, 900};       // variable step delay, min/max values respectively
      uint32_t current_step_delay_us_;
      uint16_t steps_per_revolution_ = 4096;
      uint16_t turns_per_operation_ = 10;
      uint16_t auto_lock_delay_ms_ = 30000;
      // every loop() iteration, give the motor this amount of ms to perform any queued actions
      // (each lock/unlock requires two movements, so there's a queue length 2 for storing those)
      uint16_t motor_dedicated_move_time_ms_ = 25;

      // MFRC522 RFID properties/functions
      uint8_t rfid_mfrc522_pins_[5] = {5, 18, 23, 19, 26};     // {CS, SCLK, MOSI, MISO, MFRC522 RST}
      MFRC522 rfid_;
      MFRC522::MIFARE_Key mifare_key_;
      byte uids_[TOTAL_CARDS][4];         // UID values from Mifare 1K tags
      byte hashes_[TOTAL_CARDS][32];      // SHA256 hashes from secrets stored on RFID tags
      uint8_t cards_stored_ = 0;          // number of cards already stored in database
      byte rf_buffer_[SECRET_SIZE];
      bool read_data();                   // will write to rfBuf
      bool write_data();                  // requires data to write to be stored in rfBuf
      void add_card(byte *uid, byte *hash);        // save UID and SHA256 hash to database of active RFID tags
      void activate_reception();                   // commands for MFRC522 to activate reception
      void init_card();                   // write data to a card and call addCard()
      void check_card_access();           // checks if the current RFID card has access to unlock; unlocks if yes
      // checks if two blocks of data are equal
      bool data_equal(byte *data_1, byte *data_2, size_t data_1_size, size_t data_2_size); 
      void clear_interrupt_flags();       // clear internal MFRC522 interrupt flags
      // check status code for current operation
      bool check_mfrc522_error(const char *command, MFRC522::StatusCode status);

      // other properties
      uint32_t last_unlock_time_ = 0;
      uint8_t led_pin_ = 2;
};

} //namespace retrofit_lock
} //namespace esphome
