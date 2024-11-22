#pragma once

namespace esphome {
namespace retrofit_lock {

// RFID encryption parameters
constexpr uint16_t SECRET_SIZE = 357;
constexpr byte SALT_ALT[] = "This is some salty salt";
constexpr byte SALT[] = "I love Michelle Tse!!!";

// SPI pins
constexpr uint8_t SPI_CS_PIN = 5;      // GPIO5
constexpr uint8_t SPI_SCLK_PIN = 18;   // GPIO18
constexpr uint8_t SPI_MOSI_PIN = 23;   // GPIO23
constexpr uint8_t SPI_MISO_PIN = 19;   // GPIO19

// MFRC522 comm pins
constexpr uint8_t RC522_RST_PIN = 26;  // GPIO26
constexpr uint8_t RC522_IRQ_PIN = 13;  // TCK (GPIO13)

// Elegoo stepper motor
constexpr uint8_t STEP_IN1 = 16;    // GPIO16
constexpr uint8_t STEP_IN2 = 17;    // GPIO17
constexpr uint8_t STEP_IN3 = 21;    // GPIO21
constexpr uint8_t STEP_IN4 = 22;    // GPIO22
constexpr uint16_t STEPS_PER_REV = 4096;
constexpr uint16_t LOCK_MOVE_RANGE_TURNS = 10;
constexpr uint32_t MIN_STEP_DELAY = 780;
constexpr uint32_t MAX_STEP_DELAY = 1000;
constexpr uint8_t ELEGOO_STEP_SEQ_HALF[9][4] = {
   {  0,    0,    0,    1 },
   {  0,    0,    1,    1 },
   {  0,    0,    1,    0 },
   {  0,    1,    1,    0 },
   {  0,    1,    0,    0 },
   {  1,    1,    0,    0 },
   {  1,    0,    0,    0 },
   {  1,    0,    0,    1 },
   {  0,    0,    0,    0 }
};

constexpr uint8_t ELEGOO_STEP_SEQ_FULL[5][4] {
   {  0,    0,    0,    1 },
   {  0,    0,    1,    0 },
   {  0,    1,    0,    0 },
   {  1,    0,    0,    0 },
   {  0,    0,    0,    0 }
};

// other
constexpr uint8_t LOCK_BUTTON = 36; // SVP (GPIO36)
constexpr uint8_t LED_PIN = 2;      // GPIO2
constexpr uint8_t TOTAL_CARDS = 5;
constexpr uint32_t AUTO_LOCK_TIME = 50000;      // time in ms to wait to automatically lock after unlock
constexpr uint32_t MOTOR_MOVE_PERIOD_MS = 25;   // time that is allocated to motor movements per loop iteration

}  // namespace retrofit_lock
}  // namespace esphome
