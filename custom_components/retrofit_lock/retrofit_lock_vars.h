#pragma once

namespace esphome {
namespace retrofit_lock {

// RFID encryption parameters
constexpr uint16_t SECRET_SIZE = 357;
constexpr byte SALT[] = "This is some salty salt";

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

constexpr uint8_t TOTAL_CARDS = 5;

}  // namespace retrofit_lock
}  // namespace esphome
