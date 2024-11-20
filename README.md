# esphome-retrofit-lock
An ESPHome component implementation of an electronic automatic lock retrofit for a front door deadbolt lever that also reads RFID tags from an MFRC522 reader.

## Installation

1. Navigate to the directory where your .yaml configuration files are stored
2. Copy all the contents of the repo into the directory
3. Modify the included .yaml to fit your specific device
4. Modify the motor drive code to fit your specific hardware
5. Profit!

## Usage

This component acts like a basic electronic lock when viewed in Home Assistant. It includes code that activates the motor when sending lock/unlock commands that can be modified depending on your specific hardware. All other functionality is self-contained with only basic changes required for your use case (ex: change pin numbers). 
