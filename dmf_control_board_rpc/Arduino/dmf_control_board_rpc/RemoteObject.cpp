/*
Copyright 2011 Ryan Fobel, 2013 Christian Fobel

This file is part of dmf_control_board.

dmf_control_board is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

dmf_control_board is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with dmf_control_board.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "RemoteObject.h"

#if defined(AVR)
  #include <util/crc16.h>
#endif
#if defined(AVR) || defined(__SAM3X8E__)
  #include "Arduino.h"
  #include "AdvancedADC.h"
  #include <Wire.h>
  #include <SPI.h>
  #include <OneWire.h>
  #ifdef AVR
    #include <EEPROM.h>
    #include <avr/eeprom.h>
  #elif defined(__SAM3X8E__)
    #include <DueFlashStorage.h>
    extern DueFlashStorage EEPROM;
  #endif
  extern "C" void __cxa_pure_virtual(void); // These declarations are needed for
  void __cxa_pure_virtual(void) {}          // virtual functions on the Arduino.
#else
  #include <string>
  #include <boost/thread.hpp>
  #include <boost/timer.hpp>
  #include <boost/date_time/posix_time/posix_time_types.hpp>
  #include <boost/format.hpp>
  #include <exception>
  using namespace std;
  using boost::format;
#endif


#ifdef __SAM3X8E__
  const char RemoteObject::MCU_TYPE_[] = "SAM3X8E";
#else
  const char RemoteObject::MCU_TYPE_[] = "ATmega2560";
#endif // __SAM3X8E__

RemoteObject::RemoteObject() {
  // Initialize pin mode and state of digital pins from persistent storage
  // _(i.e., EEPROM on AVR)_.
  for (uint8_t i = 0; i <= 54 / 8; i++) {
    uint8_t mode = persistent_read(PERSISTENT_PIN_MODE_ADDRESS + i);
    uint8_t state = persistent_read(PERSISTENT_PIN_STATE_ADDRESS + i);
    for (uint8_t j = 0; j < 8; j++) {
      if (i * 8 + j < 54) {
        pinMode(i * 8 + j, (~mode >> j) & 0x01);
        digitalWrite(i * 8 + j, (~state >> j) & 0x01);
      }
    }
  }
}

RemoteObject::~RemoteObject() {}

#if defined(AVR) || defined(__SAM3X8E__)
////////////////////////////////////////////////////////////////////////////////
//
// These functions are only defined on the Arduino
//
////////////////////////////////////////////////////////////////////////////////

void /* DEVICE */ RemoteObject::begin() {
  uint32_t baud_rate;
  for (uint8_t i = 0; i < 4; i++) {
    ((uint8_t*)&baud_rate)[i] = persistent_read(PERSISTENT_BAUD_RATE_ADDRESS
                                                + i);
  }
  // if the baud rate hasn't been set (default is that all bits are high)
  if (baud_rate == 0xFFFFFFFF) {
    baud_rate = 115200;
    // write the baud_rate to eeprom
    for (uint8_t i = 0; i < 4; i++) {
      persistent_write(PERSISTENT_BAUD_RATE_ADDRESS + i,
                       ((uint8_t*)&baud_rate)[i]);
    }
  }
  Serial.begin(baud_rate);
  Wire.begin();
  SPI.begin();
  aref_ = AdvancedADC.readVcc();
}

void /* DEVICE */ RemoteObject::i2c_scan() {
  for (uint8_t i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found i2c address: ");
      Serial.print(i, DEC);
      Serial.print(" (0x");
      Serial.print(i, HEX);
      Serial.println (")");
      delay(1);  // maybe unneeded?
    }
  }
}

void /* DEVICE */ RemoteObject::i2c_write(const uint8_t address,
                                          const uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(data);
    Wire.endTransmission();
}

void /* DEVICE */ RemoteObject::i2c_write(const uint8_t address,
                                          const uint8_t* data,
                                          const uint8_t n_bytes) {
    Wire.beginTransmission(address);
    for (uint8_t i = 0; i < n_bytes; i++) {
        Wire.write(data[i]);
    }
    Wire.endTransmission();
}

uint8_t /* DEVICE */ RemoteObject::i2c_read(const uint8_t address,
                                            uint8_t* data,
                                            const uint8_t n_bytes_to_read) {
    uint8_t n_bytes_read = 0;
    Wire.requestFrom(address, n_bytes_to_read);
    while (Wire.available()) {
        data[n_bytes_read++] = Wire.read();
    }
    return n_bytes_read;
}

uint8_t /* HOST */ RemoteObject::persistent_read(uint16_t address) {
    return EEPROM.read(address);
}

void /* HOST */ RemoteObject::persistent_write(uint16_t address,
                                               uint8_t value) {
    /* Use [`eeprom_update_byte`][1] instead of [`EEPROM.write`][2] _(which
     * uses [`eeprom_write_byte`][1])_ to avoid unnecessary write when byte has
     * not changed.
     *
     * __NB__ This likely slightly affects EEPROM write performance since a
     * read needs to be performed, followed by a write.
     *
     * [1]: http://www.nongnu.org/avr-libc/user-manual/group__avr__eeprom.html
     * [2]: http://arduino.cc/en/Reference/EEPROM */
    eeprom_update_byte((uint8_t *)address, value);
}
#endif
