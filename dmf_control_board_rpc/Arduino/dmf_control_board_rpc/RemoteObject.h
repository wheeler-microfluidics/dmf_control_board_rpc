/**
RemoteObject:

This class implements a simple communication protocol between a PC and
Arduino over an RS-232 link using HDLC-like framing. It uses a wrapper
for the boost asio library (SimpleSerial) that provides an interface that
closely matches the Serial library on the Arduino. This allows sharing
of the bulk of the code between the PC (Windows, Linux or Mac) and the
Arduino. This implementation was partly inspired by Alvaro Lopes' serpro
project:

  https://github.com/alvieboy/arduino-serpro

Each packet has the following structure:

<pre>
+------------+---------+----------------+---------+------------+----------+
| Start Flag | Command | Payload Length | Payload |    CRC     | End Flag |
|   1 byte   | 1 byte  |    1-2 bytes   | N bytes |  2 bytes   |  1 byte  |
|    0x7E    |         |                |         | (optional) |   0x7E   |
+------------+---------+----------------+---------+------------+----------+
</pre>

Commands are uint8_t and should have the MSB=1 (replies have MSB=0). The
RemoteObject base class reserves the commands 0x80 to 0x9F, while derived
classes should restrict themselves to commands in the range 0xA0 to 0xFF.

The payload length can be one or two bytes.  If the payload is less than 128
bytes, it's length is expressed as a single byte.  If the most-significant
bit is set, the length is expressed as two bytes and can be recovered by
clearing the most significant byte (i.e. PAYLOAD_LENGTH & 0x7FFF).

<b>Examples:</b>
   - payload length of 3, one byte: 0x04
   - payload length of 512, two bytes: 0x82 0x01

Total packet length (not including flags) = Header Length (2-3 bytes)
                                            + Payload Length
                                            (+ 2 if CRC is enabled)

To use this class, you must derive a class based on it and reimplement the
virtual member function process_command().
*/
/*
__________________________________________________________________________

Copyright 2011 Ryan Fobel

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

__________________________________________________________________________
*/

#ifndef _REMOTE_OBJECT_H
#define	_REMOTE_OBJECT_H

#include <stdint.h>
#include "Deserialize.h"

#if !( defined(AVR) || defined(__SAM3X8E__) )
  #include "Logging.h"
  #include "SimpleSerial.h"
  #include <string>
  #include <boost/format.hpp>
#endif


class RemoteObject {
public:
#if !( defined(AVR) || defined(__SAM3X8E__) )
  static const uint32_t TIMEOUT_MILLISECONDS =   1000; // TODO: this should be configurable
#else
  static const uint8_t I2C_DELAY = 100; // delay between i2c write/reads
                                        // in future, we can avoid this by
                                        // adding interrupts on the
                                        // communication bus
#endif
  // Persistent storage _(e.g., EEPROM)_ addresses.
  static const uint16_t PERSISTENT_PIN_MODE_ADDRESS =       0;
  static const uint16_t PERSISTENT_PIN_STATE_ADDRESS =      7;
  static const uint16_t PERSISTENT_BAUD_RATE_ADDRESS =     15;
  static const uint16_t PERSISTENT_SERIAL_NUMBER_ADDRESS = 19;

  // reserved return codes
  static const uint8_t RETURN_OK =                      0x00;
  static const uint8_t RETURN_GENERAL_ERROR =           0x01;
  static const uint8_t RETURN_UNKNOWN_COMMAND =         0x02;
  static const uint8_t RETURN_TIMEOUT =                 0x03;
  static const uint8_t RETURN_NOT_CONNECTED =           0x04;
  static const uint8_t RETURN_BAD_INDEX =               0x05;
  static const uint8_t RETURN_BAD_PACKET_SIZE =         0x06;
  static const uint8_t RETURN_BAD_CRC =                 0x07;
  static const uint8_t RETURN_BAD_VALUE =               0x08;
  static const uint8_t RETURN_MAX_PAYLOAD_EXCEEDED =    0x09;

  static const char MCU_TYPE_[];

  RemoteObject();
  ~RemoteObject();

#if defined(AVR) || defined(__SAM3X8E__)
  float aref() { return aref_; }
  virtual void begin();
  void i2c_scan();

  void i2c_write(const uint8_t address, const uint8_t data);
  void i2c_write(const uint8_t address,
                 const uint8_t* data,
                 const uint8_t n_bytes);
  uint8_t i2c_read(const uint8_t address, uint8_t* data,
                   const uint8_t n_bytes_to_read);
  uint8_t i2c_send_command(uint8_t address,
                           uint8_t cmd,
                           uint8_t* data,
                           uint8_t delay_ms);
  /* The following two `persistent...` methods provide sub-classes a mechanism
   * to customize persistent storage.  For example, the Arduino DUE does not
   * support the `EEPROM` library used by the AVR chips. */
  virtual uint8_t persistent_read(uint16_t address);
  virtual void persistent_write(uint16_t address, uint8_t value);
#endif
private:
  float aref_;
};

#endif  // _REMOTE_OBJECT_H
