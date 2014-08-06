/*
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
*/

#ifndef _DMF_CONTROL_BOARD_H_
#define _DMF_CONTROL_BOARD_H_

#include "Arduino.h"
#include "Wire.h"
#include "Config.h"
#include "RemoteObject.h"

#if (___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ > 1) ||\
        ___HARDWARE_MAJOR_VERSION___ == 2
#define ATX_POWER_SUPPLY
#endif


struct PeakToPeakMeasurement {
  uint8_t analog_pin_index_;
  int8_t resistor_index_;
  float reading_sum_;
  uint16_t reading_count_;
  uint16_t max_reading_;
  uint16_t min_reading_;
  bool saturated_;
  static const uint16_t SATURATION_THRESHOLD_READING = 820;

  PeakToPeakMeasurement(uint8_t analog_pin_index, int8_t resistor_index)
    : analog_pin_index_(analog_pin_index), resistor_index_(resistor_index),
      saturated_(false) {
    reset_extrema();
  }

  void reset_extrema() {
    min_reading_ = 1023;
    max_reading_ = 0;
    reading_count_ = 0;
    reading_sum_ = 0;
  }

  bool valid() const {
    return !(max_reading_ == 0 || (min_reading_ == 1023 && !saturated_));
  }

  uint16_t peak_to_peak() const { return max_reading_ - min_reading_; }

  uint16_t mean() const { return reading_sum_ / reading_count_; }

  bool update(uint16_t reading) {
    /* Return `true` if series resistor needs to be updated. */

    if (reading > SATURATION_THRESHOLD_READING) {
      /* The ADC is saturated, so use a smaller resistor and reset the peak. */
      if (resistor_index_ > 0) {
        reset_extrema();
        resistor_index_--;
        return true;
      } else {
        /* The ADC is still saturated using the lowest available resistor
         * value.  Mark measurement as saturated. */
        saturated_ = true;
        resistor_index_ = -1;
      }
    } else {
      /* Update local stats based on reading. */
      max_reading_ = (reading > max_reading_) ? reading : max_reading_;
      min_reading_ = (reading < min_reading_) ? reading : min_reading_;
      reading_sum_ += reading;
      reading_count_++;
    }
    return false;
  }
};


class DMFControlBoard : public RemoteObject {
public:
  static const uint8_t SINE = 0;
  static const uint8_t SQUARE = 1;

  /**\brief Address of config settings in persistent storage _(i.e., EEPROM)_.
   */
  static const uint16_t PERSISTENT_CONFIG_SETTINGS = 100;
  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ > 1
    static const uint8_t POWER_SUPPLY_ON_PIN_ = 8;
  #elif ___HARDWARE_MAJOR_VERSION___ == 2
    static const uint8_t POWER_SUPPLY_ON_PIN_ = 2;
  #endif
  static const int8_t RETURN_BAD_VALUE_S = -1;

  struct version_t {
    uint16_t major;
    uint16_t minor;
    uint16_t micro;
  };

  struct watchdog_t {
    /* # `watchdog_t` #
     *
     * This structure is used to maintain the state of a watchdog timer, which
     * can be used for any purpose.  For now, the watchdog timer is used to
     * maintain the state of the ATX power supply control signal.
     *
     * If the watchdog timer is enabled, _i.e., `enabled=true`, when
     * the timer period finishes:
     *
     *  - If the `state` is not `true`, the power-supply must be
     *   turned off.
     *  - The `state` must be set to `false`.
     *
     * When the watchdog timer is enabled, it is the responsibility of the
     * client to reset the watchdog-state before each timer period ends to
     * prevent the power supply from being turned off. */
    bool enabled;
    bool state;

    watchdog_t() : enabled(false), state(false) {}
  };

  struct ConfigSettings {
    /**\brief This is the software version that the persistent configuration
     * data was written with.*/
    version_t version;

    #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
      /**\brief This byte sets the value of the analog reference (between 0 and
      ~5V).*/
      uint8_t aref;
    #endif

    /**\brief i2c address of first high-voltage switching board.*/
    uint8_t switching_board_i2c_address;

    #if ___HARDWARE_MAJOR_VERSION___ == 1
      /**\brief This byte sets the maximum output voltage for the waveform
      generator. It should be trimmed so that the output waveform is 4Vp-p when
      POT_WAVEOUT_GAIN_2 is set to 255.*/
      uint8_t waveout_gain_1;

      /**\brief This byte sets the value of the virtual ground reference (between
      0 and 5V).*/
      uint8_t vgnd;

      /**\brief Series resistor values for channel 0.*/
      float A0_series_resistance[2];
      /**\brief Series capacitance values for channel 0.*/
      float A0_series_capacitance[2];
      /**\brief Series resistor values for channel 1.*/
      float A1_series_resistance[4];
      /**\brief Series capacitance values for channel 1.*/
      float A1_series_capacitance[4];
    #else  // #if ___HARDWARE_MAJOR_VERSION___ == 1
      /**\brief i2c address of first signal generator board.*/
      uint8_t signal_generator_board_i2c_address;
      /**\brief Series resistor values for channel 0.*/
      float A0_series_resistance[3];
      /**\brief Series capacitance values for channel 0.*/
      float A0_series_capacitance[3];
      /**\brief Series resistor values for channel 1.*/
      float A1_series_resistance[5];
      /**\brief Series capacitance values for channel 1.*/
      float A1_series_capacitance[5];
    #endif  // #if ___HARDWARE_MAJOR_VERSION___ == 1

    /**\brief Amplifier gain (if >0). If <=0, the gain is automatically
    adjusted based on the measured voltage from the amplifier.*/
    float amplifier_gain;

    /**\brief voltage tolerance for amplifier gain adjustment.*/
    float voltage_tolerance;

    uint8_t A0_series_resistor_count() const {
      return sizeof(A0_series_resistance) / sizeof(float);
    }
    uint8_t A1_series_resistor_count() const {
      return sizeof(A1_series_resistance) / sizeof(float);
    }
  };

  static const uint16_t MAX_SAMPLE_COUNT = 249;

  uint16_t most_recent_sample_count_;
  int16_t high_voltage_samples[MAX_SAMPLE_COUNT];
  int8_t high_voltage_resistor_indexes[MAX_SAMPLE_COUNT];
  int16_t feedback_voltage_samples[MAX_SAMPLE_COUNT];
  int8_t feedback_voltage_resistor_indexes[MAX_SAMPLE_COUNT];

  // TODO:
  //  Eventually, all of these variables should defined only on the arduino.
  //  The PC can interrogate device using `CMD_GET_NUMBER_OF_ADC_CHANNELS`.
  static const uint8_t NUMBER_OF_ADC_CHANNELS = 2;

  //////////////////////////////////////////////////////////////////////////////
  //
  // Return codes:
  //
  // Return codes are uint8_t.  Some generic return codes are defined by the
  // RemoteObject class.  You may also create custom return codes as long as
  // they are > 0x10.
  //
  //////////////////////////////////////////////////////////////////////////////

  DMFControlBoard();
  ~DMFControlBoard();

  void begin();

  void measure(PeakToPeakMeasurement &measurement);
  void update_amplifier_gain(PeakToPeakMeasurement &hv_measurement);
  // local accessors
  const char* protocol_name() { return PROTOCOL_NAME_; }
  const char* protocol_version() { return PROTOCOL_VERSION_; }
  const char* name() { return NAME_; } //device name
  const char* manufacturer() { return MANUFACTURER_; }
  const char* software_version() { return SOFTWARE_VERSION_; }
  const char* hardware_version();
  const char* url() { return URL_; }
  virtual void persistent_write(uint16_t address, uint8_t value);
#ifdef ATX_POWER_SUPPLY
  /* Note that the ATX power-supply output-enable is _active-low_. */
  void atx_power_on() const { digitalWrite(POWER_SUPPLY_ON_PIN_, LOW); }
  void atx_power_off() const { digitalWrite(POWER_SUPPLY_ON_PIN_, HIGH); }
  bool atx_power_state() const { return !digitalRead(POWER_SUPPLY_ON_PIN_); }
#endif  // ATX_POWER_SUPPLY
  bool watchdog_enabled() const { return watchdog_.enabled; }
  void watchdog_enabled(bool state) { watchdog_.enabled = state; }
  bool watchdog_state() const { return watchdog_.state; }
  void watchdog_state(bool state) { watchdog_.state = state; }
  void watchdog_reset() { watchdog_state(true); }

  /* Callback function when watchdog timer period is finished. */
  void watchdog_timeout() {
    /* If the watchdog timer is enabled, _i.e., `enabled=true`, when the timer
     * period finishes:
     *
     *  - If the `state` is not `true`, the power-supply must be turned off.
     *  - The `state` must be set to `false`.
     *
     * When the watchdog timer is enabled, it is the responsibility of the
     * client to reset the watchdog-state before each timer period ends to
     * prevent the power supply from being turned off. */
    if (watchdog_enabled()) {
      if (!watchdog_state()) {
        /* Watchdog timer has timed out and the state has not been reset since
         * the previous timeout, so execute error handler. */
        watchdog_error();
      } else {
        /* Watchdog timer has timed out, but the state has been reset.  Prepare
         * for next timeout event. */
        watchdog_state(false);
      }
    }
  }

  virtual void watchdog_error() const {
#ifdef ATX_POWER_SUPPLY
    atx_power_off();
#endif  // ATX_POWER_SUPPLY
  }

  /* Expose to allow timer callback to check state. */
  watchdog_t watchdog_;

  // private functions
  uint8_t update_channel(const uint16_t channel, bool state);
  void update_all_channels();
  void send_spi(uint8_t pin, uint8_t address, uint8_t data);
  uint8_t set_pot(uint8_t index, uint8_t value);
  uint8_t set_series_resistor(const uint8_t channel,
                              const uint8_t index);
  void load_config(bool use_defaults=false);
  void save_config();
  version_t config_version();
  float set_waveform_voltage(float output_vrms);
  float waveform_voltage();
  float set_waveform_frequency(float frequency);
  float waveform_frequency();
#if ___HARDWARE_MAJOR_VERSION___ == 1
  uint8_t waveform() const {
    return digitalRead(WAVEFORM_SELECT_);
  }

  int8_t set_waveform(uint16_t waveform_type) {
    if (waveform == SINE || waveform == SQUARE) {
      digitalWrite(WAVEFORM_SELECT_, waveform_type);
      return RETURN_OK;
    }
    return RETURN_BAD_VALUE_S;
  }
#endif  //#if ___HARDWARE_MAJOR_VERSION___ == 1
  uint16_t measure_impedance(uint16_t sampling_time_ms, uint16_t n_samples,
                             uint16_t delay_between_samples_ms);
  uint8_t set_adc_prescaler(const uint8_t index);
  float series_resistance(uint8_t channel) {
    switch(channel) {
      case 0:
        return config_settings_.A0_series_resistance
          [A0_series_resistor_index_];
        break;
      case 1:
        return config_settings_.A1_series_resistance
          [A1_series_resistor_index_];
        break;
      default:
        break;
    }
    return RETURN_BAD_VALUE_S;
  }

  int8_t set_series_resistance(uint8_t channel, float value) {
    switch(channel) {
      case 0:
        config_settings_.A0_series_resistance
          [A0_series_resistor_index_] = value;
        save_config();
        return RETURN_OK;
        break;
      case 1:
        config_settings_.A1_series_resistance
          [A1_series_resistor_index_] = value;
        save_config();
        return RETURN_OK;
        break;
      default:
        break;
    }
    return RETURN_BAD_INDEX;
  }

  float series_capacitance(uint8_t channel) {
    switch(channel) {
      case 0:
        return config_settings_.A0_series_capacitance[A0_series_resistor_index_];
        break;
      case 1:
        return config_settings_.A1_series_capacitance
          [A1_series_resistor_index_];
        break;
      default:
        break;
    }
    return RETURN_BAD_VALUE_S;
  }

  int8_t set_series_capacitance(uint8_t channel, float value) {
    switch(channel) {
      case 0:
        config_settings_.A0_series_capacitance
          [A0_series_resistor_index_] = value;
        save_config();
        return RETURN_OK;
        break;
      case 1:
        config_settings_.A1_series_capacitance
          [A1_series_resistor_index_] = value;
        save_config();
        return RETURN_OK;
        break;
      default:
        break;
    }
    return RETURN_BAD_INDEX;
  }

  void set_amplifier_gain(float value) {
    if (value > 0) {
      if (auto_adjust_amplifier_gain_) {
        amplifier_gain_ = value;
      } else {
        config_settings_.amplifier_gain = value;
        auto_adjust_amplifier_gain_ = false;
        save_config();
      }
    }
  }

  bool auto_adjust_amplifier_gain() const {
    return config_settings_.amplifier_gain <= 0;
  }

  void set_auto_adjust_amplifier_gain(bool value) {
    if (value) {
      config_settings_.amplifier_gain = 0;
    } else {
      config_settings_.amplifier_gain = amplifier_gain_;
    }
    /* Trigger enabling of auto-adjust amplifier gain based on rules in
     * `save_config`. */
    save_config();
  }

  //private members
  uint16_t number_of_channels_;
  uint8_t sampling_rate_index_;
  uint8_t A0_series_resistor_index_;
  uint8_t A1_series_resistor_index_;
  uint8_t peak_;
  float waveform_voltage_;
  float waveform_frequency_;
  float amplifier_gain_;
  bool auto_adjust_amplifier_gain_;
  ConfigSettings config_settings_;

  static const float SAMPLING_RATES_[];
private:
  // private static members
  static const char SOFTWARE_VERSION_[];
  static const char NAME_[];
  static const char HARDWARE_VERSION_[];
  static const char MANUFACTURER_[];
  static const char URL_[];
  static const char PROTOCOL_NAME_[];
  static const char PROTOCOL_VERSION_[];

  static const uint8_t AD5204_SLAVE_SELECT_PIN_ = 53; // digital pot

  #if ___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ < 3
    static const uint8_t POT_INDEX_AREF_ = 0;
  #endif

  #if ___HARDWARE_MAJOR_VERSION___ == 1
    static const uint8_t POT_INDEX_VGND_ = 1;
    static const uint8_t POT_INDEX_WAVEOUT_GAIN_1_ = 2;
    static const uint8_t POT_INDEX_WAVEOUT_GAIN_2_ = 3;
  #endif

  #if ___HARDWARE_MAJOR_VERSION___ == 1
    static const uint8_t WAVEFORM_SELECT_ = 9;
    static const uint8_t A0_SERIES_RESISTOR_0_ = 13;
    static const uint8_t A1_SERIES_RESISTOR_0_ = 12;
    static const uint8_t A1_SERIES_RESISTOR_1_ = 11;
    static const uint8_t A1_SERIES_RESISTOR_2_ = 10;
  #elif ___HARDWARE_MAJOR_VERSION___ == 2 && ___HARDWARE_MINOR_VERSION___ == 0
    static const uint8_t A0_SERIES_RESISTOR_0_ = 8;
    static const uint8_t A0_SERIES_RESISTOR_1_ = 9;
    static const uint8_t A1_SERIES_RESISTOR_0_ = 10;
    static const uint8_t A1_SERIES_RESISTOR_1_ = 11;
    static const uint8_t A1_SERIES_RESISTOR_2_ = 12;
    static const uint8_t A1_SERIES_RESISTOR_3_ = 13;
  #else // 2.1
    static const uint8_t A0_SERIES_RESISTOR_0_ = 4;
    static const uint8_t A0_SERIES_RESISTOR_1_ = 5;
    static const uint8_t A1_SERIES_RESISTOR_0_ = 6;
    static const uint8_t A1_SERIES_RESISTOR_1_ = 7;
    static const uint8_t A1_SERIES_RESISTOR_2_ = 8;
    static const uint8_t A1_SERIES_RESISTOR_3_ = 9;
  #endif

  // I2C bus
  // =======
  // A4 SDA
  // A5 SCL

  // SPI bus
  // =======
  // D10 SS
  // D11 MOSI
  // D13 SCK

  // PCA9505 (gpio) chip/register addresses
  static const uint8_t PCA9505_CONFIG_IO_REGISTER_ = 0x18;
  static const uint8_t PCA9505_OUTPUT_PORT_REGISTER_ = 0x08;

  // LTC6904 (programmable oscillator) chip address
  static const uint8_t LTC6904_ = 0x17;
};
#endif // _DMF_CONTROL_BOARD_H_
