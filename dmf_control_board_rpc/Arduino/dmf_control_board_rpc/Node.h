#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include "Array.h"
#include "Memory.h"
#include "DMFControlBoard.h"
#include "custom_pb.h"
#include "AdvancedADC.h"
#include "FeedbackController.h"

/* # Arduino digital pins connected to shift registers #
 *
 * Note that both the _output-enable_ and _clear_ pins are _active low_,
 * meaning that setting the respective pin to logic `LOW` will trigger the
 * corresponding behavioural response.  For instance, driving the `SRCLR` pin
 * `LOW` will cause the shift register contents to be cleared, _i.e., set to
 * 0_. */
/* Output enable. */
#define OE            8
/* Clear. */
#define SRCLR         9


#ifndef DISABLE_I2C
/* Callback functions to respond to I2C communication. */
extern void i2c_receive_event(int byte_count);
extern void i2c_request_event();
#endif  // #ifndef DISABLE_I2C

#define PACKET_SIZE  1024

class Node {
public:
  DMFControlBoard &board_;
  FeedbackController feedback_controller_;
  uint8_t buffer[120];
  static const uint8_t MIN_I2C_ADDRESS = 0x05;
  static const uint8_t MAX_I2C_ADDRESS = 0x7F;

  Node(DMFControlBoard &board) : board_(board) {}

  UInt8Array i2c_scan() {
    UInt8Array result;
    result.data = &buffer[0];
    result.length = 0;

    for (uint8_t i = MIN_I2C_ADDRESS; i <= MAX_I2C_ADDRESS; i++) {
      Wire.beginTransmission(i);
      if (Wire.endTransmission() == 0) {
        result.data[result.length++] = i;
        delay(1);  // maybe unneeded?
      }
    }
    return result;
  }

  UInt8Array description_string(uint8_t key) {
    UInt8Array result;

    switch (key) {
      case DescriptionStrings_NAME:
        result.length = strlen(board_.name());
        result.data = (uint8_t *)(board_.name());
        break;
      case DescriptionStrings_HARDWARE_VERSION:
        result.length = strlen(board_.hardware_version());
        result.data = (uint8_t *)(board_.hardware_version());
        break;
      case DescriptionStrings_URL:
        result.length = strlen(board_.url());
        result.data = (uint8_t *)(board_.url());
        break;
      case DescriptionStrings_SOFTWARE_VERSION:
        result.length = strlen(board_.software_version());
        result.data = (uint8_t *)(board_.software_version());
        break;
      case DescriptionStrings_PROTOCOL_NAME:
        result.length = strlen(board_.protocol_name());
        result.data = (uint8_t *)(board_.protocol_name());
        break;
      case DescriptionStrings_PROTOCOL_VERSION:
        result.length = strlen(board_.protocol_version());
        result.data = (uint8_t *)(board_.protocol_version());
        break;
      case DescriptionStrings_MANUFACTURER:
        result.length = strlen(board_.manufacturer());
        result.data = (uint8_t *)(board_.manufacturer());
        break;
      default:
        result.data[0] = '\0';
        result.length = 0;
        break;
    }
    return result;
  }

  uint32_t total_ram_size() { return ram_size(); }
  uint32_t ram_free() { return free_memory(); }
  uint32_t packet_size() const { return PACKET_SIZE; }

  /**************************************************************
   * METHODS
   **************************************************************/

#if 0
  uint32_t high_voltage_samples_address() {
    return (uint32_t)(&board_.high_voltage_samples[0]);
  }

  uint16_t max_sample_count() const {
    return MAX_SAMPLE_COUNT;
  }

  Int16Array high_voltage_samples() {
    Int16Array result;
    result.data = &board_.high_voltage_samples[0];
    result.length = board_.most_recent_sample_count_;
    return result;
  }

  Int8Array high_voltage_resistor_indexes() {
    Int8Array result;
    result.data = &board_.high_voltage_resistor_indexes[0];
    result.length = board_.most_recent_sample_count_;
    return result;
  }

  Int16Array feedback_voltage_samples() {
    Int16Array result;
    result.data = &board_.feedback_voltage_samples[0];
    result.length = board_.most_recent_sample_count_;
    return result;
  }

  Int8Array feedback_voltage_resistor_indexes() {
    Int8Array result;
    result.data = &board_.feedback_voltage_resistor_indexes[0];
    result.length = board_.most_recent_sample_count_;
    return result;
  }
#endif

  void write_uint8(uint32_t address, UInt8Array array) {
    uint8_t *data = (uint8_t *)address;
    memcpy(data, array.data, array.length);
  }

  UInt8Array readn_uint8(uint32_t address, uint16_t count) {
    UInt8Array result;
    result.data = (uint8_t *)(address);
    result.length = count;
    return result;
  }

  void send_spi(uint8_t pin, uint8_t address, uint8_t data) {
    board_.send_spi(pin, address, data);
  }

  int8_t set_series_resistor_index(uint8_t channel, uint8_t index) {
    return feedback_controller_.set_series_resistor_index(channel, index);
  }

  int8_t series_resistor_index(uint8_t channel) {
    return feedback_controller_.series_resistor_index(channel);
  }

  void load_config(bool use_defaults) { board_.load_config(use_defaults); }

  void save_config() { board_.save_config(); }

  void set_adc_prescaler(const uint8_t index) {
    AdvancedADC.setPrescaler(index);
  }

#if 0
  float signal_waveform_voltage() const {
    return board_.waveform_voltage();
  }
#endif

  float set_signal_waveform_voltage(float vrms) {
    return board_.set_waveform_voltage(vrms);
  }

  float signal_waveform_frequency() { return board_.waveform_frequency(); }

  uint8_t signal_generator_board_i2c_address() {
    return board_.signal_generator_board_i2c_address();
  }

  void set_signal_generator_board_i2c_address(uint8_t address) {
    board_.set_signal_generator_board_i2c_address(address);
    board_.save_config();
  }

  float set_signal_waveform_frequency(float frequency) {
    return board_.set_waveform_frequency(frequency);
  }

#ifdef AVR // only on Arduino Mega 2560
  float sampling_rate() const {
    return AdvancedADC.samplingRate();
  }

  void set_sampling_rate(float rate) { AdvancedADC.setSamplingRate(rate); }
#endif

  float series_resistance(uint8_t channel) {
    return board_.config_settings_.series_resistance(
        channel,
        feedback_controller_.series_resistor_index(channel));
  }

  int8_t set_series_resistance(uint8_t channel, float value) {
    return board_.set_series_resistance(channel, value);
  }

  float series_capacitance(uint8_t channel) {
    return board_.series_capacitance(channel);
  }

  int8_t set_series_capacitance(uint8_t channel, float value) {
    return board_.set_series_capacitance(channel, value);
  }

  float amplifier_gain() const { return board_.amplifier_gain_; }

  void set_amplifier_gain(float value) { board_.set_amplifier_gain(value); }

  bool auto_adjust_amplifier_gain() const {
    return board_.auto_adjust_amplifier_gain();
  }

  void set_auto_adjust_amplifier_gain(bool value) {
    board_.set_auto_adjust_amplifier_gain(value);
  }

  UInt16Array config_version() {
    UInt16Array result;
    result.data = reinterpret_cast<uint16_t *>(&buffer[0]);
    DMFControlBoard::version_t config_version = board_.config_version();
    result.data[0] = config_version.major;
    result.data[1] = config_version.minor;
    result.data[2] = config_version.micro;
    result.length = 3;
    return result;
  }

  uint16_t measure_impedance(uint16_t n_samples_per_window,
                             uint16_t n_sampling_windows,
                             float delay_between_windows_ms,
                             bool interleave_samples, bool rms) {
    board_.reset_feedback_count();
    return feedback_controller_.measure_impedance(n_samples_per_window,
                                                  n_sampling_windows,
                                                  delay_between_windows_ms,
                                                  interleave_samples, rms);
  }

  UInt8Array feedback_readings(uint16_t offset) {
    UInt8Array result;
    if (offset < board_.feedback_count_) {
      uint16_t packet_samples = (PACKET_SIZE - 8) / sizeof(FeedbackReading);
      if (board_.feedback_count_ - offset < packet_samples) {
        packet_samples = board_.feedback_count_ - offset;
      }
      result.data = (uint8_t *)&board_.feedback_readings_[offset];
      result.length = packet_samples * sizeof(FeedbackReading);
    } else {
      result.data = NULL;
      result.length = 0;
    }
    return result;
  }

  void reset_config_to_defaults() { board_.load_config(true); }

#if (___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ > 1) ||\
        ___HARDWARE_MAJOR_VERSION___ == 2
  uint8_t power_supply_pin() const { return board_.POWER_SUPPLY_ON_PIN_; }
#endif

  bool watchdog_enabled() const { return board_.watchdog_enabled(); }
  void set_watchdog_enabled(bool value) const { board_.watchdog_enabled(value); }
  bool watchdog_state() const { return board_.watchdog_state(); }
  void set_watchdog_state(bool value) const { board_.watchdog_state(value); }
#ifdef ATX_POWER_SUPPLY
  bool atx_power_state() const { return board_.atx_power_state(); }
  void set_atx_power_state(bool value) const {
    if (value) {
      board_.atx_power_on();
    } else {
      board_.atx_power_off();
    }
  }
#endif  // ATX_POWER_SUPPLY
};


#endif  // #ifndef ___NODE__H___
