#ifndef ___NODE__H___
#define ___NODE__H___

#include <stdint.h>
#include "Array.h"
#include "Memory.h"
#include "DMFControlBoard.h"

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


class Node {
public:
  DMFControlBoard &board_;

  Node(DMFControlBoard &board) : board_(board) {}

  uint32_t total_ram_size() { return ram_size(); }
  uint32_t ram_free() { return free_memory(); }

  /**************************************************************
   * METHODS
   **************************************************************/

  uint8_t update_channel(const uint16_t channel, const uint8_t state) {
    return board_.update_channel(channel, state);
  }
#if 0
  void update_all_channels() {
    board_.update_all_channels();
  }
  /* `get_state_of_all_channels`
   *
   * Return state of channels.
   *
   * ## TODO ##
   *
   * I/O multiplier switching board:
   *
   *  - Use 5 I2C requests to each switching board to load channel states.
   *
   * Arduino-based switching board:
   *
   *  - Use 5 I2C protocol buffer requests to each switching board to load
   *    channel states. */
  void get_state_of_all_channels();
#endif
  void send_spi(uint8_t pin, uint8_t address, uint8_t data) {
    board_.send_spi(pin, address, data);
  }
  int8_t set_series_resistor_index(uint8_t index, uint8_t channel) {
    return board_.set_series_resistor(channel, index);
  }
  int8_t series_resistor_index(uint8_t channel) {
    switch(channel) {
      case 0:
        return board_.A0_series_resistor_index_;
        break;
      case 1:
        return board_.A1_series_resistor_index_;
        break;
      default:
        return -1;
        break;
    }
  }
#if 0
  void load_config(bool use_defaults=false);
  void save_config();
#endif
  uint8_t set_adc_prescaler(const uint8_t index) {
    return board_.set_adc_prescaler(index);
  }

  uint8_t number_of_channels() const {
    return board_.number_of_channels_;
  }

  /* `get_state_of_all_channels`
   *
   * Return state of a single channel.
   *
   * ## TODO ##
   *
   * I/O multiplier switching board:
   *
   *  - Use I2C request to corresponding switching board to load channel
   *    state.
   *
   * Arduino-based switching board:
   *
   *  - Use I2C protocol buffer request to corresponding switching board to
   *    load channel state. */
  bool state_of_channel(uint16_t channel) const {
    return false;
  }

  float signal_waveform_voltage(uint8_t return_byte_index) { return board_.waveform_voltage(return_byte_index); }

  float set_signal_waveform_voltage(float vrms) {
    return board_.set_waveform_voltage(vrms);
  }

  float signal_waveform_frequency(uint8_t return_byte_index) { return board_.waveform_frequency(return_byte_index); }

  float set_signal_waveform_frequency(float frequency) {
    return board_.set_waveform_frequency(frequency);
  }

#ifdef AVR // only on Arduino Mega 2560
  float sampling_rate() const {
    return board_.SAMPLING_RATES_[board_.sampling_rate_index_];
  }

  void set_sampling_rate(uint8_t rate) { set_adc_prescaler(rate); }
#endif

  float series_resistance(uint8_t channel) {
    return board_.series_resistance(channel);
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

  float measure_impedance(uint16_t sampling_time_ms, uint16_t n_samples,
                          uint16_t delay_between_samples_ms) {
    board_.measure_impedance(sampling_time_ms, n_samples,
                             delay_between_samples_ms);
  }

  void reset_config_to_defaults() { board_.load_config(true); }

#if (___HARDWARE_MAJOR_VERSION___ == 1 && ___HARDWARE_MINOR_VERSION___ > 1) ||\
        ___HARDWARE_MAJOR_VERSION___ == 2
  uint8_t power_supply_pin() const {
    return board_.POWER_SUPPLY_ON_PIN_;
  }
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
