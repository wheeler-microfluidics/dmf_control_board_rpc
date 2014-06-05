/* Automatically generated nanopb header */
/* Generated by nanopb-0.2.8-dev at Wed Jun  4 09:20:26 2014. */

#pragma once

#include <pb.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _SignalGeneratorCommandType {
    SignalGeneratorCommandType_FORWARD_I2C_REQUEST = 1,
    SignalGeneratorCommandType_RAM_FREE = 10,
    SignalGeneratorCommandType_POT = 11,
    SignalGeneratorCommandType_WAVEFORM_FREQUENCY = 12,
    SignalGeneratorCommandType_WAVEFORM_VOLTAGE = 13,
    SignalGeneratorCommandType_I2C_ADDRESS = 14,
    SignalGeneratorCommandType_SET_POT = 15,
    SignalGeneratorCommandType_SET_WAVEFORM_FREQUENCY = 16,
    SignalGeneratorCommandType_SET_WAVEFORM_VOLTAGE = 17,
    SignalGeneratorCommandType_SET_I2C_ADDRESS = 18,
    SignalGeneratorCommandType_SET_HF_AMPLITUDE_CORRECTION = 19,
    SignalGeneratorCommandType_VOUT_PK_PK = 20,
    SignalGeneratorCommandType_LOAD_CONFIG = 21,
    SignalGeneratorCommandType_CONFIG_VERSION = 22
} SignalGeneratorCommandType;

/* Struct definitions */
typedef struct _SignalGeneratorI2cAddressRequest {
    uint8_t dummy_field;
} SignalGeneratorI2cAddressRequest;

typedef struct _SignalGeneratorLoadConfigResponse {
    uint8_t dummy_field;
} SignalGeneratorLoadConfigResponse;

typedef struct _SignalGeneratorRamFreeRequest {
    uint8_t dummy_field;
} SignalGeneratorRamFreeRequest;

typedef struct _SignalGeneratorSetHfAmplitudeCorrectionResponse {
    uint8_t dummy_field;
} SignalGeneratorSetHfAmplitudeCorrectionResponse;

typedef struct _SignalGeneratorSetI2cAddressResponse {
    uint8_t dummy_field;
} SignalGeneratorSetI2cAddressResponse;

typedef struct _SignalGeneratorSetPotResponse {
    uint8_t dummy_field;
} SignalGeneratorSetPotResponse;

typedef struct _SignalGeneratorVoutPkPkRequest {
    uint8_t dummy_field;
} SignalGeneratorVoutPkPkRequest;

typedef struct _SignalGeneratorWaveformFrequencyRequest {
    uint8_t dummy_field;
} SignalGeneratorWaveformFrequencyRequest;

typedef struct _SignalGeneratorWaveformVoltageRequest {
    uint8_t dummy_field;
} SignalGeneratorWaveformVoltageRequest;

typedef struct _SignalGeneratorConfigVersionRequest {
    uint32_t position;
} SignalGeneratorConfigVersionRequest;

typedef struct _SignalGeneratorConfigVersionResponse {
    uint32_t result;
} SignalGeneratorConfigVersionResponse;

typedef struct _SignalGeneratorForwardI2cRequestRequest {
    uint32_t address;
    pb_callback_t request;
} SignalGeneratorForwardI2cRequestRequest;

typedef struct _SignalGeneratorForwardI2cRequestResponse {
    int32_t result;
} SignalGeneratorForwardI2cRequestResponse;

typedef struct _SignalGeneratorI2cAddressResponse {
    uint32_t result;
} SignalGeneratorI2cAddressResponse;

typedef struct _SignalGeneratorLoadConfigRequest {
    bool use_defaults;
} SignalGeneratorLoadConfigRequest;

typedef struct _SignalGeneratorPotRequest {
    uint32_t index;
} SignalGeneratorPotRequest;

typedef struct _SignalGeneratorPotResponse {
    uint32_t result;
} SignalGeneratorPotResponse;

typedef struct _SignalGeneratorRamFreeResponse {
    uint32_t result;
} SignalGeneratorRamFreeResponse;

typedef struct _SignalGeneratorSetHfAmplitudeCorrectionRequest {
    float correction;
} SignalGeneratorSetHfAmplitudeCorrectionRequest;

typedef struct _SignalGeneratorSetI2cAddressRequest {
    uint32_t address;
} SignalGeneratorSetI2cAddressRequest;

typedef struct _SignalGeneratorSetPotRequest {
    uint32_t address;
    uint32_t level;
    bool save_to_eeprom;
} SignalGeneratorSetPotRequest;

typedef struct _SignalGeneratorSetWaveformFrequencyRequest {
    float frequency;
} SignalGeneratorSetWaveformFrequencyRequest;

typedef struct _SignalGeneratorSetWaveformFrequencyResponse {
    float result;
} SignalGeneratorSetWaveformFrequencyResponse;

typedef struct _SignalGeneratorSetWaveformVoltageRequest {
    float vrms;
} SignalGeneratorSetWaveformVoltageRequest;

typedef struct _SignalGeneratorSetWaveformVoltageResponse {
    float result;
} SignalGeneratorSetWaveformVoltageResponse;

typedef struct _SignalGeneratorVoutPkPkResponse {
    float result;
} SignalGeneratorVoutPkPkResponse;

typedef struct _SignalGeneratorWaveformFrequencyResponse {
    float result;
} SignalGeneratorWaveformFrequencyResponse;

typedef struct _SignalGeneratorWaveformVoltageResponse {
    float result;
} SignalGeneratorWaveformVoltageResponse;

typedef struct _SignalGeneratorCommandRequest {
    bool has_forward_i2c_request;
    SignalGeneratorForwardI2cRequestRequest forward_i2c_request;
    bool has_ram_free;
    SignalGeneratorRamFreeRequest ram_free;
    bool has_pot;
    SignalGeneratorPotRequest pot;
    bool has_waveform_frequency;
    SignalGeneratorWaveformFrequencyRequest waveform_frequency;
    bool has_waveform_voltage;
    SignalGeneratorWaveformVoltageRequest waveform_voltage;
    bool has_i2c_address;
    SignalGeneratorI2cAddressRequest i2c_address;
    bool has_set_pot;
    SignalGeneratorSetPotRequest set_pot;
    bool has_set_waveform_frequency;
    SignalGeneratorSetWaveformFrequencyRequest set_waveform_frequency;
    bool has_set_waveform_voltage;
    SignalGeneratorSetWaveformVoltageRequest set_waveform_voltage;
    bool has_set_i2c_address;
    SignalGeneratorSetI2cAddressRequest set_i2c_address;
    bool has_set_hf_amplitude_correction;
    SignalGeneratorSetHfAmplitudeCorrectionRequest set_hf_amplitude_correction;
    bool has_vout_pk_pk;
    SignalGeneratorVoutPkPkRequest vout_pk_pk;
    bool has_load_config;
    SignalGeneratorLoadConfigRequest load_config;
    bool has_config_version;
    SignalGeneratorConfigVersionRequest config_version;
} SignalGeneratorCommandRequest;

typedef struct _SignalGeneratorCommandResponse {
    bool has_forward_i2c_request;
    SignalGeneratorForwardI2cRequestResponse forward_i2c_request;
    bool has_ram_free;
    SignalGeneratorRamFreeResponse ram_free;
    bool has_pot;
    SignalGeneratorPotResponse pot;
    bool has_waveform_frequency;
    SignalGeneratorWaveformFrequencyResponse waveform_frequency;
    bool has_waveform_voltage;
    SignalGeneratorWaveformVoltageResponse waveform_voltage;
    bool has_i2c_address;
    SignalGeneratorI2cAddressResponse i2c_address;
    bool has_set_pot;
    SignalGeneratorSetPotResponse set_pot;
    bool has_set_waveform_frequency;
    SignalGeneratorSetWaveformFrequencyResponse set_waveform_frequency;
    bool has_set_waveform_voltage;
    SignalGeneratorSetWaveformVoltageResponse set_waveform_voltage;
    bool has_set_i2c_address;
    SignalGeneratorSetI2cAddressResponse set_i2c_address;
    bool has_set_hf_amplitude_correction;
    SignalGeneratorSetHfAmplitudeCorrectionResponse set_hf_amplitude_correction;
    bool has_vout_pk_pk;
    SignalGeneratorVoutPkPkResponse vout_pk_pk;
    bool has_load_config;
    SignalGeneratorLoadConfigResponse load_config;
    bool has_config_version;
    SignalGeneratorConfigVersionResponse config_version;
} SignalGeneratorCommandResponse;

/* Default values for struct fields */

/* Field tags (for use in manual encoding/decoding) */
#define SignalGeneratorConfigVersionRequest_position_tag        1
#define SignalGeneratorConfigVersionResponse_result_tag         1
#define SignalGeneratorForwardI2cRequestRequest_address_tag     1
#define SignalGeneratorForwardI2cRequestRequest_request_tag     2
#define SignalGeneratorForwardI2cRequestResponse_result_tag     1
#define SignalGeneratorI2cAddressResponse_result_tag            1
#define SignalGeneratorLoadConfigRequest_use_defaults_tag       1
#define SignalGeneratorPotRequest_index_tag                     1
#define SignalGeneratorPotResponse_result_tag                   1
#define SignalGeneratorRamFreeResponse_result_tag               1
#define SignalGeneratorSetHfAmplitudeCorrectionRequest_correction_tag 1
#define SignalGeneratorSetI2cAddressRequest_address_tag         1
#define SignalGeneratorSetPotRequest_address_tag                1
#define SignalGeneratorSetPotRequest_level_tag                  2
#define SignalGeneratorSetPotRequest_save_to_eeprom_tag         3
#define SignalGeneratorSetWaveformFrequencyRequest_frequency_tag 1
#define SignalGeneratorSetWaveformFrequencyResponse_result_tag  1
#define SignalGeneratorSetWaveformVoltageRequest_vrms_tag       1
#define SignalGeneratorSetWaveformVoltageResponse_result_tag    1
#define SignalGeneratorVoutPkPkResponse_result_tag              1
#define SignalGeneratorWaveformFrequencyResponse_result_tag     1
#define SignalGeneratorWaveformVoltageResponse_result_tag       1
#define SignalGeneratorCommandRequest_forward_i2c_request_tag   1
#define SignalGeneratorCommandRequest_ram_free_tag              10
#define SignalGeneratorCommandRequest_pot_tag                   11
#define SignalGeneratorCommandRequest_waveform_frequency_tag    12
#define SignalGeneratorCommandRequest_waveform_voltage_tag      13
#define SignalGeneratorCommandRequest_i2c_address_tag           14
#define SignalGeneratorCommandRequest_set_pot_tag               15
#define SignalGeneratorCommandRequest_set_waveform_frequency_tag 16
#define SignalGeneratorCommandRequest_set_waveform_voltage_tag  17
#define SignalGeneratorCommandRequest_set_i2c_address_tag       18
#define SignalGeneratorCommandRequest_set_hf_amplitude_correction_tag 19
#define SignalGeneratorCommandRequest_vout_pk_pk_tag            20
#define SignalGeneratorCommandRequest_load_config_tag           21
#define SignalGeneratorCommandRequest_config_version_tag        22
#define SignalGeneratorCommandResponse_forward_i2c_request_tag  1
#define SignalGeneratorCommandResponse_ram_free_tag             10
#define SignalGeneratorCommandResponse_pot_tag                  11
#define SignalGeneratorCommandResponse_waveform_frequency_tag   12
#define SignalGeneratorCommandResponse_waveform_voltage_tag     13
#define SignalGeneratorCommandResponse_i2c_address_tag          14
#define SignalGeneratorCommandResponse_set_pot_tag              15
#define SignalGeneratorCommandResponse_set_waveform_frequency_tag 16
#define SignalGeneratorCommandResponse_set_waveform_voltage_tag 17
#define SignalGeneratorCommandResponse_set_i2c_address_tag      18
#define SignalGeneratorCommandResponse_set_hf_amplitude_correction_tag 19
#define SignalGeneratorCommandResponse_vout_pk_pk_tag           20
#define SignalGeneratorCommandResponse_load_config_tag          21
#define SignalGeneratorCommandResponse_config_version_tag       22

/* Struct field encoding specification for nanopb */
extern const pb_field_t SignalGeneratorForwardI2cRequestRequest_fields[3];
extern const pb_field_t SignalGeneratorRamFreeRequest_fields[1];
extern const pb_field_t SignalGeneratorPotRequest_fields[2];
extern const pb_field_t SignalGeneratorWaveformFrequencyRequest_fields[1];
extern const pb_field_t SignalGeneratorWaveformVoltageRequest_fields[1];
extern const pb_field_t SignalGeneratorI2cAddressRequest_fields[1];
extern const pb_field_t SignalGeneratorSetPotRequest_fields[4];
extern const pb_field_t SignalGeneratorSetWaveformFrequencyRequest_fields[2];
extern const pb_field_t SignalGeneratorSetWaveformVoltageRequest_fields[2];
extern const pb_field_t SignalGeneratorSetI2cAddressRequest_fields[2];
extern const pb_field_t SignalGeneratorSetHfAmplitudeCorrectionRequest_fields[2];
extern const pb_field_t SignalGeneratorVoutPkPkRequest_fields[1];
extern const pb_field_t SignalGeneratorLoadConfigRequest_fields[2];
extern const pb_field_t SignalGeneratorConfigVersionRequest_fields[2];
extern const pb_field_t SignalGeneratorForwardI2cRequestResponse_fields[2];
extern const pb_field_t SignalGeneratorRamFreeResponse_fields[2];
extern const pb_field_t SignalGeneratorPotResponse_fields[2];
extern const pb_field_t SignalGeneratorWaveformFrequencyResponse_fields[2];
extern const pb_field_t SignalGeneratorWaveformVoltageResponse_fields[2];
extern const pb_field_t SignalGeneratorI2cAddressResponse_fields[2];
extern const pb_field_t SignalGeneratorSetPotResponse_fields[1];
extern const pb_field_t SignalGeneratorSetWaveformFrequencyResponse_fields[2];
extern const pb_field_t SignalGeneratorSetWaveformVoltageResponse_fields[2];
extern const pb_field_t SignalGeneratorSetI2cAddressResponse_fields[1];
extern const pb_field_t SignalGeneratorSetHfAmplitudeCorrectionResponse_fields[1];
extern const pb_field_t SignalGeneratorVoutPkPkResponse_fields[2];
extern const pb_field_t SignalGeneratorLoadConfigResponse_fields[1];
extern const pb_field_t SignalGeneratorConfigVersionResponse_fields[2];
extern const pb_field_t SignalGeneratorCommandRequest_fields[15];
extern const pb_field_t SignalGeneratorCommandResponse_fields[15];

/* Maximum encoded size of messages (where known) */
#define SignalGeneratorRamFreeRequest_size                      0
#define SignalGeneratorPotRequest_size                          6
#define SignalGeneratorWaveformFrequencyRequest_size            0
#define SignalGeneratorWaveformVoltageRequest_size              0
#define SignalGeneratorI2cAddressRequest_size                   0
#define SignalGeneratorSetPotRequest_size                       14
#define SignalGeneratorSetWaveformFrequencyRequest_size         5
#define SignalGeneratorSetWaveformVoltageRequest_size           5
#define SignalGeneratorSetI2cAddressRequest_size                6
#define SignalGeneratorSetHfAmplitudeCorrectionRequest_size     5
#define SignalGeneratorVoutPkPkRequest_size                     0
#define SignalGeneratorLoadConfigRequest_size                   2
#define SignalGeneratorConfigVersionRequest_size                6
#define SignalGeneratorForwardI2cRequestResponse_size           6
#define SignalGeneratorRamFreeResponse_size                     6
#define SignalGeneratorPotResponse_size                         6
#define SignalGeneratorWaveformFrequencyResponse_size           5
#define SignalGeneratorWaveformVoltageResponse_size             5
#define SignalGeneratorI2cAddressResponse_size                  6
#define SignalGeneratorSetPotResponse_size                      0
#define SignalGeneratorSetWaveformFrequencyResponse_size        5
#define SignalGeneratorSetWaveformVoltageResponse_size          5
#define SignalGeneratorSetI2cAddressResponse_size               0
#define SignalGeneratorSetHfAmplitudeCorrectionResponse_size    0
#define SignalGeneratorVoutPkPkResponse_size                    5
#define SignalGeneratorLoadConfigResponse_size                  0
#define SignalGeneratorConfigVersionResponse_size               6
#define SignalGeneratorCommandResponse_size                     90

#ifdef __cplusplus
} /* extern "C" */
#endif