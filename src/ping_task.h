#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

// https://docs.bluerobotics.com/ping-protocol/pingmessage-ping1d/

// SET COMMANDS

// 1004 set_ping_interval
    // u16 ping_interval in ms
typedef struct {
    uint16_t ping_interval_ms;
} ping_set_ping_interval_t;

// 1005 set_gain_setting
    // u32 gain_setting     "The current gain setting. 0: 0.6, 1: 1.8, 2: 5.5, 3: 12.9, 4: 30.2, 5: 66.1, 6: 144"
typedef struct {
    uint32_t gain_setting;
} ping_set_gain_setting_t;

// 1006 set_ping_enable
    // u8 ping_enable 0 = disabled, 1 = enabled
typedef struct {
    uint8_t ping_enable;
} ping_set_ping_enable_t;

// 1007 set_oss_profile_configuration
    // u16 number of points
    // u8 normalization_enabled
    // u8 enhance_enabled
typedef struct {
    uint16_t number_of_points;
    uint8_t normalization_enabled;
    uint8_t enhance_enabled;
} ping_set_oss_profile_configuration_t;


// GET RESPONSES

// 1200 firmware_version
    // u8 device_type
    // u8 device_model
    // u16 firmware_version_major
    // u16 firmware_version_minor
typedef struct {
    uint8_t device_type;
    uint8_t device_model;
    uint16_t firmware_version_major;
    uint16_t firmware_version_minor;
} ping_firmware_version_t;

// 1201 device_id
    // u8 device_id
typedef struct {
    uint8_t device_id;
} ping_device_id_t;

// 1202 voltage_5
    // u16 voltage_5 5v rail voltage in mV
typedef struct {
    uint16_t voltage_5;
} ping_voltage_5_t;

// 1203 speed_of_sound
    // u32 speed_of_sound in mm/s
typedef struct {
    uint32_t speed_of_sound;
} ping_speed_of_sound_t;

// 1204 range
    // u32 scan_start  mm
    // u32 scan_length  mm
typedef struct {
    uint32_t scan_start_mm;
    uint32_t scan_length_mm;
} ping_range_t;

// 1205 mode_auto
    // u8 mode_auto 0 = manual, 1 = auto
typedef struct {
    uint8_t mode_auto;
} ping_mode_auto_t;

// 1206 ping_interval
    // u16 ping_interval in ms
typedef struct {
    uint16_t ping_interval_ms;
} ping_interval_t;

// 1207 gain_setting
    // u32 gain_setting 
typedef struct {
    uint32_t gain_setting;
} ping_gain_setting_t;

// 1208 transmit_duration
    // u16 transmit_duration in us
typedef struct {
    uint16_t transmit_duration_us;
} ping_transmit_duration_t;

// 1210 general_info
    // u16 firmware_version_major
    // u16 firmware_version_minor
    // u16 voltage_5
    // u16 ping interval
    // u8 gain_setting
    // u8 mode_auto
typedef struct {
    uint16_t firmware_version_major;
    uint16_t firmware_version_minor;
    uint16_t voltage_5;
    uint16_t ping_interval_ms;
    uint8_t gain_setting;
    uint8_t mode_auto;
} ping_general_info_t;

// 1211 distance_simple
    // u32 distance in mm
    // u8 confidence in %
typedef struct {
    uint32_t distance_mm;
    uint8_t confidence;
    uint32_t timestamp;
} ping_distance_simple_t;

// 1212 distance
    // u32 distance_mm
    // u16 confidence  %
    // u16 transmit_duration  us
    // u32 ping_number
    // u32 scan_start  mm
    // u32 scan_length  mm 
    // u32 gain_setting
typedef struct {
    uint32_t distance_mm;
    uint16_t confidence;
    uint16_t transmit_duration_us;
    uint32_t ping_number;
    uint32_t scan_start_mm;
    uint32_t scan_length_mm;
    uint32_t gain_setting;
    uint32_t timestamp;
} ping_distance_t;

// 1213 processor_temperature
    // u16 processor_temperature in 100*degC
typedef struct {
    uint16_t processor_temperature;
} ping_processor_temperature_t;

// 1214 pcb_temperature
    // u16 pcb_temperature in 100*degC
typedef struct {
    uint16_t pcb_temperature;
} ping_pcb_temperature_t;

// 1215 ping_enable
    // u8 ping_enable 0 = disabled, 1 = enabled
typedef struct {
    uint8_t ping_enable;
} ping_enable_t;

// 1300 profile
    // u32 distance in mm
    // u16 confidence in %
    // u16 transmit_duration in us
    // u32 ping_number
    // u32 scan_start in mm
    // u32 scan_length in mm
    // u32 gain_setting
    // u16 profile_data_length
    // u8[] profile_data  "An array of return strength measurements 
                        // taken at regular intervals across the scan
                        // region. The first element is the closest
                        // measurement to the sensor, and the last element
                        // is the farthest measurement in the scanned range."
typedef struct {
    uint32_t distance_mm;
    uint16_t confidence;
    uint16_t transmit_duration_us;
    uint32_t ping_number;
    uint32_t scan_start_mm;
    uint32_t scan_length_mm;
    uint32_t gain_setting;
    uint16_t profile_data_length;
    uint8_t profile_data[512];
    uint32_t timestamp;
} ping_profile_t;

// 1301 oss_profile_configuration
    // u16 number of points
    // u8 normalization_enabled
    // u8 enhance_enabled
typedef struct {
    uint16_t number_of_points;
    uint8_t normalization_enabled;
    uint8_t enhance_enabled;
} ping_oss_profile_configuration_t;

// 1100 goto_bootloader
    // no payload

// 1400 continuous_start
    // u16 id
typedef struct {
    uint16_t id;
} ping_continuous_start_t;  

// 1401 continuous_stop
    // u16 id
typedef struct {
    uint16_t id;
} ping_continuous_stop_t;




// .CPP Public API
QueueHandle_t get_ping_queue();
void init_ping_task();