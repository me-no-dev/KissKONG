/*
  Copyright (c) 2017 Hristo Gochkov. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 3 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef KISSANDMSP_H_
#define KISSANDMSP_H_

#include "Arduino.h"

/*
 * MSP Defines
 * */

#define MSP_VERSION                 0
#define MSP_IDENT                   100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS                  101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU                 102   //out message         9 DOF
#define MSP_SERVO                   103   //out message         8 servos
#define MSP_MOTOR                   104   //out message         8 motors
#define MSP_RC                      105   //out message         8 rc chan and more
#define MSP_RAW_GPS                 106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS                107   //out message         distance home, direction home
#define MSP_ATTITUDE                108   //out message         2 angles 1 heading
#define MSP_ALTITUDE                109   //out message         altitude, variometer
#define MSP_ANALOG                  110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING               111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                     112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                     113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                    114   //out message         powermeter trig
#define MSP_MOTOR_PINS              115   //out message         which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES                116   //out message         the aux switch names
#define MSP_PIDNAMES                117   //out message         the PID names
#define MSP_WP                      118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS                  119   //out message         get the permanent IDs associated to BOXes
#define MSP_OSD                     220   //in message          starts epprom send to OSD GUI

#define MSP_SET_RAW_RC              200   //in message          8 rc chan
#define MSP_SET_RAW_GPS             201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID                 202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX                 203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING           204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION         205   //in message          no param
#define MSP_MAG_CALIBRATION         206   //in message          no param
#define MSP_SET_MISC                207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF              208   //in message          no param
#define MSP_SET_WP                  209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING          210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD                211   //in message          define a new heading hold direction

#define MSP_BIND                    240   //in message          no param
#define MSP_EEPROM_WRITE            250   //in message          no param

#define MSP_DEBUGMSG                253   //out message         debug string buffer
#define MSP_DEBUG                   254   //out message         debug1,debug2,debug3,debug4

#define MSP_OSD_NULL                0
#define MSP_OSD_READ_CMD            1
#define MSP_OSD_WRITE_CMD           2
#define MSP_OSD_FONT                3
#define MSP_OSD_SERIAL_SPEED        4
#define MSP_OSD_RESET               5

/*
 * KISS Defines
 * */
//#define KISS_ENABLE_DEBUG_CALLBACK

#define KISS_SET_SETTINGS           0x10
#define KISS_MOTOR_TEST             0x11
#define KISS_GET_TELEMETRY          0x20 // Telemetry Data
#define KISS_GET_INFO               0x21 // FC+ESC Info
#define KISS_ESC_INFO               0x22 // Call before GET_INFO to get ESC info
#define KISS_GET_SETTINGS           0x30 // FC Settings

//Roll, Pitch, Yaw and Aux 0-4 have range -1000 > 1000
#define KISS_AUX_IS_LOW(a)          ((a) <= -500)
#define KISS_AUX_IS_MID(a)          ((a) > -500 && (a) < 500)
#define KISS_AUX_IS_HIGH(a)         ((a) >= 500)

// Throttle has range 0 -> 1000
#define KISS_THROTTLE_IS_LOW(a)     ((a) <= 250)
#define KISS_THROTTLE_IS_MID(a)     ((a) > 250 && (a) < 750)
#define KISS_THROTTLE_IS_HIGH(a)    ((a) >= 750)

/*
 * Telemetry Data
 * */

typedef struct { //len 154
    int16_t throttle;       // v+1000
    int16_t roll;           // (v/2)+1500
    int16_t pitch;          // (v/2)+1500
    int16_t yaw;            // (v/2)+1500
    int16_t aux[4];         // (v/2)+1500
    uint8_t armed;          // boolean
    int16_t voltage;        // v/1000
    int16_t gyro[3];
    int16_t acc[3];
    int16_t angle[3];       // v/1000 *10
    int16_t i2c_errors;
    int16_t gyro_calib_done;
    uint8_t failsafe;
    uint16_t debug0;        // v/1000
    uint8_t found_rx;
    int16_t gyro_raw[3];    // v/1000
    int16_t acc_raw[3];     // v/1000
    int16_t acc_trim[2];    // v/1000
    int16_t acc_angle[2];   // v/1000
    uint8_t mode;           // { ACRO, LEVEL, 3D }
    uint16_t debug1;        // v/1000
    int16_t pwm_out[6];
    uint16_t debug2;        // v/1000
    uint8_t idle_time;
    struct {
        int16_t temperature;
        int16_t voltage;    // /100
        int16_t current;    // /100
        int16_t used_ah;    // /1000
        int16_t rpm;        // (v*32)/MAGNETPOLECOUNT
    } esc[6];
    uint8_t max_temperature;
    int16_t max_voltage;    // /100
    int16_t max_current;    // /10
    int16_t total_ah;       // /1000
    int16_t max_rpm;        // /10
    int16_t max_watt;
} kiss_telemetry_t;

/*
 * Settings Data
 * */

typedef enum {
    AUX_ARM, AUX_LEVEL, AUX_BUZZER, AUX_LED, AUX_3D
} kiss_aux_config_type_t;

typedef enum {
    AUX_OFF, AUX_LOW, AUX_LOW_MED, AUX_MED, AUX_MED_HIGH, AUX_HIGH
} kiss_aux_config_level_t;

typedef enum {
    AUX_NONE, AUX_1, AUX_2, AUX_3, AUX_4, AUX_5
} kiss_aux_config_channel_t;

typedef union {
    struct {
        kiss_aux_config_level_t level:4;
        kiss_aux_config_channel_t channel:4;
    };
    uint8_t value;
} kiss_aux_config_t;

typedef struct { // len 144
    uint16_t pid_p[3];      // v/1000
    uint16_t pid_i[3];      // v/1000
    uint16_t pid_d[3];      // v/1000
    uint16_t angle_p;       // v/1000
    uint16_t angle_i;       // v/1000
    uint16_t angle_d;       // v/1000
    int16_t acc_trim[2];    // v/1000
    int16_t rc_rate[3];     // v/1000
    int16_t rc_expo[3];     // v/1000
    int16_t rc_curve[3];    // v/1000
    int16_t rx_type;
    int16_t ppm_chan_order;
    int16_t copter_type;
    int16_t active_3d_mode;
    int16_t oneshot_125;    // output mode { PWM, OS125, OS42, DS150, DS300, DS600, DS1200 }
    int16_t min_command;    // v+1000
    int16_t mid_command;    // v+1000
    int16_t min_throttle;   // v+1000
    int16_t max_throttle;   // v+1000
    int16_t tri_mid;        // TYmid16 TriCopter Y mid
    uint8_t tri_inverted;   //TYinv8 TriCopter Y inverted
    int16_t acc_zero[3];
    kiss_aux_config_t aux[4];
    uint16_t max_angle;     // v/14.3
    uint8_t lpf;
    uint8_t serial_num[12];
    uint8_t version;
    uint16_t tpa[3];        // v/1000
    uint8_t oneshot_42;     // is zero
    uint8_t failsafe_seconds;
    uint8_t board_rotation;
    uint8_t is_active;
    uint8_t custom_tpa;
    uint8_t tpa_bp[2];
    uint8_t tpa_bpi[4];
    uint8_t voltage_influence;
    int16_t voltage[3];     // v/10
    uint8_t influence[3];
    uint8_t secret;
    uint8_t logger_config;
    uint8_t rgb[3];
    uint16_t vbat_alarm;    // v/10
    int16_t cbo[3];
    kiss_aux_config_t aux_4;
    uint8_t lap_timer_type;
    uint16_t transponder_id;
    uint8_t logger_debug_vars;
    uint8_t notch_filter_enable;
    uint16_t notch_filter_center;
    uint16_t notch_filter_cut;
    uint8_t yaw_c_filter;
} kiss_settings_t;

/*
 * Info Data
 * */

typedef struct { // len 91
    uint8_t count;
    struct {
        uint8_t serial_num[12];
        uint8_t version[2];
        uint8_t type;
    } esc[6];
} kiss_esc_info_t;

/*
 * Motor Test
 * */

typedef struct { // len 7
    uint8_t test_enabled;
    uint8_t motor_values[6];
} kiss_test_motors_t;

/*
 * MSP Direction and Callback
 * */

typedef enum {
    APP_TO_DEVICE, DEVICE_TO_APP
} msp_direction_t;

typedef void (msp_callback_t)(msp_direction_t, bool, uint8_t, uint8_t*, uint8_t);

#ifdef KISS_ENABLE_DEBUG_CALLBACK
typedef void (kiss_dbg_cb_t)(uint8_t, uint8_t*, uint8_t);
#endif

#ifdef __cplusplus
extern "C"{
#endif

// Init UART
void uart_init(uint32_t baud);

/*
 * MSP Methods
 * */

void msp_update();
void msp_on_packet(msp_callback_t * cb);
bool msp_send_cmd(msp_direction_t direction, bool err, uint8_t cmd, const uint8_t * data, uint8_t len);

/*
 * KISS Methods
 * */

bool kiss_set_settings(kiss_settings_t * settings);
bool kiss_get_settings(kiss_settings_t * settings);
bool kiss_get_telemetry(kiss_telemetry_t * data);
bool kiss_get_info(char * str, kiss_esc_info_t * info);
bool kiss_test_motors(kiss_test_motors_t * motors);

#ifdef KISS_ENABLE_DEBUG_CALLBACK
void kiss_set_dbg_cb(kiss_dbg_cb_t * cb);
#endif

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* KISSANDMSP_H_ */
