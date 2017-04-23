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
#include "KissAndMsp.h"
/*
 * KISS Variables
 * */

//Big to Little
#define swapI16(a) (int16_t)((uint16_t)(a)>>8|(uint16_t)(a)<<8)
#define swapU16(a) (uint16_t)((uint16_t)(a)>>8|(uint16_t)(a)<<8)
#define ttInt(v) data->v = swapI16(data->v)
#define ttUint(v) data->v = swapU16(data->v)
#define cpInt(v) data_out->v = swapI16(data_in->v)
#define cpUint(v) data_out->v = swapU16(data_in->v)
#define cpByte(v) data_out->v = data_in->v

#define KISS_MAGIC      0x05 //data packet starts with this byte
#define KISS_BUFFER_LEN 255  //uart buffer to hold the incomming data
#define KISS_RX_TIMEOUT 50   //time in ms for response to be received from FC

//KISS Set Settings Packet
typedef struct { // len is 153, but output 161
    uint16_t pid_p[3];  // v*1000
    uint16_t pid_i[3];  // v*1000
    uint16_t pid_d[3];  // v*1000
    uint16_t angle_p;   // v*1000
    uint16_t angle_i;   // v*1000
    uint16_t angle_d;   // v*1000
    int16_t acc_trim[2];// v*1000
    int16_t rc_rate[3]; // v*1000
    int16_t rc_expo[3]; // v*1000
    int16_t rc_curve[3];// v*1000
    int16_t rx_type;
    int16_t ppm_chan_order;
    int16_t copter_type;
    int16_t active_3d_mode;
    int16_t oneshot_125;
    int16_t min_command;  // v-1000
    int16_t mid_command;  // v-1000
    int16_t min_throttle; // v-1000
    int16_t max_throttle; // v-1000
    int16_t tri_mid; // TYmid16
    uint8_t tri_inverted; //TYinv8
    int16_t acc_zero[3];
    kiss_aux_config_t aux[4];
    uint16_t max_angle;   // v*14.3
    uint8_t lpf;
    uint16_t tpa[3];      // v*1000
    uint8_t oneshot_42;
    uint8_t failsafe_seconds;
    // v100 93
    uint8_t activation_key[4];
    uint8_t board_rotation;
    // v101 110
    uint8_t custom_tpa;
    uint8_t tpa_bp[2];
    uint8_t tpa_bpi[4];
    uint8_t voltage_influence;
    int16_t voltage[3];   // v*10
    uint8_t influence[3];
    // v102 112
    uint8_t vtx_channel;
    uint8_t logger_config;
    // v103 136
    kiss_led_color_t rgb;
    uint16_t vbat_alarm;  // v*10
    int16_t cbo[3];
    kiss_aux_config_t aux_4;
    uint8_t lap_timer_type;
    uint16_t transponder_id;
    uint8_t logger_debug_vars;
    // v104 147
    kiss_notch_filter_t notch_filter[2];
    uint8_t yaw_c_filter;
    // v106 161
    uint8_t vtx_type;
    uint16_t vtx_power_low;
    uint16_t vtx_power_high;
    kiss_aux_config_t aux567[3];
    uint16_t mah_alarm;
    uint8_t deadband[3];
    uint8_t motor_buzzer;
} kiss_set_settings_t;

//KISS parse states
typedef enum {
    KISS_IDLE,  // discard any RX data
    KISS_CMD,   // sending request. discard any RX data
    KISS_START, // expect KISS_MAGIC
    KISS_LEN,   // expect data length
    KISS_DATA,  // receive data
    KISS_CHKSUM,// expect data checksum
    KISS_DONE,  // packet received
    KISS_ERROR  // packet receive failed
} kiss_packet_state_t;

static volatile kiss_packet_state_t _kiss_rx_state = KISS_IDLE;
static volatile uint8_t _kiss_rx_cmd = 0;
static volatile uint8_t _kiss_rx_buffer[KISS_BUFFER_LEN];
static volatile uint8_t _kiss_rx_len = 0;
static volatile uint8_t _kiss_rx_index = 0;
static volatile uint32_t _kiss_rx_checksum = 0;
static volatile uint32_t _kiss_rx_started_at = 0;

#ifdef KISS_ENABLE_DEBUG_CALLBACK
static kiss_dbg_cb_t * kiss_dbg_cb = NULL;
void kiss_set_dbg_cb(kiss_dbg_cb_t * cb){
    kiss_dbg_cb = cb;
}
#endif
/*
 * MSP Variables
 * */

#define MSP_BUFFER_LEN 64 //uart buffer to hold the incomming data
#define MSP_RX_TIMEOUT 10 //time in ms for packet to be received

typedef enum {
    MSP_START, MSP_M, MSP_ERR, MSP_LEN, MSP_CMD, MSP_DATA, MSP_CS, MSP_DONE
} msp_rx_state_t;

static  msp_rx_state_t _msp_rx_state = MSP_START;
static  msp_callback_t * _msp_rx_cb = NULL;
static  uint8_t * _msp_rx_buf = NULL;

static bool _msp_rx_err = false;
static uint8_t _msp_rx_cmd = 0;
static uint8_t _msp_rx_len = 0;
static uint8_t _msp_rx_index = 0;
static uint8_t _msp_rx_checksum = 0;
static uint32_t _msp_rx_started_at = 0;
static msp_direction_t _msp_rx_direction = APP_TO_DEVICE;

/*
 * Forward Declarations
 * */

static bool _can_send();
static void _uart_tx_byte(uint8_t data);

/*
 * KISS Private Methods
 * */

//feed UART data through interrupt
static void _kiss_rx_byte(uint8_t data){
    if(_kiss_rx_state == KISS_START){
        if(data != KISS_MAGIC){
            _kiss_rx_state = KISS_ERROR;
        } else {
            _kiss_rx_state = KISS_LEN;
        }
    } else if(_kiss_rx_state == KISS_LEN){
        _kiss_rx_len = data;
        _kiss_rx_index = 0;
        _kiss_rx_checksum = 0;
        _kiss_rx_state = KISS_DATA;
    } else if(_kiss_rx_state == KISS_DATA){
        _kiss_rx_buffer[_kiss_rx_index++] = data;
        _kiss_rx_checksum += data;
        if(_kiss_rx_index == _kiss_rx_len){
            _kiss_rx_state = KISS_CHKSUM;
        }
    } else if(_kiss_rx_state == KISS_CHKSUM){
        _kiss_rx_checksum /= _kiss_rx_len;
        if(data == _kiss_rx_checksum){
            _kiss_rx_state = KISS_DONE;
        } else {
            _kiss_rx_state = KISS_ERROR;
        }
    }
}

//check if transmission is currently running
static bool _kiss_can_send(){
    if(_kiss_rx_state > KISS_CMD && _kiss_rx_state < KISS_DONE && (millis() - _kiss_rx_started_at) > KISS_RX_TIMEOUT){
        _kiss_rx_state = KISS_ERROR;
        return false;
    }
    return _kiss_rx_state == KISS_IDLE;
}

//send packet to the flight controller
static bool _kiss_send_cmd(uint8_t cmd, const uint8_t * data_in, uint8_t len_in, uint8_t * data_out, uint8_t * len_out){
    uint32_t checksum = 0;
    uint8_t i, d;
    if(!_can_send()){
        return false;
    }
    _kiss_rx_state = KISS_CMD;
    _kiss_rx_cmd = cmd;
    _uart_tx_byte(cmd);
    if(len_in){
        _uart_tx_byte(KISS_MAGIC);
        _uart_tx_byte(len_in);
        for(i=0;i<len_in;i++){
            d = data_in[i];
            checksum += d;
            _uart_tx_byte(d);
        }
        _uart_tx_byte(checksum/len_in);
    }
    _kiss_rx_state = KISS_START;
    _kiss_rx_started_at = millis();

    while(!_kiss_can_send() && _kiss_rx_state < KISS_DONE){
        delay(1);
    }
    if(_kiss_rx_state == KISS_DONE){
#ifdef KISS_ENABLE_DEBUG_CALLBACK
        if(kiss_dbg_cb){
            kiss_dbg_cb(_kiss_rx_cmd, _kiss_rx_buffer, _kiss_rx_len);
        }
#endif
        if(data_out){
            memcpy(data_out, (uint8_t *)_kiss_rx_buffer, _kiss_rx_len);
        }
        if(len_out){
            *len_out = _kiss_rx_len;
        }
        //maybe check for action response?
        _kiss_rx_state = KISS_IDLE;
        return true;
    }
    _kiss_rx_state = KISS_IDLE;
    return false;
}

/*
 * KISS Public Methods
 * */

bool kiss_test_motors(kiss_test_motors_t * motors){
    bool res = _kiss_send_cmd(KISS_MOTOR_TEST, (uint8_t*)motors, 7, NULL, NULL);
    if(res && (_kiss_rx_len != 1 || _kiss_rx_buffer[0] != 6)){
        return false;
    }
    return res;
}

bool kiss_set_settings(kiss_settings_t * data_in){
    uint8_t * tmp = (uint8_t *)malloc(161);
    if(!tmp || !_can_send()){
        return false;
    }
    memset(tmp, 0, 161);
    kiss_set_settings_t * data_out = (kiss_set_settings_t*)tmp;
    cpUint(pid_p[0]);
    cpUint(pid_p[1]);
    cpUint(pid_p[2]);
    cpUint(pid_i[0]);
    cpUint(pid_i[1]);
    cpUint(pid_i[2]);
    cpUint(pid_d[0]);
    cpUint(pid_d[1]);
    cpUint(pid_d[2]);
    cpUint(angle_p);
    cpUint(angle_i);
    cpUint(angle_d);
    cpInt(acc_trim[0]);
    cpInt(acc_trim[1]);
    cpInt(rc_rate[0]);
    cpInt(rc_rate[1]);
    cpInt(rc_rate[2]);
    cpInt(rc_expo[0]);
    cpInt(rc_expo[1]);
    cpInt(rc_expo[2]);
    cpInt(rc_curve[0]);
    cpInt(rc_curve[1]);
    cpInt(rc_curve[2]);
    cpInt(rx_type);
    cpInt(ppm_chan_order);
    cpInt(copter_type);
    cpInt(active_3d_mode);
    cpInt(oneshot_125);
    cpInt(min_command);
    cpInt(mid_command);
    cpInt(min_throttle);
    cpInt(max_throttle);
    cpInt(tri_mid);
    cpByte(tri_inverted);
    cpInt(acc_zero[0]);
    cpInt(acc_zero[1]);
    cpInt(acc_zero[2]);
    cpByte(aux[0].value);
    cpByte(aux[1].value);
    cpByte(aux[2].value);
    cpByte(aux[3].value);
    cpUint(max_angle);
    cpByte(lpf);
    cpUint(tpa[0]);
    cpUint(tpa[1]);
    cpUint(tpa[2]);
    cpByte(oneshot_42);
    cpByte(failsafe_seconds);
    cpByte(board_rotation);
    cpByte(custom_tpa);
    cpByte(tpa_bp[0]);
    cpByte(tpa_bp[1]);
    cpByte(tpa_bpi[0]);
    cpByte(tpa_bpi[1]);
    cpByte(tpa_bpi[2]);
    cpByte(tpa_bpi[3]);
    cpByte(voltage_influence);
    cpInt(voltage[0]);
    cpInt(voltage[1]);
    cpInt(voltage[2]);
    cpByte(influence[0]);
    cpByte(influence[1]);
    cpByte(influence[2]);
    cpByte(vtx_channel);
    cpByte(logger_config);
    cpByte(rgb.r);
    cpByte(rgb.g);
    cpByte(rgb.b);
    cpUint(vbat_alarm);
    cpInt(cbo[0]);
    cpInt(cbo[1]);
    cpInt(cbo[2]);
    cpByte(aux_4.value);
    cpByte(lap_timer_type);
    cpUint(transponder_id);
    cpByte(logger_debug_vars);
    cpByte(notch_filter[0].enable);
    cpUint(notch_filter[0].center_freq);
    cpUint(notch_filter[0].cutoff_freq);
    cpByte(notch_filter[1].enable);
    cpUint(notch_filter[1].center_freq);
    cpUint(notch_filter[1].cutoff_freq);
    cpByte(yaw_c_filter);
    cpByte(vtx_type);
    cpUint(vtx_power_low);
    cpUint(vtx_power_high);
    cpByte(aux567[0].value);
    cpByte(aux567[1].value);
    cpByte(aux567[2].value);
    cpUint(mah_alarm);
    cpByte(deadband[0]);
    cpByte(deadband[1]);
    cpByte(deadband[2]);
    cpByte(motor_buzzer);
    bool res = _kiss_send_cmd(KISS_SET_SETTINGS, tmp, 161, NULL, NULL);
    free(tmp);
    if(res && (_kiss_rx_len != 1 || _kiss_rx_buffer[0] != 6)){
        return false;
    }
    return res;
}

bool kiss_get_settings(kiss_settings_t * data){
    if(!_kiss_send_cmd(KISS_GET_SETTINGS, NULL, 0, (uint8_t*)data, NULL)){
        return false;
    }
    ttUint(pid_p[0]);
    ttUint(pid_p[1]);
    ttUint(pid_p[2]);
    ttUint(pid_i[0]);
    ttUint(pid_i[1]);
    ttUint(pid_i[2]);
    ttUint(pid_d[0]);
    ttUint(pid_d[1]);
    ttUint(pid_d[2]);
    ttUint(angle_p);
    ttUint(angle_i);
    ttUint(angle_d);
    ttInt(acc_trim[0]);
    ttInt(acc_trim[1]);
    ttInt(rc_rate[0]);
    ttInt(rc_rate[1]);
    ttInt(rc_rate[2]);
    ttInt(rc_expo[0]);
    ttInt(rc_expo[1]);
    ttInt(rc_expo[2]);
    ttInt(rc_curve[0]);
    ttInt(rc_curve[1]);
    ttInt(rc_curve[2]);
    ttInt(rx_type);
    ttInt(ppm_chan_order);
    ttInt(copter_type);
    ttInt(active_3d_mode);
    ttInt(oneshot_125);
    ttInt(min_command);
    ttInt(mid_command);
    ttInt(min_throttle);
    ttInt(max_throttle);
    ttInt(tri_mid);
    ttInt(acc_zero[0]);
    ttInt(acc_zero[1]);
    ttInt(acc_zero[2]);
    ttUint(max_angle);
    ttUint(tpa[0]);
    ttUint(tpa[1]);
    ttUint(tpa[2]);
    ttInt(voltage[0]);
    ttInt(voltage[1]);
    ttInt(voltage[2]);
    ttUint(vbat_alarm);
    ttInt(cbo[0]);
    ttInt(cbo[1]);
    ttInt(cbo[2]);
    ttUint(transponder_id);
    ttUint(notch_filter[0].center_freq);
    ttUint(notch_filter[0].cutoff_freq);
    ttUint(notch_filter[1].center_freq);
    ttUint(notch_filter[1].cutoff_freq);
    ttUint(vtx_power_low);
    ttUint(vtx_power_high);
    ttUint(mah_alarm);
    return true;
}

bool kiss_get_telemetry(kiss_telemetry_t * data){
    if(!_kiss_send_cmd(KISS_GET_TELEMETRY, NULL, 0, (uint8_t*)data, NULL)){
        return false;
    }
    ttInt(throttle);
    ttInt(roll);
    ttInt(pitch);
    ttInt(yaw);
    ttInt(aux[0]);
    ttInt(aux[1]);
    ttInt(aux[2]);
    ttInt(aux[3]);
    ttInt(voltage);
    ttInt(gyro[0]);
    ttInt(gyro[1]);
    ttInt(gyro[2]);
    ttInt(acc[0]);
    ttInt(acc[1]);
    ttInt(acc[2]);
    ttInt(angle[0]);
    ttInt(angle[1]);
    ttInt(angle[2]);
    ttInt(i2c_errors);
    ttInt(gyro_calib_done);
    ttUint(debug0);
    ttInt(gyro_raw[0]);
    ttInt(gyro_raw[1]);
    ttInt(gyro_raw[2]);
    ttInt(acc_raw[0]);
    ttInt(acc_raw[1]);
    ttInt(acc_raw[2]);
    ttInt(acc_trim[0]);
    ttInt(acc_trim[1]);
    ttInt(acc_angle[0]);
    ttInt(acc_angle[1]);
    ttUint(debug1);
    ttInt(pwm_out[0]);
    ttInt(pwm_out[1]);
    ttInt(pwm_out[2]);
    ttInt(pwm_out[3]);
    ttInt(pwm_out[4]);
    ttInt(pwm_out[5]);
    ttUint(debug2);
    ttInt(esc[0].temperature);
    ttInt(esc[0].voltage);
    ttInt(esc[0].current);
    ttInt(esc[0].used_ah);
    ttInt(esc[0].rpm);
    ttInt(esc[1].temperature);
    ttInt(esc[1].voltage);
    ttInt(esc[1].current);
    ttInt(esc[1].used_ah);
    ttInt(esc[1].rpm);
    ttInt(esc[2].temperature);
    ttInt(esc[2].voltage);
    ttInt(esc[2].current);
    ttInt(esc[2].used_ah);
    ttInt(esc[2].rpm);
    ttInt(esc[3].temperature);
    ttInt(esc[3].voltage);
    ttInt(esc[3].current);
    ttInt(esc[3].used_ah);
    ttInt(esc[3].rpm);
    ttInt(esc[4].temperature);
    ttInt(esc[4].voltage);
    ttInt(esc[4].current);
    ttInt(esc[4].used_ah);
    ttInt(esc[4].rpm);
    ttInt(esc[5].temperature);
    ttInt(esc[5].voltage);
    ttInt(esc[5].current);
    ttInt(esc[5].used_ah);
    ttInt(esc[5].rpm);
    ttInt(max_voltage);
    ttInt(max_current);
    ttInt(total_ah);
    ttInt(max_rpm);
    ttInt(max_watt);
    return true;
}

bool kiss_get_info(char * str, kiss_esc_info_t * info){
    _kiss_send_cmd(KISS_ESC_INFO, NULL, 0, NULL, NULL);
    if(!_kiss_send_cmd(KISS_GET_INFO, NULL, 0, NULL, NULL)){
        return false;
    }
    char * name = (char *)_kiss_rx_buffer;
    size_t len = strlen(name);
    if(len >= _kiss_rx_len){
        return false;
    }
    kiss_esc_info_t * i = (kiss_esc_info_t *)(_kiss_rx_buffer+len+1);
    if(i->count > 6){
        return false;
    }
    uint8_t info_len = _kiss_rx_len - len - 1;
    uint8_t full_len = sizeof(kiss_esc_info_t);
    if(info_len > full_len){
        return false;
    }
    memcpy((uint8_t*)info, (uint8_t*)i, full_len);
    memcpy((uint8_t*)str, (uint8_t*)_kiss_rx_buffer, len+1);
    return true;
}

/*
 * MSP Private Methods
 * */

//reset rx buffer and state
static void _msp_reset(){
    _msp_rx_err = false;
    _msp_rx_cmd = 0;
    _msp_rx_len = 0;
    _msp_rx_index = 0;
    _msp_rx_checksum = 0;
    _msp_rx_direction = APP_TO_DEVICE;
    _msp_rx_state = MSP_START;
}

//check if MSP transmission is in progress
static bool _msp_can_send(){
    if(_msp_rx_state > MSP_START && _msp_rx_state < MSP_DONE && (millis() - _msp_rx_started_at) > MSP_RX_TIMEOUT){
        _msp_rx_state = MSP_START;
    }
    return _msp_rx_state == MSP_START;
}

//check if a transmission is complete and call the callback
static void _msp_update(){
    _msp_can_send();
    if(_msp_rx_state == MSP_DONE){
        if(_msp_rx_cb){
            _msp_rx_cb(_msp_rx_direction, _msp_rx_err, _msp_rx_cmd, (uint8_t *)_msp_rx_buf, _msp_rx_len);
        }
        if(_msp_rx_buf){
            free((void *)_msp_rx_buf);
            _msp_rx_buf = NULL;
        }
        _msp_rx_state = MSP_START;
    }
}

//feed UART data through interrupt
static void _msp_rx_byte(uint8_t data){
    _msp_can_send();

    if(_msp_rx_state == MSP_START){
        if(data != '$'){
            //fail
        } else {
            _msp_reset();
            _msp_rx_started_at = millis();
            _msp_rx_state = MSP_M;
        }
    }

    else if(_msp_rx_state == MSP_M){
        if(data != 'M'){
            _msp_rx_state = MSP_START;
        } else {
            _msp_rx_state = MSP_ERR;
        }
    }

    else if(_msp_rx_state == MSP_ERR){
        if(data == '>'){
            _msp_rx_direction = APP_TO_DEVICE;
            _msp_rx_err = false;
            _msp_rx_state = MSP_LEN;
        } else if(data == '<'){
            _msp_rx_direction = DEVICE_TO_APP;
            _msp_rx_err = false;
            _msp_rx_state = MSP_LEN;
        } else if(data == '!'){
            _msp_rx_err = true;
            _msp_rx_state = MSP_LEN;
        } else {
            _msp_rx_state = MSP_START;
        }
    }

    else if(_msp_rx_state == MSP_LEN){
        if(data <= MSP_BUFFER_LEN){
            if(!_msp_rx_buf){
                _msp_rx_buf = (uint8_t *)malloc(MSP_BUFFER_LEN);
                if(!_msp_rx_buf){
                    _msp_rx_state = MSP_START;
                    return;
                }
            }
            memset((void *)_msp_rx_buf, 0, MSP_BUFFER_LEN);
        }
        _msp_rx_checksum = 0;
        _msp_rx_len = data;
        _msp_rx_checksum ^= data;
        _msp_rx_state = MSP_CMD;
    }

    else if(_msp_rx_state == MSP_CMD){
        _msp_rx_cmd = data;
        _msp_rx_checksum ^= data;
        _msp_rx_index = 0;
        _msp_rx_state = MSP_DATA;
    }

    else if(_msp_rx_state == MSP_DATA){
        _msp_rx_buf[_msp_rx_index++] = data;
        _msp_rx_checksum ^= data;
        if(_msp_rx_index == MSP_BUFFER_LEN){
            _msp_rx_state = MSP_START;
        }
        if(_msp_rx_index == _msp_rx_len){
            _msp_rx_state = MSP_CS;
        }
    }

    else if(_msp_rx_state == MSP_CS){
        if(_msp_rx_checksum != data){
            _msp_rx_state = MSP_START;
        } else {
            _msp_rx_state = MSP_DONE;
        }
    }
}

/*
 * MSP Public Methods
 * */

bool msp_send_cmd(msp_direction_t direction, bool err, uint8_t cmd, const uint8_t * data, uint8_t len){
    uint8_t i, checksum = 0;
    if(!_can_send()){
        return false;
    }
    _uart_tx_byte('$');
    _uart_tx_byte('M');
    _uart_tx_byte(err?'!':((direction == DEVICE_TO_APP)?'<':'>'));
    checksum ^= len;
    _uart_tx_byte(len);
    checksum ^= cmd;
    _uart_tx_byte(cmd);
    for(i=0;i<len;i++){
        checksum ^= data[i];
        _uart_tx_byte(data[i]);
    }
    _uart_tx_byte(checksum);
    return true;
}

void msp_on_packet(msp_callback_t * cb){
    _msp_rx_cb = cb;
}

void msp_update(){
    _msp_update();
}

//generic method to ckeck if both protocols are idle
bool _can_send(){
    return _kiss_can_send() && _msp_can_send();
}

/*
 * UART Methods
 * */

void _uart_tx_byte(uint8_t data){
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

ISR(USART_RX_vect){
    unsigned char c = UDR0;
    //check if KISS is expecting data
    if(_kiss_rx_state > KISS_CMD && _kiss_rx_state < KISS_DONE){
        _kiss_rx_byte(c);
    } else {
        _msp_rx_byte(c);
    }
}

void uart_init(uint32_t baud){
    uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
    UCSR0A = 1 << U2X0;

    if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095)){
        UCSR0A = 0;
        baud_setting = (F_CPU / 8 / baud - 1) / 2;
    }

    UBRR0H = baud_setting >> 8;
    UBRR0L = baud_setting;

    //set the data bits, parity, and stop bits
    UCSR0C = 0x06;

    UCSR0B |= (1 << RXEN0);
    UCSR0B |= (1 << TXEN0);
    UCSR0B |= (1 << RXCIE0);
    UCSR0B &= ~(1 << UDRIE0);
}

