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
#include "Tramp.h"

#define SUART_PCICR_MASK        (1 << digitalPinToPCICRbit(TRAMP_PIN))
#define SUART_PCMSK_REG         (*digitalPinToPCMSK(TRAMP_PIN))
#define SUART_PCMSK_MASK        (1 << digitalPinToPCMSKbit(TRAMP_PIN))

#define SUART_DDR               *portModeRegister(digitalPinToPort(TRAMP_PIN))
#define SUART_PORT              *portOutputRegister(digitalPinToPort(TRAMP_PIN))
#define SUART_PIN               *portInputRegister(digitalPinToPort(TRAMP_PIN))
#define SUART_MASK              digitalPinToBitMask(TRAMP_PIN)

#define SUART_SET_OUTPUT        SUART_DDR |= SUART_MASK
#define SUART_SET_INPUT         SUART_DDR &= ~SUART_MASK
#define SUART_SET_HIGH          SUART_PORT |= SUART_MASK
#define SUART_SET_LOW           SUART_PORT &= ~SUART_MASK
#define SUART_TOGGLE            SUART_PORT ^= SUART_MASK
#define SUART_GET_STATE         (SUART_PIN & SUART_MASK)

#define SUART_BIT_PERIOD        220
#define SUART_TIMER_PERIOD      208
#define SUART_TIMER_PREDELAY    60

#define sUartStartTimer() {             \
    TCCR1A = 0;                         \
    TCCR1B = (_BV(WGM12) | _BV(CS11));  \
    TCCR1C = 0;                         \
    TIMSK1 = _BV(OCIE1A);               \
    TIFR1  = _BV(OCF1A);                \
    OCR1A = SUART_TIMER_PERIOD;         \
    OCR1B = 1;                          \
}

#define sUartStopTimer() {              \
    TCCR1A = 0;                         \
    TCCR1B = 0;                         \
    TCCR1C = 0;                         \
    TIMSK1 = 0;                         \
    TIFR1  = 0;                         \
}

#define sUartStartInterrupt() {         \
    PCICR = SUART_PCICR_MASK;           \
    SUART_PCMSK_REG = SUART_PCMSK_MASK; \
}

#define sUartStopInterrupt() {          \
    PCICR = 0;                          \
    SUART_PCMSK_REG = 0;                \
}

volatile uint8_t sUart_rx_value; //the byte that is currently sending
volatile uint8_t sUart_rx_bit_index; //0-10 Index of the bit from the byte that is currently sending

static bool sUartOnData(uint8_t data);

inline void sUartDelay(uint16_t value) {
    uint8_t tmp=0;
    asm volatile(
        "sbiw    %0, 0x01 \n\t"
        "ldi %1, 0xFF \n\t"
        "cpi %A0, 0xFF \n\t"
        "cpc %B0, %1 \n\t"
        "brne .-10 \n\t"
        : "+r" (value), "+a" (tmp)
        : "0" (value)
    );
}

static inline void sUartPinInterrupt(){
    if(SUART_GET_STATE) {
        return;
    }
    //Start Bit
    cli();
    sUartStopInterrupt();
    sUartDelay(SUART_TIMER_PREDELAY);
    sUartStartTimer();
    sei();
}

ISR(TIMER1_COMPA_vect) {
    cli();
    if(sUart_rx_bit_index < 8){//Data Bits
        if(SUART_GET_STATE){
            sUart_rx_value |= (1 << sUart_rx_bit_index);
        }
        sUart_rx_bit_index++;
    } else if(sUart_rx_bit_index == 8){//Stop Bit
        sUartStopTimer();
        if(sUartOnData(sUart_rx_value)){
            sUartStartInterrupt();
        }
        sUart_rx_value = 0;
        sUart_rx_bit_index = 0;
    }
    sei();
}

ISR(PCINT0_vect){
    sUartPinInterrupt();
}
ISR(PCINT1_vect){
    sUartPinInterrupt();
}
ISR(PCINT2_vect){
    sUartPinInterrupt();
}

uint8_t sUartWriteByte(uint8_t data){
    SUART_SET_LOW;
    sUartDelay(SUART_BIT_PERIOD);//start bit
    for(uint8_t i = 0; i < 8; i++){
        if((data & (1 << i)) > 0){
            SUART_SET_HIGH;
        } else {
            SUART_SET_LOW;
        }
        sUartDelay(SUART_BIT_PERIOD - 1);//data bit
    }
    SUART_SET_HIGH;
    sUartDelay(SUART_BIT_PERIOD);//stop bit
    return 1;
}

uint8_t sUartWriteBytes(uint8_t *data, uint8_t len){
    uint8_t i;
    for(i = 0; i < len; i++){
        sUartWriteByte(data[i]);
    }
    return len;
}

void sUartInitTX(){
    cli();
    sUartStopTimer();
    sUartStopInterrupt();
    SUART_SET_OUTPUT;
    SUART_SET_HIGH;
    sei();
}

void sUartInitRX(){
    cli();
    sUartStopTimer();
    SUART_SET_INPUT;
    sUart_rx_bit_index = 0;
    sUart_rx_value = 0;
    sUartStartInterrupt();
    sei();
}


#define TRAMP_PACKET_LENGTH 16

#define TRAMP_CMD_RF        0x72
#define TRAMP_CMD_STATUS    0x76
#define TRAMP_CMD_SENSOR    0x73
#define TRAMP_CMD_PIT_MODE  0x49
#define TRAMP_CMD_SET_POWER 0x50
#define TRAMP_CMD_SET_FREQ  0x46

typedef struct {
        uint8_t len; //15
        uint8_t cmd;
        union {
                struct {// TRAMP_CMD_RF response
                        uint16_t min_freq;
                        uint16_t max_freq;
                        uint16_t max_power;
                        uint8_t reserved[6];
                } rf;
                struct {// TRAMP_CMD_STATUS response
                       uint16_t freq;
                       uint16_t configured_power;
                       uint8_t reserved0;
                       uint8_t pit_mode;
                       uint16_t actual_power;
                       uint8_t reserved[4];
                } status;
                struct {// TRAMP_CMD_SENSOR response
                       uint8_t reserved0[4];
                       uint16_t temp;
                       uint8_t reserved1[6];
                } sensor;
                uint8_t bytes[12];
        } data;
        uint8_t crc; //sum of cmd and data bytes
        uint8_t end; //0x00
} tramp_packet_t;

typedef enum { TRAMP_IDLE, TRAMP_TX, TRAMP_RX } tramp_state_t;
typedef enum { TRAMP_DATA, TRAMP_DONE, TRAMP_ERROR } tramp_packet_state_t;

volatile tramp_data_t tramp_data;
volatile tramp_state_t tramp_state;
volatile tramp_packet_state_t tramp_rx_state;
volatile uint8_t tramp_rx_buf[TRAMP_PACKET_LENGTH]; //Queue that stores the bytes to be sent
volatile uint8_t tramp_rx_len; //Current bytes waiting
volatile uint8_t tramp_rx_crc;
volatile bool tramp_data_updated = false;
volatile long tramp_tx_at = 0;

static bool sUartOnData(uint8_t data){
    if(tramp_rx_state || tramp_rx_len >= TRAMP_PACKET_LENGTH){
        return false;
    }
    if(!tramp_rx_len){
        if(data != 15){
            //len error
            tramp_rx_state = TRAMP_ERROR;
        }
        tramp_rx_crc = 0;
    } else if(tramp_rx_len < 14){
        tramp_rx_crc += data;
    } else if(tramp_rx_len == 14 && tramp_rx_crc != data){
        //crc error!
        tramp_rx_state = TRAMP_ERROR;
    } else if(tramp_rx_len == 15){
        if(data){
            //end error
            tramp_rx_state = TRAMP_ERROR;
        } else {
            tramp_rx_state = TRAMP_DONE;
            tramp_packet_t * p = (tramp_packet_t*)tramp_rx_buf;
            if(p->cmd == TRAMP_CMD_RF){
                tramp_data.min_freq = p->data.rf.min_freq;
                tramp_data.max_freq = p->data.rf.max_freq;
                tramp_data.max_power = p->data.rf.max_power;
            } else if(p->cmd == TRAMP_CMD_STATUS){
                tramp_data.freq = p->data.status.freq;
                tramp_data.configured_power = p->data.status.configured_power;
                tramp_data.actual_power = p->data.status.actual_power;
                tramp_data.pit_mode = p->data.status.pit_mode;
                if(!tramp_data.new_freq){
                    tramp_data.new_freq = p->data.status.freq;
                    tramp_data.new_power = p->data.status.configured_power;
                    tramp_data.new_pit = p->data.status.pit_mode;
                }
            } else if(p->cmd == TRAMP_CMD_SENSOR){
                tramp_data.temp = p->data.sensor.temp;
            }
            tramp_data_updated = true;
        }
        tramp_tx_at = millis();
        sUartInitTX();
        tramp_state = TRAMP_IDLE;
    }
    tramp_rx_buf[tramp_rx_len++] = data;
    return !tramp_rx_state;
}

static inline void trampSendPacket(uint8_t cmd, uint8_t * data, uint8_t len){
    tramp_packet_t packet;
    uint8_t i;

    if(tramp_state != TRAMP_IDLE){
        return;
    }

    tramp_state = TRAMP_TX;

    memset((void*)&packet, 0, sizeof(tramp_packet_t));
    packet.len = 15;
    packet.cmd = cmd;
    packet.crc += cmd;
    for(i=0; i<len; i++){
        packet.data.bytes[i] = data[i];
        packet.crc += data[i];
    }
    cli();
    sUartWriteBytes((uint8_t*)&packet, sizeof(tramp_packet_t));
    sei();
    tramp_tx_at = millis();
    if(cmd == TRAMP_CMD_RF || cmd == TRAMP_CMD_STATUS || cmd == TRAMP_CMD_SENSOR){
        tramp_rx_len = 0;
        tramp_rx_state = TRAMP_DATA;
        tramp_state = TRAMP_RX;
        sUartInitRX();
    } else {
        tramp_state = TRAMP_IDLE;
    }
}

static inline uint8_t trampGetState(){
    if(tramp_state == TRAMP_RX && (millis() - tramp_tx_at) > 100){
        tramp_tx_at = millis();
        sUartInitTX();
        tramp_state = TRAMP_IDLE;
    }
    return tramp_state;
}

void trampSendFrequency(uint16_t freq){
    if(!tramp_data.min_freq || !tramp_data.max_freq){
        return;//Update RF first
    }
    if(freq > tramp_data.max_freq){
        freq = tramp_data.max_freq;
    } else if(freq < tramp_data.min_freq){
        freq = tramp_data.min_freq;
    }
    tramp_data.new_freq = freq;
    return trampSendPacket(TRAMP_CMD_SET_FREQ, (uint8_t*)&freq, sizeof(uint16_t));
}

void trampSendPower(uint16_t milliWatts){
    if(!tramp_data.max_power){
        return;//Update RF first
    }
    if(milliWatts > tramp_data.max_power){
        milliWatts = tramp_data.max_power;
    }
    tramp_data.new_power = milliWatts;
    return trampSendPacket(TRAMP_CMD_SET_POWER, (uint8_t*)&milliWatts, sizeof(uint16_t));
}

void trampSendPitMode(bool enabled){
    uint8_t value = !enabled;
    return trampSendPacket(TRAMP_CMD_PIT_MODE, (uint8_t*)&value, 1);
}

bool trampUpdate(){
    static bool turn = false;
    bool res = false;
    if(trampGetState() != TRAMP_IDLE){
        return res;
    }
    if(tramp_data_updated){
        tramp_data_updated = false;
        res = true;
    }
    if((millis() - tramp_tx_at) > 200){
        if(!tramp_data.min_freq || !tramp_data.max_freq || !tramp_data.max_power){
            tramp_data_updated = false;
            trampSendPacket(TRAMP_CMD_RF, NULL, 0);
        } else if(!tramp_data.freq){
            trampSendPacket(TRAMP_CMD_STATUS, NULL, 0);
        } else if(!tramp_data.temp){
            trampSendPacket(TRAMP_CMD_SENSOR, NULL, 0);
        } else {
            //all data initialized
            if(tramp_data.new_pit != tramp_data.pit_mode){
                tramp_data.pit_mode = tramp_data.new_pit;
                trampSendPitMode(tramp_data.new_pit);
                turn = false;
            } else if(tramp_data.new_freq != tramp_data.freq){
                tramp_data.freq = tramp_data.new_freq;
                trampSendFrequency(tramp_data.new_freq);
                turn = false;
            } else if(tramp_data.new_power != tramp_data.configured_power){
                tramp_data.configured_power = tramp_data.new_power;
                trampSendPower(tramp_data.new_power);
                turn = false;
            } else {
                trampSendPacket(turn?TRAMP_CMD_SENSOR:TRAMP_CMD_STATUS, NULL, 0);
                turn = !turn;
            }
        }
    }
    return res;
}

void trampInit(){
    tramp_data_updated = false;
    memset((void*)&tramp_data, 0, sizeof(tramp_data_t));
    sUartInitTX();
    tramp_state = TRAMP_IDLE;
}


volatile tramp_data_t * trampGetData(){
    if(!tramp_data.temp){
        return NULL;
    }
    return &tramp_data;
}
