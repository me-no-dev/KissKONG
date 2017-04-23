//Uncomment the font that you want to upload
#include "KissKONG_border.h"
//#include "KissKONG_white.h"

#define OSD_SELECT 6

void setup(){
    Serial.begin(115200);
    MAX7456_init();
    uint16_t _p = 0;
    uint16_t i,h;
    uint8_t r,b;
    for(i=0; i<256; i++){
        h = i % 24;
        if(i && !h){
            for(r=0; r<6; r++){
                MAX7456_write(_p++, 0x20);
            }
        }
        MAX7456_write(_p++, i);
    }

    Serial.print("Uploading font");
    uint8_t charBuf[54];
    for(i=0; i<256; i++){
        const uint8_t * cpgm = (const uint8_t *)pgm_read_ptr(chars + i);
        for(b = 0; b < 54; b++){
            charBuf[b] = pgm_read_byte(cpgm + b);
        }
        Serial.write('.');
        MAX7456_writeFontChar(i, charBuf);
    }
    Serial.println("\nDone!");
}

void loop(){

}

#define MAX7456_VM0                         0x00 // Video Mode 0
#define MAX7456_HOS                         0x02 // Horizontal Offset
#define MAX7456_VOS                         0x03 // Vertical Offset
#define MAX7456_DMM                         0x04 // Display Memory Mode
#define MAX7456_DMAH                        0x05 // Display Memory Address High
#define MAX7456_DMAL                        0x06 // Display Memory Address Low
#define MAX7456_DMDI                        0x07 // Display Memory Data In
#define MAX7456_CMM                         0x08 // Character Memory Mode
#define MAX7456_CMAH                        0x09 // Character Memory Address High
#define MAX7456_CMAL                        0x0a // Character Memory Address Low
#define MAX7456_CMDI                        0x0b // Character Memory Data In
#define MAX7456_CMDO                        0xC0 // Character Memory Data Out
#define MAX7456_STAT                        0xA0 // Status

#define MAX7456_NVR_WRITE                   0xA0
#define MAX7456_NVR_READ                    0x50
#define MAX7456_STAT_NVR_BUSY               0x20
#define MAX7456_RESET                       0x02
#define MAX7456_SYNC_NEXT_VSYNC             0x04
#define MAX7456_OSD_ENABLE                  0x08
#define MAX7456_CLEAR_DISPLAY_MEMORY        0x04

#define _MAX7456_write_byte(b)  SPDR = (b); while (!(SPSR & (1<<SPIF))) //_transfer(b)
#define _MAX7456_write(a,d)     _MAX7456_write_byte(a); _MAX7456_write_byte(d)
#define _MAX7456_stat()         _MAX7456_read(MAX7456_STAT)
#define _MAX7456_enable()       _MAX7456_write(MAX7456_VM0, MAX7456_OSD_ENABLE | MAX7456_SYNC_NEXT_VSYNC)
#define _MAX7456_disable()      _MAX7456_write(MAX7456_VM0, 0)

#define _MAX7456_wait_nvr()     while (_MAX7456_stat() & MAX7456_STAT_NVR_BUSY)
#define _MAX7456_select()       digitalWrite(OSD_SELECT, LOW)
#define _MAX7456_deselect()     digitalWrite(OSD_SELECT, HIGH)

static uint8_t _transfer(uint8_t data){
    SPDR = data; while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

static uint8_t _MAX7456_read(uint8_t addr){
    _transfer(addr|0x80);
    return _transfer(0xFF);
}

void MAX7456_init(){
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);
    pinMode(SS, OUTPUT);
    pinMode(OSD_SELECT, OUTPUT);
    digitalWrite(OSD_SELECT, HIGH);
    SPCR = (1<<SPE)|(1<<MSTR);
    SPSR = (1<<SPI2X);
    delayMicroseconds(100);
    _MAX7456_select();
    _MAX7456_write(MAX7456_VM0, MAX7456_RESET);
    _MAX7456_deselect();
    delayMicroseconds(100);
    _MAX7456_select();
    _MAX7456_enable();
    _MAX7456_deselect();
    delayMicroseconds(100);
    _MAX7456_select();
    _MAX7456_enable();
    _MAX7456_deselect();
    delayMicroseconds(100);
    _MAX7456_select();
    _MAX7456_write(MAX7456_HOS, 50);
    _MAX7456_write(MAX7456_VOS, 28);
    _MAX7456_deselect();
    delayMicroseconds(100);
    _MAX7456_select();
    _MAX7456_write(MAX7456_DMM, MAX7456_CLEAR_DISPLAY_MEMORY);
    _MAX7456_deselect();
    delayMicroseconds(100);
    _MAX7456_select();
    _MAX7456_enable();
    _MAX7456_deselect();
}

size_t MAX7456_write(uint16_t index, uint8_t data){
    _MAX7456_select();
    _MAX7456_write(MAX7456_DMAH, index >> 8);
    _MAX7456_write(MAX7456_DMAL, index & 0xFF);
    _MAX7456_write(MAX7456_DMDI, data);
    _MAX7456_deselect();
    return 1;
}

void MAX7456_writeFontChar(uint8_t index, const uint8_t * bitmap){
    uint8_t i;

    _MAX7456_select();
    _MAX7456_disable();

    while(_MAX7456_read(MAX7456_CMAH) != index){
        _MAX7456_write(MAX7456_CMAH, index);
    }

    for(i = 0; i < 54; i++){
        _MAX7456_write(MAX7456_CMAL, i);
        _MAX7456_write(MAX7456_CMDI, bitmap[i]);
    }

    _MAX7456_write(MAX7456_CMM, MAX7456_NVR_WRITE);

    _MAX7456_wait_nvr();
    _MAX7456_enable();
    _MAX7456_deselect();
}
