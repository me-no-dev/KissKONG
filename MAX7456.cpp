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
#include "MAX7456.h"

static void _spi_begin(){
    SPCR = (1<<SPE)|(1<<MSTR);
    SPSR = (1<<SPI2X);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);
}

uint8_t _transfer(uint8_t data){
    SPDR = data; while (!(SPSR & (1<<SPIF)));
    return SPDR;
}

#define MAX7456_VM0                         0x00 // Video Mode 0
#define MAX7456_VM1                         0x01 // Video Mode 1
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
#define MAX7456_OSDM                        0x0c // OSD Insertion Mux
#define MAX7456_RB0                         0x10 // Row 0 Brightness
#define MAX7456_RB1                         0x11 //
#define MAX7456_RB2                         0x12 //
#define MAX7456_RB3                         0x13 //
#define MAX7456_RB4                         0x14 //
#define MAX7456_RB5                         0x15 //
#define MAX7456_RB6                         0x16 //
#define MAX7456_RB7                         0x17 //
#define MAX7456_RB8                         0x18 //
#define MAX7456_RB9                         0x19 //
#define MAX7456_RB10                        0x1a //
#define MAX7456_RB11                        0x1b //
#define MAX7456_RB12                        0x1c //
#define MAX7456_RB13                        0x1d //
#define MAX7456_RB14                        0x1e //
#define MAX7456_RB15                        0x1f // Row 15 Brightness
#define MAX7456_OSDBL                       0x6c // OSD Black Level
#define MAX7456_STAT                        0xA0 // Status
#define MAX7456_DMDO                        0xB0 // Display Memory Data Out
#define MAX7456_CMDO                        0xC0 // Character Memory Data Out

// MAX7456_VM0 bits
#define MAX7456_VBUF_DISABLE                0x01
#define MAX7456_RESET                       0x02
#define MAX7456_SYNC_NEXT_VSYNC             0x04
#define MAX7456_OSD_ENABLE                  0x08
#define MAX7456_SYNC_MODE_AUTO              0x10
#define MAX7456_SYNC_MODE_EXTERNAL          0x20
#define MAX7456_SYNC_MODE_INTERNAL          0x30
#define MAX7456_VIDEO_MODE_PAL              0x40

// MAX7456_DMM bits
#define MAX7456_8BIT_MODE                   0x40
#define MAX7456_LOCAL_BACKGROUND_ENABLE     0x20
#define MAX7456_BLINK                       0x10
#define MAX7456_INVERT                      0x08
#define MAX7456_CLEAR_DISPLAY_MEMORY        0x04
#define MAX7456_CLEAR_ON_VSYNC              0x02
#define MAX7456_AUTO_INCREMENT_MODE         0x01

#define MAX7456_END                         0xFF
#define MAX7456_NVR_WRITE                   0xA0
#define MAX7456_NVR_READ                    0x50

#define MAX7456_STAT_RESET_BUSY             0x40
#define MAX7456_STAT_NVR_BUSY               0x20
#define MAX7456_STAT_VSINC_INACTIVE         0x10
#define MAX7456_STAT_HSINC_INACTIVE         0x08
#define MAX7456_STAT_LOS                    0x04
#define MAX7456_STAT_NTSC_DETECTED          0x02
#define MAX7456_STAT_PAL_DETECTED           0x01

#define _MAX7456_write_byte(b)  SPDR = (b); while (!(SPSR & (1<<SPIF))) //_transfer(b)
#define _MAX7456_write(a,d)     _MAX7456_write_byte(a); _MAX7456_write_byte(d)
#define _MAX7456_stat()         _MAX7456_read(MAX7456_STAT)
#define _MAX7456_enable()       _MAX7456_write(MAX7456_VM0, MAX7456_OSD_ENABLE | _vmode | MAX7456_SYNC_NEXT_VSYNC)
#define _MAX7456_disable()      _MAX7456_write(MAX7456_VM0, _vmode)
#define _MAX7456_wait_nvr()     while (_MAX7456_stat() & MAX7456_STAT_NVR_BUSY)
#define _MAX7456_select()       *csport &= ~cspinmask
#define _MAX7456_deselect()     *csport |= cspinmask

static uint8_t _MAX7456_read(uint8_t addr){
    _transfer(addr|0x80);
    return _transfer(0xFF);
}

MAX7456::MAX7456(uint8_t ss)
    : _ss(ss)
    , _index(0)
    , _vmode(VIDEO_NTSC)
    , _pmode(PRINT_DIRECT)
    , _buffer(NULL)
    , csport(NULL)
    , cspinmask(0)
{}

MAX7456::~MAX7456(){}

print_mode_t MAX7456::getPrintMode(){
    return _pmode;
}

bool MAX7456::setPrintMode(print_mode_t mode){
    if(mode == _pmode){
        return true;
    }
    if(mode != PRINT_BUFFERED && _buffer){
        free(_buffer);
        _buffer = NULL;
    } else if(mode == PRINT_BUFFERED && !_buffer){
        _buffer = (uint8_t *)malloc(480);
        if(!_buffer){
            return false;
        }
    }
    _pmode = mode;
    clearDisplay();
    return true;
}

uint8_t MAX7456::width(){
    return 30;
}

uint8_t MAX7456::height(){
    if(_vmode){
        return 15;
    }
    return 12;
}

void MAX7456::begin(video_mode_t vmode){
    _vmode = vmode;
    csport    = portOutputRegister(digitalPinToPort(_ss));
    cspinmask = digitalPinToBitMask(_ss);
    pinMode(_ss, OUTPUT);
    _MAX7456_deselect();
    _spi_begin();
    reset();
}

void MAX7456::reset(){
    _MAX7456_select();
    _MAX7456_write(MAX7456_VM0, MAX7456_RESET | _vmode);
    _MAX7456_deselect();
    delayMicroseconds(100);
    enable();
    delayMicroseconds(100);
    enable();
}

void MAX7456::enable(){
    _MAX7456_select();
    _MAX7456_enable();
    _MAX7456_deselect();
}

void MAX7456::disable(){
    _MAX7456_select();
    _MAX7456_disable();
    _MAX7456_deselect();
}

void MAX7456::clear(){
    _MAX7456_select();
    _MAX7456_write(MAX7456_DMM, MAX7456_CLEAR_DISPLAY_MEMORY);
    _MAX7456_deselect();
    delayMicroseconds(100);
    enable();
}

void MAX7456::clearLine(uint8_t line){
    size_t i;
    uint16_t index = line * 30;

    if(line >= height()){
        return;
    }

    if(_pmode == PRINT_BUFFERED){
        for(i=0;i<30;i++){
            _buffer[index+i] = ' ';
        }
    } else {
        _MAX7456_select();

        _MAX7456_write(MAX7456_DMM, MAX7456_AUTO_INCREMENT_MODE);
        _MAX7456_write(MAX7456_DMAH, index >> 8);
        _MAX7456_write(MAX7456_DMAL, index & 0xFF);

        for(i=0;i<30;i++){
            _MAX7456_write_byte(MAX7456_DMDI);
            _MAX7456_write_byte(' ');
        }

        _MAX7456_write(MAX7456_DMDI, MAX7456_END);
        _MAX7456_write(MAX7456_DMM, 0);
        _MAX7456_enable();

        _MAX7456_deselect();
    }
}

video_mode_t MAX7456::getVideoMode(){
    uint8_t vmode = 0;
    _MAX7456_select();
    vmode = _MAX7456_read(MAX7456_VM0) & VIDEO_PAL;
    _MAX7456_deselect();
    if(vmode != _vmode){
        _vmode = (video_mode_t)vmode;
    }
    return (video_mode_t)(vmode);
}

void MAX7456::setVideoMode(video_mode_t vmode){
    if(_vmode == vmode || (vmode && vmode != VIDEO_PAL)){
        return;
    }
    _vmode = vmode;
    _MAX7456_select();
    uint8_t temp = _MAX7456_read(MAX7456_VM0) & ~VIDEO_PAL;
    _MAX7456_write(MAX7456_VM0, temp | _vmode);
    _MAX7456_deselect();
}

video_mode_t MAX7456::detectVideoMode(){
    _MAX7456_select();
    byte stat = _MAX7456_stat();
    _MAX7456_deselect();

    if (stat & MAX7456_STAT_PAL_DETECTED){ //PAL
        setVideoMode(VIDEO_PAL);
        return VIDEO_PAL;
    } else if(stat & MAX7456_STAT_NTSC_DETECTED){ //NTSC
        setVideoMode(VIDEO_NTSC);
        return VIDEO_NTSC;
    }
    //If no signal was detected
    return VIDEO_NONE;
}

brightness_level_t MAX7456::getBrightness(){
    uint8_t brightness = 0;
    _MAX7456_select();
    brightness = _MAX7456_read(MAX7456_RB0) >> 5;
    _MAX7456_deselect();
    return (brightness_level_t)(brightness);
}

void MAX7456::setBrightness(brightness_level_t level){
    uint8_t i, maxI = 12;
    if(_vmode){
        maxI = 15;
    }
    _MAX7456_select();
    for(i=0;i<maxI;i++){
        _MAX7456_write(MAX7456_RB0 + i, (level << 5) | 0x10);
    }
    _MAX7456_deselect();
}

int8_t MAX7456::getOffsetLeft(){
    uint8_t offset = 0;
    _MAX7456_select();
    offset = _MAX7456_read(MAX7456_HOS);
    _MAX7456_deselect();
    return offset - 32;
}

void MAX7456::setOffsetLeft(int8_t pixels){
    int8_t offset = pixels + 32;
    if(offset < 0 || offset > 63){
        return;
    }
    _MAX7456_select();
    _MAX7456_write(MAX7456_HOS, offset);
    _MAX7456_deselect();
}

int8_t MAX7456::getOffsetTop(){
    uint8_t offset = 0;
    _MAX7456_select();
    offset = _MAX7456_read(MAX7456_VOS);
    _MAX7456_deselect();
    return offset - 15;
}

void MAX7456::setOffsetTop(int8_t pixels){
    int8_t offset = pixels + 15;
    if(offset < 0 || offset > 31){
        return;
    }
    _MAX7456_select();
    _MAX7456_write(MAX7456_VOS, offset);
    _MAX7456_deselect();
}

//index 0-30, y 0-390/480 NTSC/PAL
size_t MAX7456::write(uint16_t index, const uint8_t * buf, size_t len){
    size_t i;
    size_t maxLen = 480;

    if(index >= maxLen){
        return 0;
    }
    maxLen -= index;
    if(len > maxLen){
        len = maxLen;
    }

    _MAX7456_select();

    _MAX7456_write(MAX7456_DMM, MAX7456_AUTO_INCREMENT_MODE);
    _MAX7456_write(MAX7456_DMAH, index >> 8);
    _MAX7456_write(MAX7456_DMAL, index & 0xFF);

    for(i=0;i<len;i++){
        _MAX7456_write_byte(MAX7456_DMDI);
        _MAX7456_write_byte(buf[i]);
    }

    _MAX7456_write(MAX7456_DMDI, MAX7456_END);
    _MAX7456_write(MAX7456_DMM, 0);
    _MAX7456_enable();

    _MAX7456_deselect();

    return len;
}

size_t MAX7456::write(uint16_t index, uint8_t data){
    size_t maxLen = 480;

    if(index >= maxLen){
        return 0;
    }

    _MAX7456_select();

    _MAX7456_write(MAX7456_DMAH, index >> 8);
    _MAX7456_write(MAX7456_DMAL, index & 0xFF);
    _MAX7456_write(MAX7456_DMDI, data);

    _MAX7456_deselect();

    return 1;
}

/*
 * Print functions
 * */
void MAX7456::clearDisplay(){
    if(_pmode == PRINT_BUFFERED){
        memset(_buffer, 0x20, 480);
    } else {
        clear();
    }
}

void MAX7456::display(){
    if(_pmode == PRINT_BUFFERED){
        write(0,0,_buffer,480);
    }
}

size_t MAX7456::write(uint8_t x, uint8_t y, uint8_t data){
    return write((y * 30) + x, data);
}

size_t MAX7456::write(uint8_t x, uint8_t y, const uint8_t * buf, size_t len){
    return write((y * 30) + x, buf, len);
}

void MAX7456::setCursor(uint8_t x, uint8_t y){
    uint16_t index = (y * 30) + x;
    if(index < (uint16_t)(width() * height())){
        _index = index;
    }
}

size_t MAX7456::write(uint8_t data){
    //start of current line
    if(data == '\r'){
        _index = (_index / 30) * 30;
        return 1;
    }

    //new line
    if(data == '\n'){
        _index = (((_index / 30) + 1) % height()) * 30;
        return 1;
    }
    if(!_index){
        clearDisplay();
    }
    if(_pmode == PRINT_BUFFERED){
        _buffer[_index] = data;
    } else {
        _MAX7456_select();
        _MAX7456_write(MAX7456_DMAH, _index >> 8);
        _MAX7456_write(MAX7456_DMAL, _index & 0xFF);
        _MAX7456_write(MAX7456_DMDI, data);
        _MAX7456_deselect();
    }

    if(_vmode){
        _index = (_index + 1) % 450;
    } else {
        _index = (_index + 1) % 360;
    }

    return 1;
}

size_t MAX7456::write(const uint8_t * buf, size_t len){
    size_t i, maxLen = 360;
    if(_vmode){
        maxLen = 450;
    }
    if((_index + len) > maxLen){//more than what is left from the screen.
        len = maxLen - _index;
    }
    for(i=0;i<len;i++){
        write(buf[i]);
    }
    return len;
}

/*
 * Font Functions
 * */

void MAX7456::writeFontChar(uint8_t index, const uint8_t * bitmap){
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

void MAX7456::readFontChar(uint8_t index, uint8_t * bitmap){
    uint8_t i;


    _MAX7456_select();
    _MAX7456_disable();

    while(_MAX7456_read(MAX7456_CMAH) != index){
        _MAX7456_write(MAX7456_CMAH, index);
    }

    _MAX7456_write(MAX7456_CMM, MAX7456_NVR_READ);

    for(i = 0; i < 54; i++){
        _MAX7456_write(MAX7456_CMAL, i);
        bitmap[i] = _MAX7456_read(MAX7456_CMDO);
    }

    _MAX7456_wait_nvr();
    _MAX7456_enable();
    _MAX7456_deselect();
}

void MAX7456::replaceFontChar(uint8_t index, const uint8_t * bitmap_in, uint8_t * bitmap_out){
    uint8_t i, temp;

    _MAX7456_select();
    _MAX7456_disable();

    while(_MAX7456_read(MAX7456_CMAH) != index){
        _MAX7456_write(MAX7456_CMAH, index);
    }

    _MAX7456_write(MAX7456_CMM, MAX7456_NVR_READ);

    for(i = 0; i < 54; i++){
        _MAX7456_write(MAX7456_CMAL, i);
        temp = _MAX7456_read(MAX7456_CMDO);
        _MAX7456_write(MAX7456_CMDI, bitmap_in[i]);
        bitmap_out[i] = temp;
    }

    _MAX7456_write(MAX7456_CMM, MAX7456_NVR_WRITE);

    _MAX7456_wait_nvr();
    _MAX7456_enable();
    _MAX7456_deselect();
}

