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
#ifndef MAX7456_H_
#define MAX7456_H_

#include "Arduino.h"

typedef enum {
    VIDEO_NTSC = 0x00,
    VIDEO_PAL = 0x40,
    VIDEO_NONE = 0x80
} video_mode_t;

typedef enum {
    BRIGHTNESS_120,
    BRIGHTNESS_100,
    BRIGHTNESS_90,
    BRIGHTNESS_80
} brightness_level_t;

typedef enum {
    PRINT_DIRECT, PRINT_BUFFERED
} print_mode_t;

class MAX7456: public Print {
    private:
        uint8_t _ss;
        uint16_t _index;
        video_mode_t _vmode;
        print_mode_t _pmode;
        uint8_t * _buffer;
        volatile uint8_t *csport;
        uint8_t cspinmask;

    public:
        MAX7456(uint8_t ss=10);
        virtual ~MAX7456();

        void begin(video_mode_t vmode=VIDEO_NTSC);
        void reset();
        void clear();
        void enable();
        void disable();

        uint8_t width(); //30
        uint8_t height();//13/16 NTSC/PAL

        print_mode_t getPrintMode();
        bool setPrintMode(print_mode_t mode);

        video_mode_t detectVideoMode();
        video_mode_t getVideoMode();
        void setVideoMode(video_mode_t vmode);

        brightness_level_t getBrightness();
        void setBrightness(brightness_level_t level);

        //-32 > +31 pixels horizontal offset
        int8_t getOffsetLeft();
        void setOffsetLeft(int8_t pixels);

        //-15 > +16 pixels vertical offset
        int8_t getOffsetTop();
        void setOffsetTop(int8_t pixels);

        //index 0-30, y 0-390/480 NTSC/PAL
        size_t write(uint16_t index, uint8_t data);
        size_t write(uint16_t index, const uint8_t * buf, size_t len);

        //x 0-30, y 0-13/16 NTSC/PAL
        size_t write(uint8_t x, uint8_t y, uint8_t data);
        size_t write(uint8_t x, uint8_t y, const uint8_t * buf, size_t len);

        //index 0-255, bitmap is 54 bytes array
        void writeFontChar(uint8_t index, const uint8_t * bitmap);
        void readFontChar(uint8_t index, uint8_t * bitmap);
        void replaceFontChar(uint8_t index, const uint8_t * bitmap_in, uint8_t * bitmap_out);

        void clearDisplay();
        void display();
        void setCursor(uint8_t x, uint8_t y);
        size_t write(uint8_t data);
        size_t write(const uint8_t * buf, size_t len);

};




#endif /* MAX7456_H_ */
