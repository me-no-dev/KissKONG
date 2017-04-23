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

/*
 * Print Methods for various types of data
 * */

//0 -> 999 (4)
void printPid(int16_t value){
    if(value < 100){
        osd.write(SPACE);
    }
    osd.print(value/10.0, 1);
}

//0 -> 999 (4)
void printRate(int16_t value){
    osd.print(value/100.0, 2);
}

//0 -> 99999 (7)
void printCurent(int32_t value, bool printName){
    if(value < 10000){
        osd.write(SPACE);
        if(value < 1000){
            osd.write(SPACE);
        }
    }
    osd.print(value/100.0, 2);
    if(printName){
        osd.write('A');
    }
}

//0 -> 9999 (6)
void printVoltage(int32_t value, bool printName){
    if(value < 1000){
        osd.write(SPACE);
    }
    osd.print(value/100.0, 2);
    if(printName){
        osd.write('V');
    }
}

//0 -> 99999 (8) poles is usually 14
void printRpm(int32_t value, uint8_t poles, bool printName){
    value = (value*32)/poles;
    if(value < 10000){
        osd.write(SPACE);
        if(value < 1000){
            osd.write(SPACE);
            if(value < 100){
                osd.write(SPACE);
                if(value < 10){
                    osd.write(SPACE);
                }
            }
        }
    }
    osd.print(value);
    if(printName){
        osd.print("rpm");
    }
}

//0 -> 9999 (7)
void printMah(int32_t value, bool printName){
    if(value < 1000){
        osd.write(SPACE);
        if(value < 100){
            osd.write(SPACE);
            if(value < 10){
                osd.write(SPACE);
            }
        }
    }
    osd.print(value);
    if(printName){
        osd.print("mAh");
    }
}

//0 -> 100 (4)
void printPercentage(uint16_t data, bool printName){
    if(data < 100){
        osd.write(SPACE);
        if(data < 10){
            osd.write(SPACE);
        }
    }
    osd.print(data);
    if(printName){
        osd.write('%');
    }
}

//0 -> 100 (4)
void printMaxAngle(uint16_t data){
    if(data < 100){
        osd.write(SPACE);
        if(data < 10){
            osd.write(SPACE);
        }
    }
    osd.print(data);
    osd.write(0x87);
}

//-99 -> +999 (4)
void printTemperature(int16_t value, bool printName){
    if(value < 100 && value >= 0){
        osd.write(SPACE);
    }
    if(value < 10 && value > -10){
        osd.write(SPACE);
    }
    osd.print(value);
    if(printName){
        osd.write(0x86);
    }
}

//-18000 -> 18000 (7)
void printAngle(int16_t value, bool printName){
    if(value >= 0){
        osd.write(SPACE);
    }
    if(value > -10000 && value < 10000){
        osd.write(SPACE);
    }
    if(value > -1000 && value < 1000){
        osd.write(SPACE);
    }
    osd.print(value/100.0, 1);
    if(printName){
        osd.write(0x87);
    }
}

//takes ms (5)
void printDuration(uint32_t duration){
    duration /= 1000;
    uint8_t seconds = duration % 60;
    uint32_t minutes = (duration - seconds) / 60;
    if(minutes < 10){
        osd.write(SPACE);
    }
    osd.print(minutes);
    osd.write(':');
    if(seconds < 10){
        osd.write('0');
    }
    osd.print(seconds);
}

// (5)
void printFlightMode(){
    uint8_t m = telemetry.mode;
    if(!m){
        osd.print(F(" ACRO"));
    } else if(m == 1){
        osd.print(F("LEVEL"));
    } else if(m == 2){
        osd.print(F("   3D"));
    } else {
        osd.print(F("     "));
    }
}

// (8)
void printEscType(uint8_t type){
    if(type == 1){
        osd.print(F("KISS 8A "));
    } else if(type == 2){
        osd.print(F("KISS 16A"));
    } else if(type == 3){
        osd.print(F("KISS 24A"));
    } else {
        osd.print(F("UNKNOWN"));
        osd.print(type);
    }
}

// (3)
void printAuxConfig(kiss_aux_config_type_t type){
    kiss_aux_config_t * conf = NULL;
    if(type > AUX_LED){
        conf = &settings.aux_4;
    } else {
        conf = &settings.aux[type];
    }

    if(conf->level){
        osd.write(0x30+conf->channel);
        switch(conf->level){
            case AUX_LOW:       osd.print(F("LO")); break;
            case AUX_LOW_MED:   osd.print(F("ML")); break;
            case AUX_MED:       osd.print(F("ME")); break;
            case AUX_MED_HIGH:  osd.print(F("MH")); break;
            case AUX_HIGH:      osd.print(F("HI")); break;
            default: break;
        }
    } else {
        osd.print(F("OFF"));
    }
}

// (5)
void printOutputMode(){
    switch(settings.oneshot_125){
        case 0:  osd.print(F("  PWM")); break;
        case 1:  osd.print(F("OS125")); break;
        case 2:  osd.print(F(" OS42")); break;
        case 3:  osd.print(F("DS150")); break;
        case 4:  osd.print(F("DS300")); break;
        case 5:  osd.print(F("DS600")); break;
        default: osd.print(F("ERROR")); break;
    }
}

// (4)
void printRxType(){
    switch(settings.rx_type){
        case 0:  osd.print(F("AUTO")); break;
        case 1:  osd.print(F("46SC")); break;
        case 2:  osd.print(F("TRPY")); break;
        case 3:  osd.print(F("PTRY")); break;
        case 4:  osd.print(F("TPYR")); break;
        case 5:  osd.print(F("RPYT")); break;
        case 6:  osd.print(F("DSM2")); break;
        case 7:  osd.print(F("DMX2")); break;
        case 8:  osd.print(F("SBUS")); break;
        case 9:  osd.print(F("SUMD")); break;
        case 10: osd.print(F("XBUS")); break;
        case 11: osd.print(F("FBSB")); break;
        case 12: osd.print(F("PRTY")); break;
        case 13: osd.print(F("RPTY")); break;
        case 14: osd.print(F("SRXL")); break;
        case 15: osd.print(F("SBNI")); break;
        case 16: osd.print(F("JRXB")); break;
        default: osd.print(F("OTHR")); break;
    }
}

// (5)
void printLpfFreq(){
    switch(settings.lpf){
        case 0:  osd.print(F("  OFF")); break;
        case 1:  osd.print(F("   HI")); break;
        case 2:  osd.print(F("MEDHI")); break;
        case 3:  osd.print(F("  MED")); break;
        case 4:  osd.print(F("MEDLO")); break;
        case 5:  osd.print(F("   LO")); break;
        case 6:  osd.print(F("VRYLO")); break;
        default: osd.print(F("ERROR")); break;
    }
}

// (5)
void printFrameType(){
    switch(settings.copter_type){
        case 0:  osd.print(F("  TRI")); break;
        case 1:  osd.print(F("Quad+")); break;
        case 2:  osd.print(F("QuadX")); break;
        case 3:  osd.print(F("   Y4")); break;
        case 4:  osd.print(F("   Y6")); break;
        case 5:  osd.print(F("Hexa+")); break;
        case 6:  osd.print(F("HexaX")); break;
        default: osd.print(F("ERROR")); break;
    }
}

// (6)
void printLedColor(){
    uint8_t index = getColorIndex();
    switch(index){
        case LED_BLACK:  osd.print(F(" BLACK")); break;
        case LED_WHITE:  osd.print(F(" WHITE")); break;
        case LED_YELLOW: osd.print(F("YELLOW")); break;
        case LED_ORANGE: osd.print(F("ORANGE")); break;
        case LED_RED:    osd.print(F("   RED")); break;
        case LED_PURPLE: osd.print(F("PURPLE")); break;
        case LED_BLUE:   osd.print(F("  BLUE")); break;
        case LED_GREEN:  osd.print(F(" GREEN")); break;
        case LED_CYAN:   osd.print(F("  CYAN")); break;
        default:         osd.print(F("CUSTOM")); break;
    }
}

// (8)
void printVtxBand(uint8_t band){
    switch(band){
        case 0:  osd.print(F("BOSCAM A")); break;
        case 1:  osd.print(F("BOSCAM B")); break;
        case 2:  osd.print(F("BOSCAM E")); break;
        case 3:  osd.print(F("FatShark")); break;
        case 4:  osd.print(F("RaceBand")); break;
        case 5:  osd.print(F( "D / 5.3")); break;
        default: osd.print(F(" UNKNOWN")); break;
    }
}
