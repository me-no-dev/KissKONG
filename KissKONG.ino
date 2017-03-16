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
#include "MAX7456.h"

MAX7456 osd(10);

/*
 * MSP implementation for Font Upload
 * */

void onMSP(msp_direction_t direction, bool err, uint8_t cmd, uint8_t * data, uint8_t len){
    if(cmd == MSP_OSD){
        if(data[0] == MSP_OSD_FONT && direction == APP_TO_DEVICE && !err){
            if(len == 56 || len == 64){
                osd.writeFontChar(data[55], &data[1]);
            }
            return;
        }
    }
}

void msp_delay(uint32_t ms){
    uint32_t _now = millis();
    uint32_t _end = _now + ms;
    if(_now > _end){
        while(millis() > _end){
            msp_update();
        }
    }
    while(millis() < _end){
        msp_update();
    }
}

// Table drawing chars (in the OSD font)
#define TB_HL   "\x01"
#define TB_VL   "\x02"
#define TB_TL   "\x03"
#define TB_TR   "\x04"
#define TB_BL   "\x05"
#define TB_BR   "\x06"
#define TB_LM   "\x07"
#define TB_RM   "\x08"
#define TB_TM   "\x09"
#define TB_BM   "\x0b"
#define TB_XC   "\x0c"

#define TB_AU   "\x0e"
#define TB_AD   "\x0f"

#define PID_TABLE_TOP " " TB_TL TB_HL TB_HL TB_HL TB_HL TB_HL TB_TM TB_HL TB_HL TB_HL TB_HL TB_HL TB_TM TB_HL TB_HL TB_HL TB_HL TB_HL TB_TM TB_HL TB_HL TB_HL TB_HL TB_HL TB_TR
#define PID_TABLE_MID " " TB_LM TB_HL TB_HL TB_HL TB_HL TB_HL TB_XC TB_HL TB_HL TB_HL TB_HL TB_HL TB_XC TB_HL TB_HL TB_HL TB_HL TB_HL TB_XC TB_HL TB_HL TB_HL TB_HL TB_HL TB_RM
#define PID_TABLE_BOT " " TB_BL TB_HL TB_HL TB_HL TB_HL TB_HL TB_BM TB_HL TB_HL TB_HL TB_HL TB_HL TB_BM TB_HL TB_HL TB_HL TB_HL TB_HL TB_BM TB_HL TB_HL TB_HL TB_HL TB_HL TB_BR
#define PID_TABLE_ROW " " TB_VL "     " TB_VL "     " TB_VL "     " TB_VL "     " TB_VL

// Available positions in PIDs and Rates table
#define TABLE_NUM_INDEXES 11

// Sicks Axis for actions
#define AXIS_LR telemetry.roll  //Left+Right
#define AXIS_UD telemetry.pitch //Up+Down
#define AXIS_YN telemetry.yaw   //Yes(Add)+No(Substract)

typedef void (view_cb_t)(void);

// View Methods
typedef struct {
        view_cb_t * init;
        view_cb_t * update;
        view_cb_t * deinit;
} view_methods_t;

// Combined ESC Statistics
typedef struct {
        uint8_t count;
        int32_t temperature;
        int32_t voltage;// /100
        int32_t current;// /100
        int32_t used_ah;// /1000
        int32_t rpm; // (v*32)/MAGNETPOLECOUNT
} kiss_esc_stats_t;

// Flight Statistics
typedef struct {
        uint32_t duration;//ms duration
        uint8_t min_signal;
        int16_t min_voltage;
        int16_t max_current;
        int16_t max_rpm;
        int16_t max_temperature;
} flight_stats_t;

// Position struct to hold coordinates
typedef struct {
        uint8_t x;
        uint8_t y;
} osd_pos_t;

// Views
typedef enum {
    VIEW_INFO, VIEW_PIDS, VIEW_RATES, VIEW_LAST_FLIGHT
    , VIEW_OFF, VIEW_MAIN, VIEW_STATS, VIEW_BAD
} view_index_t;

// Stick Combinations
typedef enum {
    STICKS_NONE, STICKS_LEFT, STICKS_RIGHT, STICKS_UP, STICKS_DOWN, STICKS_YES, STICKS_NO, STICKS_MENU, STICKS_ERR
} sticks_value_t;

// PIDs and Rates table value positions
static uint8_t table_current_index = 0;
const osd_pos_t table_positions[TABLE_NUM_INDEXES] = {
    {8,4}, {14,4}, {20,4},
    {8,6}, {14,6}, {20,6},
    {8,8}, {14,8}, {20,8},
        {4,10}, {17,10}
};

static view_index_t currentView = VIEW_OFF;
static view_index_t lastView = VIEW_OFF;
static uint32_t lastViewSwitch = 0;

static uint8_t mainMenuIndex = 0;

static sticks_value_t currentSticks = STICKS_ERR;

static flight_stats_t flight_stats = {0,0,0,0,0,0};
static kiss_esc_stats_t esc_stats = {0,0,0,0,0,0};
static kiss_telemetry_t telemetry;
static kiss_settings_t settings;
static kiss_esc_info_t info;
static char name[17] = {0};
static bool update_settings = true;
static bool update_info = true;

// Update combined live ESP stats
void update_esc_stats(){
    esc_stats.count = 0;
    esc_stats.temperature = 0;
    esc_stats.voltage = 0;
    esc_stats.current = 0;
    esc_stats.used_ah = 0;
    esc_stats.rpm = 0;
    uint8_t i;
    for(i=0; i<6; i++){
        if(telemetry.esc[i].voltage < 6){
            continue;
        }
        esc_stats.count++;
        esc_stats.temperature += telemetry.esc[i].temperature;
        esc_stats.voltage += telemetry.esc[i].voltage;
        esc_stats.current += telemetry.esc[i].current;
        esc_stats.used_ah += telemetry.esc[i].used_ah;
        esc_stats.rpm += telemetry.esc[i].rpm;
    }
    if(esc_stats.count){
        esc_stats.temperature /= esc_stats.count;
        esc_stats.voltage /= esc_stats.count;
        if(telemetry.armed){
            esc_stats.rpm /= esc_stats.count;
        } else {
            esc_stats.rpm = 0;//fix RPMs being reported when disarmed
        }
    }
}

//Save current settings
bool kiss_save(){
    if(!kiss_set_settings(&settings)) {
        return false;
    }
    return true;
}

//Update telemetry, info and configuration
bool kiss_update(){
    if(!kiss_get_telemetry(&telemetry)){
        return false;
    }
    update_esc_stats();
    if(telemetry.armed){
        return true;
    }
    if(update_info){
        if(!kiss_get_info(name, &info)){
            return false;
        }
        update_info = false;
    }
    if(update_settings){
        if(!kiss_get_settings(&settings)) {
            return false;
        }
        update_settings = false;
    }
    return true;
}

/*
 * Print Methods for various types of data
 * */

//0 -> 999 (4)
void printPid(int16_t value){
    if(value < 100){
        osd.write(' ');
    }
    osd.print(value/10.0, 1);
}

//0 -> 999 (4)
void printRate(int16_t value){
    osd.print(value/100.0, 2);
}

// (5)
void printFlightMode(bool leftJustified){
    uint8_t m = telemetry.mode;
    if(leftJustified){
        osd.print((m==0)?F("ACRO "):((m==1)?F("ANGLE"):((m==2)?F("3D   "):F("     "))));
    } else {
        osd.print((m==0)?F(" ACRO"):((m==1)?F("ANGLE"):((m==2)?F("   3D"):F("     "))));
    }
}

//0 -> 99999 (7)
void printCurent(int32_t value){
    if(value < 10000){
        osd.write(' ');
        if(value < 1000){
            osd.write(' ');
        }
    }
    osd.print(value/100.0, 2); osd.write('A');
}

//0 -> 9999 (6)
void printVoltage(int32_t value){
    if(value < 1000){
        osd.write(' ');
    }
    osd.print(value/100.0, 2); osd.write('V');
}

//0 -> 99999 (8) poles is usually 14
void printRpm(int32_t value, uint8_t poles){
    value = (value*32)/poles;
    if(value < 10000){
        osd.write(' ');
        if(value < 1000){
            osd.write(' ');
            if(value < 100){
                osd.write(' ');
                if(value < 10){
                    osd.write(' ');
                }
            }
        }
    }
    osd.print(value); osd.print("rpm");
}

//0 -> 9999 (7)
void printMah(int32_t value){
    if(value < 1000){
        osd.write(' ');
        if(value < 100){
            osd.write(' ');
            if(value < 10){
                osd.write(' ');
            }
        }
    }
    osd.print(value); osd.print("mAh");
}

//0 -> 100 (4)
void printPercentage(uint8_t data){
    if(data < 100){
        osd.write(' ');
        if(data < 10){
            osd.write(' ');
        }
    }
    osd.print(data); osd.write('%');
}

//-99 -> +999 (4)
void printTemperature(int16_t value){
    if(value < 100 && value >= 0){
        osd.write(' ');
    }
    if(value < 10 && value > -10){
        osd.write(' ');
    }
    osd.print(value); osd.write('C');
}

//takes ms (5)
void printDuration(uint32_t duration){
    duration /= 1000;
    uint8_t seconds = duration % 60;
    uint32_t minutes = (duration - seconds) / 60;
    if(minutes < 10){
        osd.write(' ');
    }
    osd.print(minutes);
    osd.write(':');
    if(seconds < 10){
        osd.write('0');
    }
    osd.print(seconds);
}

// (3)
void printAuxConfig(kiss_aux_config_type_t type){
    kiss_aux_config_t * conf = NULL;
    if(type > AUX_LED){
        conf = &settings.aux_4;
    } else {
        conf = &settings.aux[type];
    }

    osd.write(0x30+conf->channel);
    switch(conf->level){
        case AUX_LOW:       osd.print(F("LO")); break;
        case AUX_LOW_MED:   osd.print(F("LM")); break;
        case AUX_MED:       osd.print(F("ME")); break;
        case AUX_MED_HIGH:  osd.print(F("MH")); break;
        case AUX_HIGH:      osd.print(F("HI")); break;
        default: break;
    }
}

//Detects if Arm switch is set
static bool hasArmSwitch(){
    return ((settings.aux[AUX_ARM].channel != AUX_NONE) && (settings.aux[AUX_ARM].level != AUX_OFF));
}

// Reads the stick values from telemetry and determines if a combination is active
static uint8_t sticksGetValue(){
    static sticks_value_t sticksLastValue = STICKS_ERR;
    static sticks_value_t sticksValueBeforeLast = STICKS_ERR;
    if(!KISS_THROTTLE_IS_MID(telemetry.throttle)){
        sticksLastValue = STICKS_ERR;
        return sticksLastValue;
    }
    sticks_value_t sticks = STICKS_ERR;
    if(KISS_AUX_IS_MID(AXIS_YN)){
        if(KISS_AUX_IS_MID(AXIS_UD)){
            if(KISS_AUX_IS_LOW(AXIS_LR)){
                sticks = STICKS_LEFT;
            } else if(KISS_AUX_IS_HIGH(AXIS_LR)){
                sticks = STICKS_RIGHT;
            } else {
                sticks = STICKS_NONE;
            }
        } else if(KISS_AUX_IS_HIGH(AXIS_UD)){
            sticks = STICKS_UP;
        } else {
            sticks = STICKS_DOWN;
        }
    } else if(KISS_AUX_IS_LOW(AXIS_YN) && KISS_AUX_IS_MID(AXIS_UD) && KISS_AUX_IS_MID(AXIS_LR)){
        sticks = STICKS_NO;
    } else if(KISS_AUX_IS_MID(AXIS_UD)){
        if(KISS_AUX_IS_MID(AXIS_LR)){
            sticks = STICKS_YES;
        } else if(KISS_AUX_IS_LOW(AXIS_LR)){
            sticks = STICKS_MENU;
        }
    }
    if(sticks == sticksLastValue && sticks == sticksValueBeforeLast){
        return sticksLastValue;
    }
    sticksValueBeforeLast = sticksLastValue;
    sticksLastValue = sticks;
    return STICKS_ERR;
}

// Swich the current view
static void setCurrentView(uint8_t view){
    currentView = (view_index_t)view;
    lastViewSwitch = millis();
}

/*
 * PIDs and Rates Table
 * */

static void drawTable(){
    osd.setCursor(0,1);
    osd.println(F(PID_TABLE_TOP));
    osd.println(F(PID_TABLE_ROW));
    osd.println(F(PID_TABLE_MID));
    osd.println(F(PID_TABLE_ROW));
    osd.println(F(PID_TABLE_MID));
    osd.println(F(PID_TABLE_ROW));
    osd.println(F(PID_TABLE_MID));
    osd.println(F(PID_TABLE_ROW));
    osd.println(F(PID_TABLE_BOT));
    osd.setCursor(5,10); osd.print(F("CANCEL"));
    osd.setCursor(18,10); osd.print(F("SAVE"));
    table_current_index = 0;
}

static void updateTableIndex(){
    if(currentSticks){
        if(currentSticks == STICKS_LEFT){
            if(!table_current_index){
                table_current_index = (TABLE_NUM_INDEXES - 1);
            } else {
                table_current_index -= 1;
            }
        } else if(currentSticks == STICKS_RIGHT){
            table_current_index += 1;
        } else if(currentSticks == STICKS_DOWN){
            table_current_index += 3;
            table_current_index %= TABLE_NUM_INDEXES;
        } else if(currentSticks == STICKS_UP){
            if(table_current_index > 2){
                table_current_index -= 3;
            } else {
                table_current_index = (table_current_index + TABLE_NUM_INDEXES) - 3;
            }
        }
    }
    if(table_current_index >= TABLE_NUM_INDEXES){
        table_current_index = 0;
    }
    uint8_t i;
    for(i=0; i<TABLE_NUM_INDEXES; i++){
        osd.setCursor(table_positions[i].x,table_positions[i].y);
        osd.write((table_current_index == i)?'>':' ');
    }
    if(table_current_index == 9 && currentSticks == STICKS_YES){
        //this is cancel
        update_settings = true;
        setCurrentView(VIEW_MAIN);
    } else if(table_current_index == 10 && currentSticks == STICKS_YES){
        //this is confirm
        if(kiss_save()){
            update_settings = true;
            setCurrentView(VIEW_MAIN);
        }
    }
}

/*
 * Board Info View
 * */

static void drawInfo(){
    uint8_t i;
    osd.setCursor((28 - strlen(name))/2,2);
    osd.print(name);
    for(i=0;i<esc_stats.count; i++){
        osd.setCursor(7,3+i);
        osd.print(F("ESC"));
        osd.print(i+1);
        osd.write(' ');
        uint8_t type = info.esc[i].type;
        if(type == 1){
            osd.print(F("KISS 8A"));
        } else if(type == 2){
            osd.print(F("KISS 16A"));
        } else if(type == 3){
            osd.print(F("KISS 24A"));
        } else {
            osd.print(F("UNKNOWN"));
            osd.print(type);
            update_info = true;
        }
    }
}

static void updateInfo(){
    if(currentSticks == STICKS_NO || currentSticks == STICKS_LEFT){
        setCurrentView(VIEW_MAIN);
    }
}

/*
 * PIDs View
 * */

static uint16_t * pidsValues[9] = {
        &settings.pid_p[0],
        &settings.pid_i[0],
        &settings.pid_d[0],
        &settings.pid_p[1],
        &settings.pid_i[1],
        &settings.pid_d[1],
        &settings.pid_p[2],
        &settings.pid_i[2],
        &settings.pid_d[2]
};

static void drawPids(){
    drawTable();

    osd.setCursor(10,2); osd.print(F("P"));
    osd.setCursor(16,2); osd.print(F("I"));
    osd.setCursor(22,2); osd.print(F("D"));
    osd.setCursor(2,4); osd.print(F("ROLL"));
    osd.setCursor(2,6); osd.print(F("PITCH"));
    osd.setCursor(2,8); osd.print(F("YAW"));

}

static void updatePids(){
    updateTableIndex();

    if(table_current_index < 9){
        uint16_t * value = pidsValues[table_current_index];
        if(currentSticks == STICKS_YES){
            if(table_current_index == 1 || table_current_index == 4 || table_current_index == 7){
                *value = *value + 1;
            } else {
                *value = *value + 100;
            }
        } else if(currentSticks == STICKS_NO){
            if(table_current_index == 1 || table_current_index == 4 || table_current_index == 7){
                if(*value) {
                    *value = *value - 1;
                }
            } else {
                if(*value > 99) {
                    *value = *value - 100;
                }
            }
        }
    }

    osd.setCursor(9,4);  printPid(*pidsValues[0]/100);
    osd.setCursor(15,4); printPid(*pidsValues[1]*10);
    osd.setCursor(21,4); printPid(*pidsValues[2]/100);
    osd.setCursor(9,6);  printPid(*pidsValues[3]/100);
    osd.setCursor(15,6); printPid(*pidsValues[4]*10);
    osd.setCursor(21,6); printPid(*pidsValues[5]/100);
    osd.setCursor(9,8);  printPid(*pidsValues[6]/100);
    osd.setCursor(15,8); printPid(*pidsValues[7]*10);
    osd.setCursor(21,8); printPid(*pidsValues[8]/100);
}

/*
 * Rates View
 * */

static int16_t * ratesValues[9] = {
        &settings.rc_rate[0],
        &settings.rc_expo[0],
        &settings.rc_curve[0],
        &settings.rc_rate[1],
        &settings.rc_expo[1],
        &settings.rc_curve[1],
        &settings.rc_rate[2],
        &settings.rc_expo[2],
        &settings.rc_curve[2]
};

static void drawRates(){
    drawTable();

    osd.setCursor(8,2); osd.print(F("RATE"));
    osd.setCursor(14,2); osd.print(F("EXPO"));
    osd.setCursor(20,2); osd.print(F("CURVE"));
    osd.setCursor(2,4); osd.print(F("ROLL"));
    osd.setCursor(2,6); osd.print(F("PITCH"));
    osd.setCursor(2,8); osd.print(F("YAW"));
}

static void updateRates(){
    updateTableIndex();

    if(table_current_index < 9){
        int16_t * value = ratesValues[table_current_index];
        if(currentSticks == STICKS_YES){
            *value = *value + 10;
        } else if(currentSticks == STICKS_NO){
            *value = *value - 10;
        }
    }

    osd.setCursor(9,4);  printRate(*ratesValues[0]/10);
    osd.setCursor(15,4); printRate(*ratesValues[1]/10);
    osd.setCursor(21,4); printRate(*ratesValues[2]/10);
    osd.setCursor(9,6);  printRate(*ratesValues[3]/10);
    osd.setCursor(15,6); printRate(*ratesValues[4]/10);
    osd.setCursor(21,6); printRate(*ratesValues[5]/10);
    osd.setCursor(9,8);  printRate(*ratesValues[6]/10);
    osd.setCursor(15,8); printRate(*ratesValues[7]/10);
    osd.setCursor(21,8); printRate(*ratesValues[8]/10);
}

/*
 * Main Menu View
 * */

static void drawMainMenu(){
    osd.setCursor(8,2); osd.print(F("Main Menu"));
    osd.setCursor(6,3); osd.print(F(TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL));
    osd.setCursor(8,4); osd.print(F("Board Info"));
    osd.setCursor(8,5); osd.print(F("PID Setup"));
    osd.setCursor(8,6); osd.print(F("RC Rates"));
    osd.setCursor(8,7); osd.print(F("Last Stats"));
    osd.setCursor(8,8); osd.print(F("Exit Menu"));
}

static void updateMainMenu(){
    if(currentSticks == STICKS_UP){
        if(!mainMenuIndex){
            mainMenuIndex = VIEW_OFF;
        } else {
            mainMenuIndex -= 1;
        }
    } else if(currentSticks == STICKS_DOWN){
        mainMenuIndex += 1;
        if(mainMenuIndex > VIEW_OFF){
            mainMenuIndex = 0;
        }
    }
    uint8_t i;
    for(i=0; i<(VIEW_MAIN); i++){
        osd.setCursor(4,4+i);
        osd.write((mainMenuIndex == i)?'>':' ');
        osd.write((mainMenuIndex == i)?'>':' ');
        osd.setCursor(19,4+i);
        osd.write((mainMenuIndex == i)?'<':' ');
        osd.write((mainMenuIndex == i)?'<':' ');
    }

    if(currentSticks == STICKS_RIGHT || currentSticks == STICKS_YES){
        setCurrentView((view_index_t)mainMenuIndex);
        if(mainMenuIndex == VIEW_OFF){
            mainMenuIndex = 0;
        }
    } else if(currentSticks == STICKS_LEFT || currentSticks == STICKS_NO){
        setCurrentView(VIEW_OFF);
        if(mainMenuIndex == VIEW_OFF){
            mainMenuIndex = 0;
        }
    }
}

/*
 * Flight Stats Collection
 * */

static void startStats(){
    memset((uint8_t*)&flight_stats, 0, sizeof(flight_stats_t));
    flight_stats.duration = millis();
    flight_stats.min_voltage = 4000;
    flight_stats.min_signal = 100;
}

static void collectStats(){
    int16_t voltage = esc_stats.count?esc_stats.voltage:telemetry.voltage;
    if(voltage < flight_stats.min_voltage){
        flight_stats.min_voltage = voltage;
    }
    if((100 - telemetry.failsafe) < flight_stats.min_signal){
        flight_stats.min_signal = (100 - telemetry.failsafe);
    }
    if(!esc_stats.count){
        return;
    }
    if(esc_stats.current > flight_stats.max_current){
        flight_stats.max_current = esc_stats.current;
    }
    if(esc_stats.rpm > flight_stats.max_rpm){
        flight_stats.max_rpm = esc_stats.rpm;
    }
    if(esc_stats.temperature > flight_stats.max_temperature){
        flight_stats.max_temperature = esc_stats.temperature;
    }
}

static void stopStats(){
    flight_stats.duration = millis() - flight_stats.duration;
}

/*
 * Flight Stats View
 * */

static void drawStats(){
    //show flight stats
    osd.setCursor(7,1);  osd.print(F("Flight Stats"));
    osd.setCursor(2,2);  osd.print(F(TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL TB_HL));
    osd.setCursor(2,3);  osd.print(F("Flight Duration:  ")); printDuration(flight_stats.duration);
    osd.setCursor(2,4);  osd.print(F("Min Signal     :   ")); printPercentage(flight_stats.min_signal);
    osd.setCursor(2,5);  osd.print(F("Min Voltage    : ")); printVoltage(flight_stats.min_voltage);
    if(esc_stats.count){
        osd.setCursor(2,6);  osd.print(F("Max Current    :")); printCurent(flight_stats.max_current);
        osd.setCursor(2,7);  osd.print(F("Used mAh       :")); printMah(esc_stats.used_ah);
        osd.setCursor(2,8);  osd.print(F("Max RPMs       :")); printRpm(flight_stats.max_rpm, 14);
        osd.setCursor(2,9); osd.print(F("Max Temperature:   ")); printTemperature(flight_stats.max_temperature);
    }
}

static void updateStats(){
    if((currentSticks && currentSticks != STICKS_ERR) || (millis() - lastViewSwitch) > 10000){
        setCurrentView(VIEW_OFF);
    }
}

/*
 * Last Flight View
 * */

static void updateLastFlight(){
    if(currentSticks && currentSticks != STICKS_ERR){
        setCurrentView(VIEW_MAIN);
    }
}

// Telemetry Live Stats - Always visible on the screen
void drawLiveStats(){
    video_mode_t mode = osd.detectVideoMode();
    static video_mode_t lastMode = mode;
    uint8_t bottomRow = 11;
    int16_t voltage = esc_stats.count?esc_stats.voltage:telemetry.voltage;
    if(mode == VIDEO_NONE){
        mode = lastMode;
    }
    if(mode == VIDEO_PAL){
        bottomRow += 3;
    }
    if(lastMode != mode){
        lastMode = mode;
        if(mode == VIDEO_PAL){
            //clear the bottom status line from NTSC
            osd.setCursor(2,bottomRow);
            uint8_t i;
            for(i=0;i<26;i++){
                osd.write(' ');
            }
        }
    }

    osd.setCursor(1,0);
    if(telemetry.armed){
        printDuration(millis() - flight_stats.duration);
    } else {
        printFlightMode(false);
    }
    osd.write(' '); printPercentage(100 - telemetry.failsafe);
    if(esc_stats.count){
        osd.write(' '); printTemperature(esc_stats.temperature); osd.write(' '); printRpm(esc_stats.rpm, 14);
    }

    osd.setCursor(2,bottomRow);
    printVoltage(voltage);
    if(esc_stats.count){
        osd.write(' '); printCurent(esc_stats.current); osd.write(' '); osd.write(' '); printMah(esc_stats.used_ah);
    }
}

/*
 * View Methods Table
 * */

static view_methods_t views[VIEW_BAD] = {
        {drawInfo, updateInfo, NULL},           //Info
        {drawPids, updatePids, NULL},           //PIDs
        {drawRates, updateRates, NULL},         //Rates
        {drawStats, updateLastFlight, NULL},    //Last Flight
        {NULL, NULL, NULL},                     //Off
        {drawMainMenu, updateMainMenu, NULL},   //Main Menu
        {drawStats, updateStats, NULL}          //Flight Stats
};

/*
 * FC Event Logic
 * */

void loopDisarmed(){
    if((millis() - lastViewSwitch) < 1000){
        currentSticks = STICKS_ERR;
    }
    if(currentView == VIEW_OFF && currentSticks == STICKS_MENU){
        setCurrentView(VIEW_MAIN);
    }
    if(currentView != lastView){
        if(lastView != VIEW_OFF && views[lastView].deinit){
            views[lastView].deinit();
        }
        lastView = currentView;
        osd.clearDisplay();
        if(views[currentView].init){
            views[currentView].init();
        } else {
            setCurrentView(VIEW_OFF);
            lastView = VIEW_OFF;
        }
    }
    drawLiveStats();
    if(currentView != VIEW_OFF && views[currentView].update){
        views[currentView].update();
    }
}

void onDisarm(){
    stopStats();
    osd.clearDisplay();
    setCurrentView(VIEW_STATS);
    lastView = VIEW_OFF;
    update_info = true;
    update_settings = true;
}

void onArm(){
    startStats();
    osd.clearDisplay();
    setCurrentView(VIEW_OFF);
    lastView = VIEW_OFF;
}

void loopArmed(){
    drawLiveStats();
}

void loop(){
    static uint8_t itteration = 0;
    static uint8_t lastArmed = 0;

    if(kiss_update()){
        if(!telemetry.armed){
            currentSticks = (sticks_value_t)sticksGetValue();
        }
        if(lastArmed != telemetry.armed){
            lastArmed = telemetry.armed;
            if(!telemetry.armed){
                onDisarm();
            } else {
                onArm();
            }
            itteration = 9;
        }
        itteration++;
        if(telemetry.armed) {
            collectStats();
        }
        if(itteration > 9){
            itteration = 0;
            if(telemetry.armed || !hasArmSwitch()){
                loopArmed();
            } else {
                loopDisarmed();
            }
            osd.display();
        }
    } else {
        msp_delay(1000);
    }
}

void setup(){
    uart_init(115200);
    memset(&telemetry, 0, sizeof(kiss_telemetry_t));
    memset(&settings, 0, sizeof(kiss_settings_t));
    //kiss_set_dbg_cb(&onDbg);
    msp_on_packet(&onMSP);
    osd.begin();
    //osd.begin(VIDEO_PAL);
    osd.setPrintMode(PRINT_BUFFERED);
    osd.setOffsetLeft(18);
    osd.setOffsetTop(13);
    osd.clearDisplay();
    osd.setCursor(2,2);
    osd.println(F("Connecting to KISS FC..."));
    osd.display();

    while(!kiss_update()){
        msp_delay(500);
    }

    osd.clearDisplay();
    osd.display();
}
/*
void onDbg(uint8_t cmd, uint8_t * data, uint8_t len){
    if(osd.getVideoMode() != VIDEO_PAL){
        return;
    }
    osd.setCursor(0,11);
    osd.print(F("DBG["));
    osd.print(len);
    osd.print(F("]:"));
    uint8_t i;
    for(i=0;i<len;i++){
        osd.write(' ');
        osd.print(data[i], HEX);
    }
    osd.display();
    delay(1000);
}
*/
