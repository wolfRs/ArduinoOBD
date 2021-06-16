/*************************************************************************
* Reference code for Freematics OBD-II Advanced Telematics Kit
* Visit http://freematics.com for more information
* Distributed under BSD license
* Written by Stanley Huang <support@freematics.com.au>
*************************************************************************/

/******
 Subaru Custom PIDS
*******/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <OBD.h>
#include <MultiLCD.h>
#include <TinyGPS.h>
#include "config.h"
#if ENABLE_DATA_LOG
#include <SD.h>
#endif
#include "Narcoleptic.h"
#include "images.h"
#include "datalogger.h"

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_CONNECTED 0x4
#define STATE_GPS_READY 0x8
#define STATE_MEMS_READY 0x10
#define STATE_GUI_ON 0x20

#if USE_GPS
// GPS logging can only be enabled when there is additional hardware serial UART
#define GPSUART Serial2
TinyGPS gps;
#endif


static uint8_t lastFileSize = 0;
static uint32_t lastRefreshTime = 0;
static uint32_t distance = 0;
static uint32_t startTime = 0;
static uint16_t lastSpeed = 0;
static uint32_t lastSpeedTime = 0;
static uint32_t gpsDate = 0;
#if USE_GPS
static uint32_t lastGPSDataTime = 0;
static int gpsSpeed = -1;
#endif

byte state = 0;

void processMEMS();
void processGPS();

CDataLogger logger;

#ifdef OBD_ADAPTER_I2C
class CMyOBD : public COBDI2C
#else
class CMyOBD : public COBD
#endif
{
public:
    void dataIdleLoop()
    {
        if (!(state & STATE_GUI_ON)) {
          delay(10);
          return;
        }

#if USE_GPS
        uint32_t t = millis();
        while (GPSUART.available() && millis() - t < MAX_GPS_PROCESS_TIME) {
            processGPS();
        }
#endif
    }
};

CMyOBD obd;

void setColorByValue(int value, int threshold1, int threshold2, int threshold3)
{
    if (value < 0) value = -value;
    if (value < threshold1) {
      lcd.setColor(RGB16_WHITE);
    } else if (value < threshold2) {
      byte n = (uint32_t)(threshold2 - value) * 255 / (threshold2 - threshold1);
      lcd.setColor(255, 255, n);
    } else if (value < threshold3) {
      byte n = (uint32_t)(threshold3 - value) * 255 / (threshold3 - threshold2);
      lcd.setColor(255, n, 0);
    } else {
      lcd.setColor(255, 0, 0);
    }
}

void showPIDData(byte pid, int value)
{
    char buf[8];
    switch (pid) {
    case PID_RPM:
        lcd.setFontSize(FONT_SIZE_XLARGE);
        lcd.setCursor(14, 8);
        if (value >= 10000) break;
        setColorByValue(value, 2500, 4000, 6000);
        lcd.printInt(value, 6);
        break;
    case PID_SPEED:
        if (value < 1000) {
            lcd.setFontSize(FONT_SIZE_XLARGE);
            lcd.setCursor(50,3);
            setColorByValue(value, 60, 100, 160);
            lcd.printInt(value, 3);

#if USE_GPS
            if (gpsSpeed != -1) {
                lcd.setFontSize(FONT_SIZE_SMALL);
                lcd.setCursor(110, 3);
                lcd.setColor(RGB16_YELLOW);
                int diff = gpsSpeed - value;
                if (diff >= 0) {
                    lcd.write('+');
                    lcd.printInt(diff);
                } else {
                    lcd.write('-');
                    lcd.printInt(-diff);
                }
                lcd.write(' ');
            }
#endif
        }
        break;
    case PID_THROTTLE:{
        
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(102, 27);
        if (value >= 100) value = 99;
        setColorByValue(value, 50, 75, 100);
        lcd.printInt(value, 2);
        break;
    }
    case PID_ENGINE_FUEL_RATE:
        if (value < 100) {
            lcd.setColor(RGB16_WHITE);
            lcd.setFontSize(FONT_SIZE_MEDIUM);
            lcd.setCursor(102, 30);
            lcd.printInt(value, 2);
        }
        break;
    case PID_INTAKE_TEMP:
        if (value >= 0 && value < 100) {
            lcd.setColor(RGB16_WHITE);
            lcd.setFontSize(FONT_SIZE_MEDIUM);
            lcd.setCursor(102, 33);
            lcd.printInt(value, 2);
        }
        break;

    }
    lcd.setColor(RGB16_WHITE);
}

void fadeOutScreen()
{
    // fade out backlight
    for (int n = 254; n >= 0; n--) {
        lcd.setBackLight(n);
        delay(3);
    }
}

void fadeInScreen()
{
    for (int n = 1; n <= 255; n++) {
        lcd.setBackLight(n);
        delay(6);
    }
}

void initScreen()
{
    lcd.clear();
    lcd.draw2x(frame0[0], 78, 78);
    lcd.setXY(162, 0);
    lcd.draw2x(frame0[0], 78, 78);
    lcd.setXY(324, 0);
    lcd.draw2x(frame0[0], 78, 78);
    lcd.setXY(0, 164);
    lcd.draw2x(frame0[0], 78, 78);
    lcd.setXY(162, 164);
    lcd.draw2x(frame0[0], 78, 78);
    lcd.setXY(324, 164);
    lcd.draw2x(frame0[0], 78, 78);

    lcd.setColor(RGB16_CYAN);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(110,4);
    lcd.print("km/h");
    lcd.setCursor(110, 9);
    lcd.print("RPM");
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(110, 14);
    lcd.print("TORQUE");
    lcd.setCursor(110, 15);
    lcd.print("Nm");

    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(180, 3);
    lcd.print("OilPres");
    lcd.setCursor(180, 6);
    lcd.print("OilTemp ");
    lcd.setCursor(180, 9);
    lcd.print("Boost");
    lcd.setCursor(180, 12);
    lcd.print("Coolant");
    lcd.setCursor(180, 15);
    lcd.print("Fuel Lvl");
 
    lcd.setCursor(16, 24);
    lcd.print("Battery     V");
    lcd.setCursor(16, 27);
    lcd.print("Throttle    %");
    lcd.setCursor(16, 30);
    lcd.print("Fuel Con    L/h");
    lcd.setCursor(16, 33);
    lcd.print("Intake:     C");
    lcd.setCursor(16, 36);
    lcd.print("Engine:     %");

//working 2

    lcd.setCursor(180, 24);
    lcd.print("Elapsed");
    lcd.setCursor(180, 27);
    lcd.print("Dist.");
    lcd.setCursor(180, 30);
    lcd.print("Avg.Sp");
    lcd.setCursor(180, 33);
    lcd.print("CVT Temp:");
    lcd.setCursor(180, 36);
//    lcd.print("SAT Nr:");
    lcd.print("Fuel Temp:");

    lcd.setCursor(340, 3);
    lcd.print("Accelerometer");
    lcd.setCursor(356, 8);
    lcd.print("Gyroscope");
    lcd.setCursor(348, 13);
    lcd.print("Temperature");

    lcd.setCursor(348, 24);
    lcd.print("OBD Interval");

    lcd.setCursor(348, 29);
    lcd.print("GPS Interval");

    lcd.setCursor(352, 34);
    lcd.print("Data Size");
    state |= STATE_GUI_ON;

    fadeInScreen();
}

#if ENABLE_DATA_LOG
bool checkSD()
{
    Sd2Card card;
    SdVolume volume;
    state &= ~STATE_SD_READY;
    pinMode(SS, OUTPUT);

    lcd.setFontSize(FONT_SIZE_MEDIUM);
    if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
        const char* type;
        switch(card.type()) {
        case SD_CARD_TYPE_SD1:
            type = "SD1";
            break;
        case SD_CARD_TYPE_SD2:
            type = "SD2";
            break;
        case SD_CARD_TYPE_SDHC:
            type = "SDHC";
            break;
        default:
            type = "SDx";
        }

        lcd.print(type);
        lcd.write(' ');
        if (!volume.init(card)) {
            lcd.println("No FAT!");
            return false;
        }

        uint32_t volumesize = volume.blocksPerCluster();
        volumesize >>= 1; // 512 bytes per block
        volumesize *= volume.clusterCount();
        volumesize >>= 10;

        lcd.print((int)volumesize);
        lcd.print("MB");
    } else {
        lcd.print("SD Card ");
        lcd.setColor(RGB16_RED);
        lcd.draw(cross, 16, 16);
        lcd.setColor(RGB16_WHITE);
        lcd.println();
        return false;
    }

    if (!SD.begin(SD_CS_PIN)) {
        lcd.println("Bad SD");
        return false;
    }

    state |= STATE_SD_READY;
    return true;
}
#endif

#if USE_GPS
void processGPS()
{
    // process GPS data
    char c = GPSUART.read();
    if (!gps.encode(c))
        return;

    // parsed GPS data is ready
    uint32_t time;
    uint32_t date;

    logger.dataTime = millis();

    gps.get_datetime(&date, &time, 0);
    if (date != gpsDate) {
        // log date only if it's changed and valid
        int year = date % 100;
        if (date < 1000000 && date >= 10000 && year >= 15 && (gpsDate == 0 || year - (gpsDate % 100) <= 1)) {
          logger.logData(PID_GPS_DATE, (int32_t)date);
          gpsDate = date;
        }
    }
    logger.logData(PID_GPS_TIME, (int32_t)time);

    int32_t lat, lng;
    gps.get_position(&lat, &lng, 0);

    byte sat = gps.satellites();

    // show GPS data interval
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    if (lastGPSDataTime) {
        lcd.setCursor(380, 31);
        lcd.printInt((uint16_t)logger.dataTime - lastGPSDataTime);
        lcd.print("ms");
        lcd.printSpace(2);
    }

    // keep current data time as last GPS time
    lastGPSDataTime = logger.dataTime;

    // display UTC date/time
    lcd.setFlags(FLAG_PAD_ZERO);
    lcd.setCursor(216, 33);
    lcd.printLong(time, 8);

//    // display number of satellites
//    if (sat < 100) {
        lcd.setCursor(216, 36);
        lcd.printInt(sat);
        lcd.write(' +');
//    }
}
#endif

void processMEMS()
{
    int acc[3];
    int gyro[3];
    int temp;

    if (!obd.memsRead(acc, gyro, 0, &temp)) return;

    logger.dataTime = millis();

    acc[0] /= ACC_DATA_RATIO;
    acc[1] /= ACC_DATA_RATIO;
    acc[2] /= ACC_DATA_RATIO;
    gyro[0] /= GYRO_DATA_RATIO;
    gyro[1] /= GYRO_DATA_RATIO;
    gyro[2] /= GYRO_DATA_RATIO;

    // display MEMS data
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setCursor(362, 5);
    setColorByValue(acc[0], 50, 100, 200);
    lcd.print(acc[0]);
    setColorByValue(acc[1], 50, 100, 200);
    lcd.write('/');
    lcd.print(acc[1]);
    setColorByValue(acc[2], 50, 100, 200);
    lcd.write('/');
    lcd.print(acc[2]);
    Serial.println(acc[2]);
    lcd.printSpace(8);

    // display gyro data
    lcd.setCursor(374, 10);
    lcd.setColor(RGB16_WHITE);
    lcd.print(gyro[0]);
    lcd.write('/');
    lcd.print(gyro[1]);
    lcd.write('/');
    lcd.print(gyro[2]);
    lcd.printSpace(8);

    // display adapter temperature
    lcd.setCursor(382, 15);
    lcd.setColor(RGB16_WHITE);
    lcd.print((float)temp / 10, 1);
    lcd.print("C ");

    // log x/y/z of accelerometer
    logger.logData(PID_ACC, acc);
    // log x/y/z of gyro meter
    logger.logData(PID_GYRO, gyro);
}

void logOBDData(byte pid, int value)
{
    char buffer[64];
    // send query for OBD-II PID
    logger.dataTime = millis();
    // display data
    showPIDData(pid, value);

    // log data to SD card
    logger.logData(0x100 | pid, value);

//custom pids
    //oil pressure?
    if(pid == ENGINE_OIL_PRESSURE){
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(260, 3);
      float pressure = value * 0.01;
      lcd.print(pressure);
      lcd.print("bar");
    }
//CVT TEMP
    if(pid == CVT_TEMP){
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(216, 33);
        lcd.printInt(value);
        lcd.write(" C");
    }
    
    //oil temp
    if(pid == ENGINE_OIL_TEMP){
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(260, 6);
      lcd.printInt(value);
      lcd.print(" C");
    }

    //boost?
    if(pid == MANIFOLD_PRESSURE){
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(260, 9);
      float boost = value * 0.01;
      lcd.print(boost);
    }

    //coolant
    if(pid == PID_COOLANT_TEMP){
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(260, 12);
      lcd.printInt(value);
      lcd.print(" C");
    }

    //fuel
    if(pid == PID_FUEL_LEVEL){
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(260, 15);
      lcd.printInt(value);
      lcd.print(" L");
    }

    //engine load
    if(pid == PID_ENGINE_LOAD){
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(90, 36); 
      if (value >= 100) value = 99;
      setColorByValue(value, 75, 80, 100);
      lcd.printInt(value, 3);
    }

    //torque
    if(pid == REFERENCE_TORQUE){
      lcd.setFontSize(FONT_SIZE_XLARGE);
      setColorByValue(value, 200, 300, 400);
      lcd.setCursor(50, 13);
      lcd.printInt(value);
    }

    //fuel temp
    if(pid == FUEL_TEMP){
      lcd.setFontSize(FONT_SIZE_MEDIUM);
      lcd.setCursor(217, 36);
      lcd.printInt(value);
      lcd.print("C");
    }
    
    if (pid == PID_SPEED) {
        // estimate distance travelled since last speed update
        distance += (uint32_t)(value + lastSpeed) * (logger.dataTime - lastSpeedTime) / 6000;
        // display speed
        //display distance
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(255, 27);
        lcd.printInt(distance / 1000);
        lcd.write('.');
        lcd.printInt(((uint16_t)distance % 1000) / 100);
        lcd.print("km");
        // calculate and display average speed
        int avgSpeed = (unsigned long)distance * 3600 / (millis() - startTime);
        lcd.setCursor(250, 30);
        lcd.printInt(avgSpeed);
        lcd.print("km/h");
        
        lastSpeed = value;
        lastSpeedTime = logger.dataTime;
    }
#if ENABLE_DATA_LOG
    // flush SD data every 1KB
    byte dataSizeKB = logger.dataSize >> 10;
    if (dataSizeKB != lastFileSize) {
        logger.flushFile();
        lastFileSize = dataSizeKB;
        // display logged data size
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(380, 36);
        lcd.print((unsigned int)(logger.dataSize >> 10));
        lcd.print("KB");
    }
#endif
}

void processTouch()
{
    int x, y;
    if (lcd.getTouchData(x, y)) {
      Serial.print("X:");
      Serial.print(x);
      Serial.print(" Y:");
      Serial.println(y);
    }
}

void showECUCap()
{
    static const byte PROGMEM pidlist[] = {FUEL_TEMP,CVT_TEMP,PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_MAF_FLOW, PID_THROTTLE, PID_AUX_INPUT,
        PID_EGR_ERROR, PID_COMMANDED_EVAPORATIVE_PURGE, PID_FUEL_LEVEL, PID_CONTROL_MODULE_VOLTAGE, PID_ABSOLUTE_ENGINE_LOAD, PID_AMBIENT_TEMP, PID_COMMANDED_THROTTLE_ACTUATOR, PID_ETHANOL_FUEL,
        PID_FUEL_RAIL_PRESSURE, ENGINE_OIL_TEMP, MANIFOLD_PRESSURE, REFERENCE_TORQUE, ENGINE_OIL_PRESSURE};

    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setColor(RGB16_WHITE);
    for (byte i = 0, n = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++, n += 2) {
      byte pid = pgm_read_byte(pidlist + i);
      bool valid = obd.isValidPID(pid);
      lcd.setCursor(320, n);
      lcd.setColor(valid ? RGB16_GREEN : RGB16_RED);
      lcd.draw(valid ? tick : cross, 16, 16);
      lcd.setColor(RGB16_WHITE);
      lcd.print(" 0");
      lcd.print((int)pid | 0x100, HEX);
    }
    int values[sizeof(pidlist)];
    bool scanned = false;
    bool touched = false;
    for (uint32_t t = millis(); millis() - t < 5000; ) {
      for (byte i = 0, n = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++, n += 2) {
          byte pid = pgm_read_byte(pidlist + i);
          if (obd.isValidPID(pid)) {
              int value;
              lcd.setCursor(392, n);
              if (obd.readPID(pid, value)) {
                if (!scanned || value == values[i])
                  lcd.setColor(RGB16_CYAN);
                else if (value > values[i])
                  lcd.setColor(RGB16_GREEN);
                else
                  lcd.setColor(RGB16_RED);
                byte n = lcd.print(value);
                for (; n < 4; n++) lcd.print(' ');
                values[i] = value;
              } else {
                lcd.setColor(RGB16_YELLOW);
                lcd.print("N/A");
              }
          }
       }
       scanned = true;
    }
}

void reconnect()
{
    fadeOutScreen();
#if ENABLE_DATA_LOG
    logger.closeFile();
#endif
    lcd.clear();
    state &= ~(STATE_OBD_READY | STATE_GUI_ON);
    //digitalWrite(SD_CS_PIN, LOW);
    for (;;) {
        if (obd.init())
            break;
        
        obd.enterLowPowerMode();
        Narcoleptic.delay(10000);
        obd.leaveLowPowerMode();
    }
    // re-initialize
    state |= STATE_OBD_READY;
    startTime = millis();
    lastSpeedTime = startTime;
    lastSpeed = 0;
    distance = 0;
#if ENABLE_DATA_LOG
    logger.openFile();
#endif
    initScreen();
}

// screen layout related stuff
void showStates()
{
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setColor(RGB16_WHITE);
    lcd.setCursor(0, 10);
    lcd.print("MEMS ");
    lcd.setColor((state & STATE_MEMS_READY) ? RGB16_GREEN : RGB16_RED);
    lcd.draw((state & STATE_MEMS_READY) ? tick : cross, 16, 16);

#if USE_GPS
    lcd.setColor(RGB16_WHITE);
    lcd.setCursor(60, 10);
    lcd.print(" GPS ");
    if (state & STATE_GPS_CONNECTED) {
        lcd.setColor(RGB16_GREEN);
        lcd.draw(tick, 16, 16);
    } else {
        lcd.setColor(RGB16_RED);
        lcd.draw(cross, 16, 16);
    }
#endif
    lcd.setColor(RGB16_WHITE);
}

void testOut()
{
    const char cmds[][6] = {"ATZ\r", "ATH1\r", "ATRV\r", "0100\r", "0902\r"};
    char buf[128];
    lcd.setFontSize(FONT_SIZE_SMALL);
    lcd.setCursor(0, 13);

    for (byte i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
      const char* cmd = cmds[i];
      lcd.setColor(RGB16_WHITE);
      lcd.print("Sending ");
      lcd.println(cmd);
      Serial.println(cmd);
      lcd.setColor(RGB16_CYAN);
      if (obd.sendCommand(cmd, buf, sizeof(buf))) {
        char *p = strstr(buf, cmd);
        if (p)
            p += strlen(cmd);
        else
            p = buf;
        Serial.println(p);
        while (*p == '\r') p++;
        while (*p) {
            lcd.write(*p);
            if (*p == '\r' && *(p + 1) != '\r') {
                lcd.write('\n');
            }
            p++;
        }
        lcd.println();
      } else {
        lcd.println("Timeout");
        Serial.println("Timeout");
      }
      delay(500);
    }
    lcd.println();
}

void setup()
{
  Serial.begin(115200);
#if USE_GPS
    GPSUART.begin(GPS_BAUDRATE);
    lastGPSDataTime = 0;
#endif
    logger.initSender();

    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);

    
    lcd.setColor(0xFFE0);
    lcd.println("MEGA LOGGER HD - OBD-II/GPS/MEMS  Subaru Custom PIDS");
    lcd.println();
    lcd.setColor(RGB16_WHITE);

#if ENABLE_DATA_LOG
    if (checkSD()) {
        uint16_t index = logger.openFile();
        lcd.println();
        if (index > 0) {
            lcd.print("File ID:");
            lcd.println(index);
        } else {
            lcd.println("No File");
        }
    }
#endif

    byte version = obd.begin();
#ifdef OBD_ADAPTER_I2C
    lcd.print("OBD-II I2C Adapter ");
#else
    lcd.print("OBD-II UART Adapter ");
#endif
    if (version) {
      lcd.print("Ver. ");
      lcd.print(version / 10);
      lcd.print('.');
      lcd.println(version % 10);
    } else {
      lcd.setColor(RGB16_RED);
      lcd.draw(cross, 16, 16);
      lcd.setColor(RGB16_WHITE);
    }

#ifdef OBD_ADAPTER_I2C
    Wire.begin();
#endif
    if (version && obd.memsInit())
      state |= STATE_MEMS_READY;

    showStates();

#if USE_GPS
    unsigned long t = millis();
    while (GPSUART.available()) GPSUART.read();
    do {
        if (GPSUART.available() && GPSUART.read() == '\r') {
            state |= STATE_GPS_CONNECTED;
            break;
        }
    } while (millis() - t <= 2000);
    showStates();
#endif

    // this will send a bunch of commands and display response
    testOut();

    // initialize the OBD until success
    while (!obd.init(OBD_PROTOCOL));
    state |= STATE_OBD_READY;

    char buf[64];
    if (obd.getVIN(buf, sizeof(buf))) {
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setColor(RGB16_WHITE);
        lcd.print("VIN:");
        lcd.setColor(RGB16_YELLOW);
        lcd.println(buf);
    }

    uint16_t dtc[6];
    int num = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    lcd.setColor(RGB16_WHITE);
    lcd.print(num);
    lcd.println(" DTC found");
    if (num > 0) {
      lcd.setColor(RGB16_YELLOW);
      for (byte i = 0; i < num; i++) {
        lcd.print(dtc[i], HEX);
        lcd.print(' ');
      }
    }
    lcd.println();

    showECUCap();
    lcd.setCursor(0, 28);
    lcd.setColor(RGB16_YELLOW);
    lcd.setFontSize(FONT_SIZE_MEDIUM);

    fadeOutScreen();
    initScreen();

    startTime = millis();
    lastSpeedTime = startTime;
    lastRefreshTime = millis();
}


void loop()
{
    static byte index2 = 0;
    const byte pids[]= {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD, REFERENCE_TORQUE};
    const byte pids2[] = {FUEL_TEMP,CVT_TEMP, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_ENGINE_FUEL_RATE, ENGINE_OIL_TEMP, PID_MAF_FLOW, MANIFOLD_PRESSURE, PID_FUEL_LEVEL, PID_ENGINE_LOAD, ENGINE_OIL_PRESSURE};
    int values[sizeof(pids)] = {0};
    uint32_t pidTime = millis();
    // read multiple OBD-II PIDs
    byte results = obd.readPID(pids, sizeof(pids), values);
    pidTime = millis() - pidTime;
    if (results == sizeof(pids)) {
      for (byte n = 0; n < sizeof(pids); n++) {
        logOBDData(pids[n], values[n]);
      }
    }
    byte pid = pids2[index2 = (index2 + 1) % sizeof(pids2)];
    // check validation and read a single OBD-II PID
    if (obd.isValidPID(pid)) {
      int value;
      if (obd.readPID(pid, value)) {
        logOBDData(pid, value); 
      }
    }

    if (state & STATE_MEMS_READY) {
        processMEMS();
    }

    if (logger.dataTime -  lastRefreshTime >= 1000) {
      float v = obd.getVoltage();
      if (v > 0) {
        lcd.setCursor(84, 24);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.print(v, 1);
      }

      char buf[12];
        // display elapsed time
        unsigned int sec = (logger.dataTime - startTime) / 1000;
        sprintf(buf, "%02u:%02u", sec / 60, sec % 60);
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setCursor(250, 24);
        lcd.print(buf);
        lcd.print("h");
       
        
        // display OBD time
        if (results) {
          lcd.setCursor(10, 5);
          lcd.print((uint16_t)(pidTime / results));
          lcd.print("ms ");
        }
        lastRefreshTime = logger.dataTime;
    }

    if (obd.errors >= 3) {
        reconnect();
    }

#if USE_GPS
    if (millis() - lastGPSDataTime > GPS_DATA_TIMEOUT || gps.satellites() < 3) {
        // GPS not ready
        state &= ~STATE_GPS_READY;
    } else {
        // GPS ready
        state |= STATE_GPS_READY;
    }
#endif
}
