/*************************************************************************
* Reference code for Freematics OBD-II Advanced Telematics Kit
* Visit http://freematics.com for more information
* Distributed under BSD license
* Written by Stanley Huang <support@freematics.com.au>
*************************************************************************/

/****
 * print it all out
 * 
 ******/

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

    }
};

CMyOBD obd;

void showPIDData(int count, byte pid, int value, int xPos,int yPos)
{
      bool valid = obd.isValidPID(pid);
     
      lcd.setCursor(xPos, yPos);
      lcd.print(count);
      lcd.setColor(valid ? RGB16_GREEN : RGB16_RED);
      lcd.draw(valid ? tick : cross, 16, 16);
      lcd.setColor(RGB16_WHITE);
      lcd.print("pid: ");
      lcd.print(pid);
      lcd.print("(");
      lcd.print((int)pid | 0x100, HEX);
      lcd.print(")");
      lcd.print("-val:");
      lcd.print(value);
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
    lcd.setCursor(2,1);
    lcd.print("print it all out");
        char buf[64];
    if (obd.getVIN(buf, sizeof(buf))) {
        lcd.setFontSize(FONT_SIZE_MEDIUM);
        lcd.setColor(RGB16_WHITE);
        lcd.print("VIN:");
        lcd.setColor(RGB16_YELLOW);
        lcd.println(buf);
    }else{
      lcd.print(" no vin");
    }
    lcd.setColor(RGB16_CYAN);
    lcd.setFontSize(FONT_SIZE_MEDIUM);
   
    state |= STATE_GUI_ON;

    fadeInScreen();
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
    static const byte PROGMEM pidlist[] = {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_MAF_FLOW, PID_THROTTLE, PID_AUX_INPUT,
        PID_EGR_ERROR, PID_COMMANDED_EVAPORATIVE_PURGE, PID_FUEL_LEVEL, PID_CONTROL_MODULE_VOLTAGE, PID_ABSOLUTE_ENGINE_LOAD, PID_AMBIENT_TEMP, PID_COMMANDED_THROTTLE_ACTUATOR, PID_ETHANOL_FUEL,
        PID_FUEL_RAIL_PRESSURE};

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
    delay(500);
    Serial.begin(115200);
#if USE_GPS
    GPSUART.begin(GPS_BAUDRATE);
    lastGPSDataTime = 0;
#endif
    logger.initSender();

    lcd.begin();
    lcd.setFontSize(FONT_SIZE_MEDIUM);
    lcd.setColor(0xFFE0);
    lcd.println("MEGA LOGGER HD - OBD-II/GPS/MEMS print ALL");
    lcd.println();
    lcd.setColor(RGB16_WHITE);

    byte version = obd.begin();
#ifdef OBD_ADAPTER_I2C
    lcd.print("OBD I2C Adapter ");
#else
    lcd.print("OBD Firmware ");
#endif
    if (version) {
      lcd.print("Ver. ");
      lcd.print(version);
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

#if USE_GPS
    unsigned long t = millis();
    while (GPSUART.available()) GPSUART.read();
    do {
        if (GPSUART.available() && GPSUART.read() == '\r') {
            state |= STATE_GPS_CONNECTED;
            break;
        }
    } while (millis() - t <= 2000);
    
#endif

    // this will send a bunch of commands and display response
//    testOut();

    // initialize the OBD until success
//    int yOBDPos = 10;
//   const OBD_PROTOCOLS  protocols[]= {PROTO_ISO_9141_2, PROTO_KWP2000_5KBPS,PROTO_KWP2000_FAST,PROTO_CAN_11B_500K, PROTO_CAN_29B_500K ,PROTO_CAN_29B_250K ,PROTO_CAN_11B_250K  };
//   for (byte n = 0; n < sizeof(protocols); n++) {
      lcd.setCursor(5, 10);
//      byte proto = protocols[n];
      lcd.print("init obd: PROTO_AUTO  ");
      while (!obd.init(PROTO_AUTO ));
      state |= STATE_OBD_READY;
  
      char buf[64];
      if (obd.getVIN(buf, sizeof(buf))) {
          lcd.setFontSize(FONT_SIZE_MEDIUM);
          lcd.setColor(RGB16_WHITE);
          lcd.println("VIN:");
          lcd.setColor(RGB16_YELLOW);
          lcd.println(buf);
      }else{
        lcd.println("no vin");
      }
      lcd.println("init obd done");
    
    lcd.println("reading DTC");    
    uint16_t dtc[6];
    int num = obd.readDTC(dtc, sizeof(dtc) / sizeof(dtc[0]));
    lcd.setColor(RGB16_WHITE);
    lcd.print(num);
    lcd.println(" DTC found (error codes)");
    if (num > 0) {
      lcd.setColor(RGB16_YELLOW);
      for (byte i = 0; i < num; i++) {
        lcd.print(dtc[i], HEX);
        lcd.print(' ');
      }
    }
    lcd.println("reading DTC done");
    
    showECUCap();
    lcd.setCursor(0, 28);
    lcd.setColor(RGB16_YELLOW);
    lcd.setFontSize(FONT_SIZE_MEDIUM);

    fadeOutScreen();
    initScreen();
    
}


void loop()
{
    static byte index2 = 0;
    const byte pids[]= {PID_ENGINE_LOAD, PID_COOLANT_TEMP, PID_FUEL_PRESSURE, PID_INTAKE_MAP, PID_RPM, PID_SPEED, PID_TIMING_ADVANCE, PID_INTAKE_TEMP, PID_MAF_FLOW, PID_THROTTLE, PID_AUX_INPUT,
        PID_EGR_ERROR, PID_COMMANDED_EVAPORATIVE_PURGE, PID_FUEL_LEVEL, PID_CONTROL_MODULE_VOLTAGE, PID_ABSOLUTE_ENGINE_LOAD, PID_AMBIENT_TEMP, PID_COMMANDED_THROTTLE_ACTUATOR, PID_ETHANOL_FUEL,
        PID_FUEL_RAIL_PRESSURE,ENGINE_OIL_TEMP,ENGINE_OIL_PRESSURE,MANIFOLD_PRESSURE,REFERENCE_TORQUE,P_TURBO_PRESSURE,P_BOOST_PRES_CONT,CVT_TEMP,FUEL_TEMP };
    int values[sizeof(pids)] = {0};
    int yPos = 4;
    int xPos = 20;
    uint32_t pidTime = millis();
    // read multiple OBD-II PIDs
    byte results = obd.readPID(pids, sizeof(pids), values);
    pidTime = millis() - pidTime;
    int rowCounter = 0;
//    if (results == sizeof(pids)) {
      for (byte n = 0; n < sizeof(pids); n++) {
        byte pid = pids[n];
        if (obd.isValidPID(pid)) {
          int value= -1;
          obd.readPID(pid, value);
          showPIDData(n, pid, value, xPos, yPos);
        }else{
          showPIDData(n, pid, "invalid", xPos, yPos);
        }
        yPos = yPos+2;
        rowCounter = rowCounter + 1;
        if(rowCounter == 18){
          xPos = 245;
          rowCounter = 0;
          yPos = 4;
        }
      }
//    }
    if (logger.dataTime -  lastRefreshTime >= 1000) {
      float voltage = obd.getVoltage();
      lastRefreshTime = logger.dataTime;
    }

//    if (obd.errors >= 3) {
//        reconnect();
//    }
}
