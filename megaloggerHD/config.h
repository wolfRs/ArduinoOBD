#ifndef CONFIG_H_INCLUDED
#define CONFIG_H_INCLUDED

/**************************************
* OBD-II Adapter options
**************************************/
//#define OBD_ADAPTER_I2C
#define OBD_ADAPTER_UART
//#define OBD_PROTOCOL PROTO_AUTO
#define OBD_PROTOCOL PROTO_CAN_29B_500K  

/**************************************
* cutom pids
**************************************/
#define ENGINE_OIL_TEMP 0x5C 
#define ENGINE_OIL_PRESSURE 0x0A //FUEL PRESSURE pid
#define MANIFOLD_PRESSURE 0x0B
#define REFERENCE_TORQUE    0x63
#define P_TURBO_PRESSURE    0x6F    
#define P_BOOST_PRES_CONT   0x70 
#define CVT_TEMP   1017 
#define FUEL_TEMP  0xA7  

/**************************************
* Data logging options
**************************************/
// enable(1)/disable(0) data logging (if SD card is present)
#define ENABLE_DATA_LOG 1
#define SD_CS_PIN SS

/**************************************
* Data streaming options
**************************************/
// enable(1)/disable(0) data streaming
#define ENABLE_DATA_OUT 1

// uses software(1)/hardware(0) serial for data streaming
#define RF_SERIAL Serial
#define STREAM_BAUDRATE 115200 /* bps */

// this defines the format of data streaming
// FORMAT_BIN is required by Freematics OBD iOS App
// FORMAT_TEXT for text-based, text names for PID
#define STREAM_FORMAT FORMAT_TEXT

/**************************************
* GPS configuration
**************************************/
#define USE_GPS 0
#define GPSUART Serial2
#define MAX_GPS_PROCESS_TIME 50 /* ms */
#define GPS_DATA_TIMEOUT 2000 /* ms */

// GPS baudrate could be 38400bps or 115200bps
#define GPS_BAUDRATE 115200 /* bps */

/**************************************
* Accelerometer & Gyro
**************************************/
#define ACC_DATA_RATIO 10
#define GYRO_DATA_RATIO 100

/**************************************
* LCD module (uncomment only one)
**************************************/
LCD_R61581 lcd; /* 3.5" CTE35IPS/R61581 based LCD */
//LCD_Null lcd;

#endif
