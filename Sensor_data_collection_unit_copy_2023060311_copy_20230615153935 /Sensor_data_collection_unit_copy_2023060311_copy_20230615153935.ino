/*AHT15:Digital Humidity & Temperature Sensor*/
/***************************************************************************************************/
/*
   This is an Arduino example for Aosong ASAIR AHT10/AHT15/AHT20/AHT21/AHT25/AM2301B/AM2311B
   Digital Humidity & Temperature Sensor

   written by : enjoyneering
   sourse code: https://github.com/enjoyneering/

   Aosong ASAIR AHT1x/AHT2x features:
   - AHT1x +1.8v..+3.6v, AHT2x +2.2v..+5.5v
   - AHT1x 0.25uA..320uA, AHT2x 0.25uA..980uA
   - temperature range -40C..+85C
   - humidity range 0%..100%
   - typical accuracy T +-0.3C, RH +-2%
   - typical resolution T 0.01C, RH 0.024%
   - normal operating range T -20C..+60C, RH 10%..80%
   - maximum operating rage T -40C..+80C, RH 0%..100%
   - response time 8..30sec*
   - I2C bus speed 100KHz..400KHz, 10KHz recommended minimum
     *measurement with high frequency leads to heating
      of the sensor, must be > 2 seconds apart to keep
      self-heating below 0.1C

   This device uses I2C bus to communicate, specials pins are required to interface
   Board:                                    SDA              SCL              Level
   Uno, Mini, Pro, ATmega168, ATmega328..... A4               A5               5v
   Mega2560................................. 20               21               5v
   Due, SAM3X8E............................. 20               21               3.3v
   Leonardo, Micro, ATmega32U4.............. 2                3                5v
   Digistump, Trinket, ATtiny85............. PB0              PB2              5v
   Blue Pill, STM32F103xxxx boards.......... PB9/PB7*         PB8/PB6*         3.3v/5v
   ESP8266 ESP-01........................... GPIO0**          GPIO2**          3.3v/5v
   NodeMCU 1.0, WeMos D1 Mini............... GPIO4/D2         GPIO5/D1         3.3v/5v
   ESP32.................................... GPIO21/D21       GPIO22/D22       3.3v
                                             *hardware I2C Wire mapped to Wire1 in stm32duino
                                              see https://github.com/stm32duino/wiki/wiki/API#i2c
                                            **most boards has 10K..12K pullup-up resistor
                                              on GPIO0/D3, GPIO2/D4/LED & pullup-down on
                                              GPIO15/D8 for flash & boot

   Frameworks & Libraries:
   ATtiny  Core - https://github.com/SpenceKonde/ATTinyCore
   ESP32   Core - https://github.com/espressif/arduino-esp32
   ESP8266 Core - https://github.com/esp8266/Arduino
   STM32   Core - https://github.com/stm32duino/Arduino_Core_STM32


   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/
/*SEN0482:wind direction sensor*/
/**
   @brief RS485_Wind_Direction_Transmitter_V2 constructor
   @param serial - serial ports for communication, supporting hard and soft serial ports
   @param rx - UART 接收数据的引脚
   @param tx - UART 发送数据的引脚
*/
/*------------------------------------------------------------------------------------------------------------------------------*/
/*50A Current Sensor(AC/DC)(SKU:SEN0098)*/
    /*
    50A Current Sensor(AC/DC)(SKU:SEN0098) Sample Code
    This code shows you how to get raw datas from the sensor through Arduino and convert the
    raw datas to the value of the current according to the datasheet;

    Smoothing algorithm (http://www.arduino.cc/en/Tutorial/Smoothing) is used
    to make the outputting current value more reliable;

    Created 27 December 2011
    By Barry Machine
    www.dfrobot.com
    Version:0.2
    */
/*------------------------------------------------------------------------------------------------------------------------------*/
/*35A WCS1800 current sensor*/
 /*
 * Read the current Winson WCS Current sensor and display it on Serial monitor
 * This is Arduino code based on Robojax WCS library 
 * for Winson WSC Current Sensor to measure current
 
 get the Robojax WCS Arduino library: http://robojax.com/L/?id=230
 
 * Watch video instructions for this code: https://youtu.be/z-s8UvCWGxY

 * 
 * Related Vidsos:
 * Introduction to Winson WCS Sensors: https://youtu.be/z-s8UvCWGxY
 * Display current on LCD: https://youtu.be/-pg7jbkaB6A
 * Overcurrent protection with WCS Current sensors: https://youtu.be/vs7Nw107AOo
 * Measure Current using ESP8266 NodeMCU, D1 Mino over Wifi:
 * Measure current using ESP32 Bluetooth on mobile devices:

 * Written on July 26, 2020 by Ahmad Shamshiri in Ajax, Ontario, Canada
 * www. Robojax.com
  Model of the sensor to select 
     //dirct wiring series
    0  "WCS38A25",//0
    1  "WCS37A50",//1
    2  "WCS2801",//2
    3  "WCS2702",//3
    4  "WCS2705",//4
    5  "WCS2810",//5
    6  "WCS2720",//6
    7  "WCS2750",//7
    8  "WCS3740",//8
      
      //through hole sensor
    9  "WCS1500",//9
    10  "WCS1600",//10
    11  "WCS1700",//11
    12  "WCS1800",//12
    13  "WCS2800",//13
    14  "WCS6800",//14
      
      //AC to DC Current sensor
    15  "WCS2202",//15


 * Get this code and other Arduino codes from Robojax.com
Learn Arduino step by step in structured course with all material, wiring diagram and library
all in once place. Purchase My course on Udemy.com http://robojax.com/L/?id=62

If you found this tutorial helpful, please support me so I can continue creating 
content like this. You can support me on Patreon http://robojax.com/L/?id=63

or make donation using PayPal http://robojax.com/L/?id=64

 *  * This code is "AS IS" without warranty or liability. Free to be used as long as you keep this note intact.* 
 * This code has been download from Robojax.com
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/   
/*==============================================================================================================================*/
/*AHT15:Digital Humidity & Temperature Sensor*/
#include <Wire.h>
#include <AHTxx.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

double ahtValueT;                               //to store T/RH result
double ahtValueH;                               //to store T/RH result

AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type
/*------------------------------------------------------------------------------------------------------------------------------*/
/*SEN0483:wind speed sensor*/
double Level = 0.0;
/*------------------------------------------------------------------------------------------------------------------------------*/
/*SEN0482:wind direction sensor*/
#include "RS485_Wind_Direction_Transmitter_V2.h"

#if defined(ARDUINO_AVR_UNO)||defined(ESP8266)   // 使用软串口
SoftwareSerial softSerial(/*rx =*/2, /*tx =*/3);
RS485_Wind_Direction_Transmitter_V2 windDirection(/*softSerial =*/&softSerial);
#elif defined(ESP32)   // 使用 可重映射引脚的 硬串口 : Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1, /*rx =*/32, /*tx =*/33);
#else   // 使用硬串口 : Serial1
RS485_Wind_Direction_Transmitter_V2 windDirection(/*hardSerial =*/&Serial1);
#endif

uint8_t Address = 0x02;
double Angle = 0.0;
const char* Orientation[17] = {
  "North", "Northeast by north", "Northeast", "Northeast by east", "East", "Southeast by east", "Southeast", "Southeast by south", "South",
  "Southwest by south", "Southwest", "Southwest by west", "West", "Northwest by west", "Northwest", "Northwest by north", "North"
};
/*------------------------------------------------------------------------------------------------------------------------------*/
/*50A Current Sensor(AC/DC)(SKU:SEN0098)*/
    const int numReadings = 30;
    float readings[numReadings];      // the readings from the analog input
    int _index = 0;                  // the index of the current reading
    float total = 0;                  // the running total
    float average = 0;                // the average
    float currentValue = 0;
/*------------------------------------------------------------------------------------------------------------------------------*/
/*35A WCS1800 current sensor*/
// #include <Wire.h> 
// #include <Robojax_WCS.h>
// #define MODEL 12 //see list above
// #define SENSOR_PIN A0 //pin for reading sensor
// #define SENSOR_VCC_PIN 8 //pin for powring up the sensor
// #define ZERO_CURRENT_LED_PIN 2 //zero current LED pin

// #define ZERO_CURRENT_WAIT_TIME 5000 //wait for 5 seconds to allow zero current measurement
// #define CORRECTION_VLALUE 164 //mA
// #define MEASUREMENT_ITERATION 100
// #define VOLTAGE_REFERENCE  5000.0 //5000mv is for 5V
// #define BIT_RESOLUTION 10
// #define DEBUT_ONCE true

// Robojax_WCS sensor(
//           MODEL, SENSOR_PIN, SENSOR_VCC_PIN, 
//           ZERO_CURRENT_WAIT_TIME, ZERO_CURRENT_LED_PIN,
//           CORRECTION_VLALUE, MEASUREMENT_ITERATION, VOLTAGE_REFERENCE,
//           BIT_RESOLUTION, DEBUT_ONCE           
//           );   
double current; 
/*------------------------------------------------------------------------------------------------------------------------------*/
/*35V voltage*/
double vol; 
/*------------------------------------------------------------------------------------------------------------------------------*/
/*MAX6675 K type temperature*/
#include "max6675.h"

int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;
double thermoData;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
/*------------------------------------------------------------------------------------------------------------------------------*/
/*MQTT*/
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
 
const char* ssid = "WindRock_V504_2.4G";    //TP-Link_DennisChen
const char* password =  "0931080715";         //a000423B
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
// 設定用戶端ID
const char clientID[] = "yard001";


// const int capacity = JSON_OBJECT_SIZE(7);   //7: 變數數量
StaticJsonDocument<500> doc;
char payload[500];  

WiFiClient espClient;
PubSubClient client(espClient);
 
void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
 
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
 
  Serial.println();
  Serial.println("-----------------------");
 
}

void EncodingJson(void) {
  // Add values in the document
  doc["environment_temperature"] = ahtValueT+=0.00000001;
  doc["environment_humidity"] = ahtValueH+=0.00000001;
  doc["wind_speed"] = Level+=0.00000001;  //+=0.00000001 avoid magoDB issue
  doc["wind_direction"] = Angle+=0.00000001;
  doc["DCvoltage"] = vol+=0.00000001;
  doc["DCcurrent"] = current+=0.00000001;
  doc["Ktemperature"] = thermoData+=0.00000001;

  serializeJson(doc, payload);
  // Generate the minified JSON and send it to the Serial port.
  serializeJson(doc, Serial);
}

void reconnect() {
  // 若目前沒有和伺服器相連，則反覆執行直到連結成功…
  while (!client.connected()) {
    // 指定用戶端ID並連結MQTT伺服器
    if (client.connect(clientID)) {
      // 若連結成功，在序列埠監控視窗顯示「已連線」。
      Serial.println("connected");
    } else {
      // 若連線不成功，則顯示錯誤訊息
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // 等候5秒，再重新嘗試連線。
      delay(5000);
    }
  }
}

/*=============================================================================================================================*/
/*ESP32 Timer Interrupt*/
hw_timer_t *Timer0_Cfg = NULL;
 
void IRAM_ATTR Timer0_ISR()
{
  // client.loop();

}
/**************************************************************************/
/*
    setup()

    Main setup
*/
/**************************************************************************/
void setup()
{
/*AHT15:Digital Humidity & Temperature Sensor*/
  #if defined(ESP8266)
  WiFi.persistent(false);  //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();  //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep
  #endif

  Serial.begin(115200);
  Serial.println();
  
  while (aht10.begin() != true) //for ESP-01 use aht10.begin(0, 2);
  {
    Serial.println(F("AHT1x not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free

    delay(1000);
  }

  Serial.println(F("AHT10 OK"));

  //Wire.setClock(400000); //experimental I2C speed! 400KHz, default 100KHz
/*------------------------------------------------------------------------------------------------------------------------------*/
/*50A Current Sensor(AC/DC)(SKU:SEN0098)*/
  // for (int thisReading = 0; thisReading < numReadings; thisReading++)
  //   readings[thisReading] = 0;
/*------------------------------------------------------------------------------------------------------------------------------*/
/*SEN0482:wind direction sensor*/
  // Init the sensor
  while ( !( windDirection.begin() ) ) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");
//修改modbus从机的地址，若忘记现在的地址可以使用0x00广播地址修改。
//  windDirection.SetSlaveAddress(/*现在modbus从机地址*/0x00, /*修改后的地址*/0x02);
//获取modbus从机的地址。
//  Address = windDirection.GetSlaveAddress();
/*------------------------------------------------------------------------------------------------------------------------------*/
/*35A WCS1800 current sensor*/
  // Serial.println("Robojax WCS Library");
  // sensor.start();
  // Serial.print("Sensor: "); Serial.println(sensor.getModel());
  // Serial.print("Library Version:");Serial.println(sensor.version());
  //sensor.printModels();//prints all supported WCS models

/*------------------------------------------------------------------------------------------------------------------------------*/
/*MQTT*/
  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
 
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
 
      Serial.println("connected");  
 
    } else {
 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
 
    }
  }
 
  // client.subscribe("mongoDB/sensors");
/*------------------------------------------------------------------------------------------------------------------------------*/
/*ESP32 Timer Interrupt*/
    // Timer0_Cfg = timerBegin(0, 80, true);
    // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    // timerAlarmWrite(Timer0_Cfg, 10000000, true);  //100mS
    // timerAlarmEnable(Timer0_Cfg);


}


/**************************************************************************/
/*
    loop()

     Main loop
*/
/**************************************************************************/
void loop()
{
/*MQTT*/  
  // 確認用戶端是否已連上伺服器
  if (!client.connected()) {
    // 若沒有連上，則執行此自訂函式。
    reconnect();
  }
  // 更新用戶端狀態
  client.loop();
/*------------------------------------------------------------------------------------------------------------------------------*/
  /*AHT15:Digital Humidity & Temperature Sensor*/
  /* DEMO - 1, every temperature or humidity call will read 6-bytes over I2C, total 12-bytes */
  Serial.println();
  Serial.println(F("DEMO 1: read 12-bytes"));

  ahtValueT = aht10.readTemperature(); //read 6-bytes via I2C, takes 80 milliseconds

  Serial.print(F("Temperature...: "));
  
  if (ahtValueT != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
  {
    Serial.print(ahtValueT);
    Serial.println(F(" +-0.3C"));
  }
  else
  {
    printStatus(); //print temperature command status

    if   (aht10.softReset() == true) Serial.println(F("reset success")); //as the last chance to make it alive
    else                             Serial.println(F("reset failed"));
  }

  // delay(2000); //measurement with high frequency leads to heating of the sensor, see NOTE

  ahtValueH = aht10.readHumidity(); //read another 6-bytes via I2C, takes 80 milliseconds

  Serial.print(F("Humidity......: "));
  
  if (ahtValueH != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
  {
    Serial.print(ahtValueH);
    Serial.println(F(" +-2%"));
  }
  else
  {
    printStatus(); //print humidity command status
  }

  // delay(2000); //measurement with high frequency leads to heating of the sensor, see NOTE

  /* DEMO - 2, temperature call will read 6-bytes via I2C, humidity will use same 6-bytes */
  Serial.println();
  Serial.println(F("DEMO 2: read 6-byte"));

  ahtValueT = aht10.readTemperature(); //read 6-bytes via I2C, takes 80 milliseconds

  Serial.print(F("Temperature: "));
  
  if (ahtValueT != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
  {
    Serial.print(ahtValueT);
    Serial.println(F(" +-0.3C"));
  }
  else
  {
    printStatus(); //print temperature command status
  }

  ahtValueH = aht10.readHumidity(AHTXX_USE_READ_DATA); //use 6-bytes from temperature reading, takes zero milliseconds!!!

  Serial.print(F("Humidity...: "));
  
  if (ahtValueH != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
  {
    Serial.print(ahtValueH);
    Serial.println(F(" +-2%"));
  }
  else
  {
    printStatus(); //print temperature command status not humidity!!! RH measurement use same 6-bytes from T measurement
  }
/*------------------------------------------------------------------------------------------------------------------------------*/
/*SEN0483:wind speed sensor*/
  int sensorValue = analogRead(34);
  float outvoltage = sensorValue * (3.3 / 4095.0);

  Serial.println("");
  Serial.print("outvoltage = ");
  Serial.print(outvoltage);
  Serial.println("V");
  Level = 6*(outvoltage-0.0);//风速等级和输出电压值呈线性关系; -x:adj
  Serial.print("wind speed is ");
  Serial.print(Level);
  Serial.print(" m/s");
  Serial.println(" level now");
  Serial.println();
/*------------------------------------------------------------------------------------------------------------------------------*/
/*SEN0482:wind direction sensor*/
  //获取16方位风向
  int Direction = windDirection.GetWindDirection(/*modbus从机的地址*/Address);
  //获取360°风向
  Angle = windDirection.GetWindAngle(/*modbus从机的地址*/Address);
   Serial.println("wind direction:");
  Serial.println(Orientation[Direction]);
  Serial.print(Angle); Serial.println("°");
  Serial.println();
/*------------------------------------------------------------------------------------------------------------------------------*/
/*50A Current Sensor(AC/DC)(SKU:SEN0098)*/
  // total= total - readings[_index];
  // readings[_index] = analogRead(35); //Raw data reading

  // //Data processing:510-raw data from analogRead when the input is 0; 5-5v;
  // //the first 0.04-0.04V/A(sensitivity); the second 0.04-offset val;

  // readings[_index] = (readings[_index]-510)*3.3/4095/0.04-27.990;   //+0.26 -> 
  // total= total + readings[_index];
  // _index = _index + 1;
  // if (_index >= numReadings)
  //   _index = 0;
  // average = total/numReadings;
  // currentValue= average;
  // Serial.println("DC Current(A):");
  // Serial.println(currentValue);
/*------------------------------------------------------------------------------------------------------------------------------*/
/*35A WCS1800 current sensor*/
  // sensor.readCurrent();//this must be inside loop
  // sensor.printCurrent();
  double adc = analogRead(35);
  double voltage = adc*3.3/4095.0*5/3.3;   //*5/3.3 for 3.3 to 5V voltage level
  current = (voltage-2.32)/0.06;   //2.32 adj
  Serial.print("DC Current : ");
  Serial.println(current);
/*------------------------------------------------------------------------------------------------------------------------------*/
/*35V voltage*/
  double adc1 = analogRead(39);
  double voltage1 = adc1*3.3/4095.0*35/3.3;    //*21000/2000 for 3.3 to 35V voltage level
  vol = voltage1+0.8;   //0.0 adj
  Serial.print("DC Voltage : ");
  Serial.println(vol);
/*------------------------------------------------------------------------------------------------------------------------------*/ 
/*MAX6675 K type temperature*/
  Serial.println("temperature of K type thermo"); 
  Serial.print("C = "); 
  Serial.println(thermocouple.readCelsius());
  thermoData = thermocouple.readCelsius();
  Serial.print("F = ");
  Serial.println(thermocouple.readFahrenheit());
  Serial.println(""); 
/*------------------------------------------------------------------------------------------------------------------------------*/
/*MQTT*/  
  EncodingJson();
  client.publish("mongoDB/sensors", payload);  
/*------------------------------------------------------------------------------------------------------------------------------*/  
  delay(10000); //setting sensor's data send delay time 
}


/**************************************************************************/
/*
    printStatus()

    Print last command status
*/
/**************************************************************************/
/*AHT15:Digital Humidity & Temperature Sensor*/
void printStatus()
{
  switch (aht10.getStatus())
  {
    case AHTXX_NO_ERROR:
      Serial.println(F("no error"));
      break;

    case AHTXX_BUSY_ERROR:
      Serial.println(F("sensor busy, increase polling time"));
      break;

    case AHTXX_ACK_ERROR:
      Serial.println(F("sensor didn't return ACK, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"));
      break;

    case AHTXX_DATA_ERROR:
      Serial.println(F("received data smaller than expected, not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit)"));
      break;

    case AHTXX_CRC8_ERROR:
      Serial.println(F("computed CRC8 not match received CRC8, this feature supported only by AHT2x sensors"));
      break;

    default:
      Serial.println(F("unknown status"));    
      break;
  }
}