/***************************************************************************
* Example sketch for the INA219_WE library
*
* This sketch shows how to use the INA219 module in continuous mode. 
*  
* Further information can be found on:
* https://wolles-elektronikkiste.de/ina219 (German)
* https://wolles-elektronikkiste.de/en/ina219-current-and-power-sensor (English)
* 
***************************************************************************/

#include <INA219_WE.h>
#define I2C_ADDRESS 0x40
#include <Wire.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
float shuntVoltage_mV = 0.0;
  float loadVoltage_V = 0.0;
  float busVoltage_V = 0.0;
  float current_mA = 0.0;
  float power_mW = 0.0; 
  bool ina219_overflow = false;
/* There are several ways to create your INA219 object:
 * INA219_WE ina219 = INA219_WE(); -> uses Wire / I2C Address = 0x40
 * INA219_WE ina219 = INA219_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
 * INA219_WE ina219 = INA219_WE(&Wire); -> you can pass any TwoWire object
 * INA219_WE ina219 = INA219_WE(&Wire, I2C_ADDRESS); -> all together
 */
 const char* ssid = "Yun";    //TP-Link_DennisChen
const char* password =  "didi901225";         //a000423B
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
// 設定用戶端ID
const char clientID[] = "yard001";
INA219_WE ina219 = INA219_WE(I2C_ADDRESS);

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
 
  doc["DCvoltage"] = busVoltage_V+=0.00000001;
  doc["DCcurrent"] = current_mA+=0.00000001;
  doc["Power"] = power_mW+=0.00000001;
  

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
void setup() {
   #if defined(ESP8266)
  WiFi.persistent(false);  //disable saving wifi config into SDK flash area
  WiFi.forceSleepBegin();  //disable AP & station by calling "WiFi.mode(WIFI_OFF)" & put modem to sleep
  #endif
  Serial.begin(115200);
  Wire.begin();
  if(!ina219.init()){
    Serial.println("INA219 not connected!");
  }
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
  /* Set ADC Mode for Bus and ShuntVoltage
  * Mode *            * Res / Samples *       * Conversion Time *
  BIT_MODE_9        9 Bit Resolution             84 µs
  BIT_MODE_10       10 Bit Resolution            148 µs  
  BIT_MODE_11       11 Bit Resolution            276 µs
  BIT_MODE_12       12 Bit Resolution            532 µs  (DEFAULT)
  SAMPLE_MODE_2     Mean Value 2 samples         1.06 ms
  SAMPLE_MODE_4     Mean Value 4 samples         2.13 ms
  SAMPLE_MODE_8     Mean Value 8 samples         4.26 ms
  SAMPLE_MODE_16    Mean Value 16 samples        8.51 ms     
  SAMPLE_MODE_32    Mean Value 32 samples        17.02 ms
  SAMPLE_MODE_64    Mean Value 64 samples        34.05 ms
  SAMPLE_MODE_128   Mean Value 128 samples       68.10 ms
  */
  //ina219.setADCMode(SAMPLE_MODE_128); // choose mode and uncomment for change of default
  
  /* Set measure mode
  POWER_DOWN - INA219 switched off
  TRIGGERED  - measurement on demand
  ADC_OFF    - Analog/Digital Converter switched off
  CONTINUOUS  - Continuous measurements (DEFAULT)
  */
  // ina219.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default
  
  /* Set PGain
  * Gain *  * Shunt Voltage Range *   * Max Current (if shunt is 0.1 ohms) *
   PG_40       40 mV                    0.4 A
   PG_80       80 mV                    0.8 A
   PG_160      160 mV                   1.6 A
   PG_320      320 mV                   3.2 A (DEFAULT)
  */
  //ina219.setPGain(PG_320); // choose gain and uncomment for change of default
  
  /* Set Bus Voltage Range
   BRNG_16   -> 16 V
   BRNG_32   -> 32 V (DEFAULT)
  */
  ina219.setBusRange(BRNG_32); // choose range and uncomment for change of default

  Serial.println("INA219 Current Sensor Example Sketch - Continuous");

  /* If the current values delivered by the INA219 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA219
  */
   ina219.setCorrectionFactor(0.0); // insert your correction factor if necessary
  
  /* If you experience a shunt voltage offset, that means you detect a shunt voltage which is not 
     zero, although the current should be zero, you can apply a correction. For this, uncomment the 
     following function and apply the offset you have detected.   
  */
  ina219.setShuntVoltOffset_mV(-0.03); // insert the shunt voltage (millivolts) you detect at zero current 
}
//dXJuOmFkc2sub2JqZWN0czpvcy5vYmplY3Q6bXNlZnlxb3B4aGNiNWp3ZG9waGRqdWZxYWNwcWFmOG8tYmFzaWMtYXBwLyVFNyVCNSU4NCVFNSU5MCU4OCVFNCVCQiVCNjMuU1RFUA
void loop() {
  
   if (!client.connected()) {
    // 若沒有連上，則執行此自訂函式。
    reconnect();
  }
  // 更新用戶端狀態
  client.loop();
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = abs(ina219.getCurrent_mA());
  power_mW = abs(ina219.getBusPower());
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
 
  Serial.print("Shunt Voltage [mV]: "); Serial.println(shuntVoltage_mV);
  Serial.print("Bus Voltage [V]: "); Serial.println(busVoltage_V);
  Serial.print("Load Voltage [V]: "); Serial.println(loadVoltage_V);
  Serial.print("Current[mA]: "); Serial.println(current_mA);
  Serial.print("Bus Power [mW]: "); Serial.println(power_mW);
  if(!ina219_overflow){
    Serial.println("Values OK - no overflow");
  }
  else{
    Serial.println("Overflow! Choose higher PGAIN");
  }
  Serial.println();
  EncodingJson();
  client.publish("mongoDB/sensors", payload);  
  delay(2000);
}
