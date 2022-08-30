#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <PubSubClient.h>  
#include <Wire.h> 
#include "ETT_PCF8574.h"
#include <string>
//=================================================================================================
#define SerialDebug           Serial                                                              
//=================================================================================================
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define SerialRS485           Serial2                                                             
//=================================================================================================
#define I2C_SCL_PIN           22                                                                  
#define I2C_SDA_PIN           21   
//=================================================================================================
ETT_PCF8574 exp_i2c_io(PCF8574A_ID_DEV0); 
#define RelayOn               LOW                                                                 // On Relay Ative(LOW)
#define RelayOff              HIGH    
#define ry3_onboard_pin       6 
#define OnRelay()            exp_i2c_io.writePin(ry3_onboard_pin, RelayOn)                  // Open  = ON  Relay
#define OffRelay()           exp_i2c_io.writePin(ry3_onboard_pin, RelayOff)                 // Close = OFF Relay
#define ToggleRelay()        exp_i2c_io.writePin(ry3_onboard_pin, !exp_i2c_io.readPin(ry3_onboard_pin))   // Close = OFF Relay
#define ReadRelay()          !exp_i2c_io.readPin(ry3_onboard_pin) 
//=================================================================================================
#define modbusSensor_SlaveID      1                                                                    
//=================================================================================================
#define MSG_BUFFER_SIZE (500)
//=================================================================================================
const char* ssid = "Oatpt";
const char* password = "23456789";
//=================================================================================================
const char* mqtt_broker = "139.59.242.154";
const char* mqtt_username = "esl";
const char* mqtt_password = "esl603";
const int   mqtt_port = 1883;
//=================================================================================================                                                        
ModbusMaster nodeSensorWeather;                                                         
WiFiClient   espClient;
PubSubClient client(espClient);
//=================================================================================================
uint8_t read_modbus_status;
//=================================================================================================
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
int value = 0;
float temperature;
float humidity;
unsigned long lastGetModbusTime = 0;
char temperature_text[10];   
char humidity_text[10];                                                                         
//=================================================================================================
void setup_wifi() {
  delay(10);
  SerialDebug.println();
  SerialDebug.print("Connecting to ");
  SerialDebug.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    SerialDebug.println("Connecting to WiFi...");
  }
  randomSeed(micros());
  SerialDebug.println("WiFi connected");
  SerialDebug.println("IP address: ");
  SerialDebug.println(WiFi.localIP());
}
//=================================================================================================
String text;
void callback(char *topic, byte *payload, unsigned int length) {
  for (int i = 0; i < length; i++) 
     text+=(char) payload[i];
  //SerialDebug.println(text);
  //SerialDebug.println(topic);
  //SerialDebug.println(strcmp(topic,"controlhumi"));
  //SerialDebug.println(text.compareTo("true"));
  //SerialDebug.println("+++++++++++++++++++++++++++++++++");
  if(strcmp(topic,"controlhumi")==0)
  {

    if(text.compareTo("true")==0)
      OnRelay();
    else
      OffRelay();
  }
  text="";

}
//=================================================================================================
void reconnect() {
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  while (!client.connected()) 
  {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) 
      Serial.println("Public emqx mqtt broker connected");
    else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}
//=================================================================================================
void setup() 
{
  Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);                                                      
  exp_i2c_io.begin(0xFF);
  OffRelay();
  //===============================================================================================
  Serial.begin(9600);
  SerialDebug.begin(9600);
  while(!SerialDebug);
  //===============================================================================================
  setup_wifi();
  reconnect();
  //===============================================================================================
  SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while(!SerialRS485);
  //===============================================================================================
  SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Start");
  //===============================================================================================
  nodeSensorWeather.begin(modbusSensor_SlaveID, SerialRS485);                                     
  //===============================================================================================
  SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Ready");
  SerialDebug.println();
  //===============================================================================================
  client.subscribe("controltemp");
  client.subscribe("controlhumi");
  //===============================================================================================
}

void loop() 
{
  //===============================================================================================
  client.loop();
  if((millis() - lastGetModbusTime) > 5000)                                                    
  {
    read_modbus_status = nodeSensorWeather.readInputRegisters(0x0001, 2);                       
    //=============================================================================================
    if(read_modbus_status == nodeSensorWeather.ku8MBSuccess)
    {
      //===========================================================================================
      temperature = (float)nodeSensorWeather.getResponseBuffer(0)/10;                   
      humidity = (float)nodeSensorWeather.getResponseBuffer(1)/10;                                             
      //===========================================================================================
      SerialDebug.print("Temperature = ");
      SerialDebug.print(temperature,1);
      SerialDebug.print(" C");
      SerialDebug.print(" : Humidity = ");
      SerialDebug.print(humidity,1);
      SerialDebug.print(" %");
      SerialDebug.println();
      //===========================================================================================
      sprintf(humidity_text, "%g", humidity);
      sprintf(temperature_text, "%g", temperature);
      
      client.publish("temp", temperature_text);
      client.publish("humi", humidity_text);
      //===========================================================================================
    }
    lastGetModbusTime = millis();
    //=============================================================================================
  }
  //===============================================================================================
}


