#include <HardwareSerial.h>
#include <ModbusMaster.h>
#include <WiFi.h>
#include <Wire.h>
#include <PubSubClient.h>
#include "ETT_PCF8574.h"
#include <string>
//=================================================================================================
#define SerialDebug Serial
//=================================================================================================
#define SerialRS485_RX_PIN 26
#define SerialRS485_TX_PIN 27
#define SerialRS485 Serial2
//=================================================================================================
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21
//=================================================================================================
ETT_PCF8574 exp_i2c_io(PCF8574A_ID_DEV0);
#define RelayOn LOW // On Relay Ative(LOW)
#define RelayOff HIGH
#define ry3_onboard_pin 6 //relay 3
#define ry2_onboard_pin 4 //relay 2
#define ry1_onboard_pin 2 //relay 1
int relay_onboard_pin[4] = {0, ry1_onboard_pin, ry2_onboard_pin, ry3_onboard_pin};
#define OnRelay(r) exp_i2c_io.writePin(relay_onboard_pin[r], RelayOn)                                  // Open  = ON  Relay
#define OffRelay(r) exp_i2c_io.writePin(relay_onboard_pin[r], RelayOff)                                // Close = OFF Relay
#define ToggleRelay(r) exp_i2c_io.writePin(relay_onboard_pin[r], !exp_i2c_io.readPin(ry3_onboard_pin)) // Close = OFF Relay
#define ReadRelay(r) !exp_i2c_io.readPin(relay_onboard_pin[r])
//=================================================================================================
#define modbusSensor_SlaveID 1
//=================================================================================================
#define MSG_BUFFER_SIZE (500)
//=================================================================================================
#define LedPin                2                                                                   // ESP32-WROVER  : IO2
#define LedLogicOn            HIGH
#define LedLogicOff           LOW
#define InitialLed()          pinMode(LedPin,OUTPUT)
//=================================================================================================

#define TIME_OUT                10   

//=================================================================================================
const char *ssid = "CE-ESL";
const char *password = "ceeslonly";
const char *mqtt_broker = "192.168.1.156";
const char *mqtt_username = "esl";
const char *mqtt_password = "esl603";
const int mqtt_port = 1883;
//=================================================================================================
ModbusMaster nodeSensorWeather;
WiFiClient espClient;
PubSubClient client(espClient);
uint8_t read_modbus_status;
String text;
//=================================================================================================
unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
int value = 0;
float temperature;
float humidity;
unsigned long lastGetModbusTime = 0;
char temperature_text[10];
char humidity_text[10];
int settemp;
int sethumi;
int tempInteger;
int error;
//=================================================================================================
void setup_wifi()
{
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
  SerialDebug.println("WiFi connected");
  SerialDebug.println("IP address: ");
  SerialDebug.println(WiFi.localIP());
}
//=================================================================================================
void callback(char *topic, byte *payload, unsigned int length)
{
  SerialDebug.println(topic);
  if (strcmp(topic, "els/mushroom1/controlhumi") == 0)
  {
    tempInteger = 0;
    for (int i = 0; i < length; i++)
    {
      tempInteger *= 10;
      tempInteger += (char)payload[i] - 48;
    }
    sethumi=tempInteger;
  }
  else if (strcmp(topic, "els/mushroom1/controltemp") == 0)
  {
    tempInteger = 0;
    for (int i = 0; i < length; i++)
    {
      tempInteger *= 10;
      tempInteger += (char)payload[i] - 48;
    }
    settemp=tempInteger;
  }
  else if (strcmp(topic, "els/mushroom1/controllight") == 0)
  {
    text = "";
    for (int i = 0; i < length; i++)
      text += (char)payload[i];
    if (text.compareTo("true") == 0)
      OnRelay(1);
    else
      OffRelay(1);
  }
}
//=================================================================================================
void reconnect()
{
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  uint8_t count = 0;
  while (!client.connected() && count < TIME_OUT)
  {
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    SerialDebug.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      SerialDebug.println("Public emqx mqtt broker connected");
      client.subscribe("els/mushroom1/controltemp");
      client.subscribe("els/mushroom1/controlhumi");
      client.subscribe("els/mushroom1/controllight");
      client.publish("els/mushroom1/status", "ready");
    }
    else
    {
      SerialDebug.print("failed with state ");
      SerialDebug.print(client.state());
      delay(500);
    }
  }
}
//=================================================================================================
void control_humi()
{
  char humiControl_text[10];
  sprintf(humiControl_text, "%d", sethumi);
  client.publish("els/mushroom1/humi/control", humiControl_text);
  if ((int)humidity <= sethumi)
    {
        OnRelay(3);
        client.publish("els/mushroom1/humi/status", "true");
    }
    else if ((int)humidity >= sethumi+5)
    {
        OffRelay(3);
        client.publish("els/mushroom1/humi/status", "false");
    }
}

void control_temp()
{
  char tempControl_text[10];
  sprintf(tempControl_text, "%d", settemp);
  client.publish("els/mushroom1/temp/control", tempControl_text);
  if ((int)temperature >= settemp )
    {
        OnRelay(2);
        client.publish("els/mushroom1/temp/status", "true");
    }
    else if ((int)temperature <= settemp-2 )
    {
        OffRelay(2);
        client.publish("els/mushroom1/temp/status", "false");
    }
}

void check_status_mqtt()
{
  if(client.state() != 0)
    {
      error++;
      reconnect();
    }
}

void read_temp_humi()
{
  read_modbus_status = nodeSensorWeather.readInputRegisters(0x0001, 2);
    //=============================================================================================
    if (read_modbus_status == nodeSensorWeather.ku8MBSuccess)
    {
      digitalWrite(LedPin, !digitalRead(LedPin));
      //===========================================================================================
      temperature = (float)nodeSensorWeather.getResponseBuffer(0) / 10;
      humidity = (float)nodeSensorWeather.getResponseBuffer(1) / 10;
      //===========================================================================================
      SerialDebug.print("Temperature = ");
      SerialDebug.print(temperature, 1);
      SerialDebug.print(" C");
      SerialDebug.print(" : Humidity = ");
      SerialDebug.print(humidity, 1);
      SerialDebug.print(" %");
      SerialDebug.println();
      //===========================================================================================
      sprintf(humidity_text, "%g", humidity);
      sprintf(temperature_text, "%g", temperature);
      client.publish("els/mushroom1/temp", temperature_text);
      client.publish("els/mushroom1/humi", humidity_text);
    }
    lastGetModbusTime = millis();
}

void setup()
{
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  exp_i2c_io.begin(0xFF);
  InitialLed();
  for (int i = 1; i < 4; i++)
    OffRelay(i);
  //===============================================================================================
  Serial.begin(9600);
  SerialDebug.begin(9600);
  while (!SerialDebug);
  //===============================================================================================
  setup_wifi();
  reconnect();
  //===============================================================================================
  SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while (!SerialRS485);
  //===============================================================================================
  SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Start");
  //===============================================================================================
  nodeSensorWeather.begin(modbusSensor_SlaveID, SerialRS485);
  //===============================================================================================
  SerialDebug.println("ET-ESP32(WROVER)RS485 V3.....Ready");
  SerialDebug.println();
  //===============================================================================================
  client.publish("els/mushroom1/status", "ready");
  //===============================================================================================
}

void loop()
{
  //===============================================================================================
  client.loop();
  if ((millis() - lastGetModbusTime) > 5000)
  {
    read_temp_humi();
    SerialDebug.print(F("mqtt state -> "));
    SerialDebug.println(client.state());
    SerialDebug.print("count mqtt error -> ");
    check_status_mqtt();
    SerialDebug.println(error);
    control_temp();
    control_humi();
    //=============================================================================================
  }
  //===============================================================================================
}

