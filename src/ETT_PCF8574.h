/*
 * File    : ETT_PCF8574.H
 * Author  : ETT CO.,LTD 
 *         : Modify From https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
 * Update  : October 2018        
 * Purpose : Library PCF8574/A For I2C Interface Arduino
 * Support : ESP32
 *         : ESP8266
 *         : MEGA32U4
 */
 
//=================================================================================
#ifndef ETT_PCF8574_H
#define ETT_PCF8574_H
//=================================================================================
#include "Arduino.h"
//=================================================================================
#define PCF8574_OK            0x00
#define PCF8574_PIN_ERROR     0x81
#define PCF8574_I2C_ERROR     0x82
//=================================================================================

//=================================================================================
#define PCF8574_ID_DEV0       (0x20)                                             // PCF8574  = 0100,000+(0:W,1:R)
#define PCF8574_ID_DEV1       (0x21)                                             // PCF8574  = 0100,001+(0:W,1:R)
#define PCF8574_ID_DEV2       (0x22)                                             // PCF8574  = 0100,010+(0:W,1:R)
#define PCF8574_ID_DEV3       (0x23)                                             // PCF8574  = 0100,011+(0:W,1:R)
#define PCF8574_ID_DEV4       (0x24)                                             // PCF8574  = 0100,100+(0:W,1:R)
#define PCF8574_ID_DEV5       (0x25)                                             // PCF8574  = 0100,101+(0:W,1:R)
#define PCF8574_ID_DEV6       (0x26)                                             // PCF8574  = 0100,110+(0:W,1:R)
#define PCF8574_ID_DEV7       (0x27)                                             // PCF8574  = 0100,111+(0:W,1:R)
//=================================================================================
#define PCF8574A_ID_DEV0      (0x38)                                             // PCF8574A = 0111,000+(0:W,1:R)
#define PCF8574A_ID_DEV1      (0x39)                                             // PCF8574A = 0111,001+(0:W,1:R)
#define PCF8574A_ID_DEV2      (0x3A)                                             // PCF8574A = 0111,010+(0:W,1:R)
#define PCF8574A_ID_DEV3      (0x3B)                                             // PCF8574A = 0111,011+(0:W,1:R)
#define PCF8574A_ID_DEV4      (0x3C)                                             // PCF8574A = 0111,100+(0:W,1:R)
#define PCF8574A_ID_DEV5      (0x3D)                                             // PCF8574A = 0111,101+(0:W,1:R)
#define PCF8574A_ID_DEV6      (0x3E)                                             // PCF8574A = 0111,110+(0:W,1:R)
#define PCF8574A_ID_DEV7      (0x3F)                                             // PCF8574A = 0111,111+(0:W,1:R)
//=================================================================================

//=================================================================================
// ET-I2C LONGLENGTH REPEATER
//=================================================================================
#define RELAY_OUT0_PIN 0
#define RELAY_OUT1_PIN 1
#define RELAY_OUT2_PIN 2
#define RELAY_OUT3_PIN 3
#define RELAY_OUT4_PIN 4
#define RELAY_OUT5_PIN 5
#define RELAY_OUT6_PIN 6
#define RELAY_OUT7_PIN 7
//=================================================================================
#define OPTO_IN0_PIN   0
#define OPTO_IN1_PIN   1
#define OPTO_IN2_PIN   2
#define OPTO_IN3_PIN   3
#define OPTO_IN4_PIN   4
#define OPTO_IN5_PIN   5
#define OPTO_IN6_PIN   6
#define OPTO_IN7_PIN   7
//=================================================================================
#define RY0_CLR_MASK_PIN      0xFE                                               // 1111 1110
#define RY1_CLR_MASK_PIN      0xFD                                               // 1111 1101
#define RY2_CLR_MASK_PIN      0xFB                                               // 1111 1011
#define RY3_CLR_MASK_PIN      0xF7                                               // 1111 0111
#define RY4_CLR_MASK_PIN      0xEF                                               // 1110 1111
#define RY5_CLR_MASK_PIN      0xDF                                               // 1101 1111
#define RY6_CLR_MASK_PIN      0xBF                                               // 1011 1111
#define RY7_CLR_MASK_PIN      0x7F                                               // 0111 1111
//=================================================================================
#define RY0_SET_MASK_PIN      0x01                                               // 0000 0001
#define RY1_SET_MASK_PIN      0x02                                               // 0000 0010
#define RY2_SET_MASK_PIN      0x04                                               // 0000 0100
#define RY3_SET_MASK_PIN      0x08                                               // 0000 1000
#define RY4_SET_MASK_PIN      0x10                                               // 0001 0000
#define RY5_SET_MASK_PIN      0x20                                               // 0010 0000
#define RY6_SET_MASK_PIN      0x40                                               // 0100 0000
#define RY7_SET_MASK_PIN      0x80                                               // 1000 0000
//=================================================================================
#define SLAVE_RELAY_OUT0_PIN  0
#define SLAVE_RELAY_OUT1_PIN  1
//=================================================================================
#define SLAVE_OPTO_IN0_PIN    4
#define SLAVE_OPTO_IN1_PIN    5
//=================================================================================
#define SLAVE_OPTO_IN0_MASK   0x10                                               // 0001 0000
#define SLAVE_OPTO_IN1_MASK   0x20                                               // 0010 0000
//=================================================================================
#define RELAY_ON              LOW
#define RELAY_OFF             HIGH
//=================================================================================
#define IN_PRESS              LOW
#define IN_RELEASE            HIGH
//=================================================================================

//=================================================================================
class ETT_PCF8574
{
public:
  explicit ETT_PCF8574(const uint8_t deviceAddress);          // I2C Assress
  
  void begin(uint8_t val = 0xFF);                             // Begin with Port Value State
  
  uint8_t readPin(uint8_t pin);                               // Read Pin(Not Write Pin = High Before Read)
  void writePin(const uint8_t pin, const uint8_t value);      // Write Port
  
  uint8_t readPort();                                         // Read Port(Not Write Port = Hight Before Read)
  void writePort(const uint8_t value);                        // Write Port 
 
  void togglePin(const uint8_t pin);                          // Toggle Pin
  void togglePinMask(const uint8_t mask);                     // Toggle Pin of Mask(1)

  uint8_t readButtonPin(const uint8_t pin);                   // Read Pin(Write Pin = High Before Read)
  uint8_t readButtonPort(const uint8_t mask=0xFF);            // Read Port(Write Port = High Before Read)
  
  uint8_t valueIn() const { return _dataIn; };                // Get Last Input State
  uint8_t valueOut() const { return _dataOut; }               // Get Last Output State
  int lastError();                                            // Get Last Error State
  
private:
  uint8_t _address;
  uint8_t _dataIn;
  uint8_t _dataOut; 
  int     _error;
};
//=================================================================================

//=================================================================================
#endif
//=================================================================================
// END OF FILE
//=================================================================================
