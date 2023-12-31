/**
   1.1 - Corrigido LIGAR/DESLIGAR ESP se perdendo (logica invertida)

   ID:
   0x128 - Dados do cambio (SPORT/NEVE)
   0x217 - ID para ativar ESP / PARK / PONTO CEGO... (ver doc git)
   0x2E9 -

*/

#include <can.h>     // Include CAN library
#include <mcp2515.h> // Include MCP2515 library
#include <SPI.h>     // Include SPI library
#include <EEPROM.h>

struct can_frame canMsg;     // Create a structure for receiving CAN packet
struct can_frame new_canMsg; // Create a structure for sending CAN packet

struct can_frame canAmbiance; // Create a structure for sending CAN packet

struct can_frame canTheme; // Create a structure for sending CAN packet

struct can_frame can217msg2;

int front_panel_command; // Declare a variable for FMUX panel command (5 bytes - numbering from 0)
int steer_key_1;         // Declare a variable for steering wheel button command
int steer_key_2;         // Declare a variable for steering wheel button command
int volume;              // Declare a variable for current volume level
bool sportMode = true;   // Check if sport mode is activated
bool ecoMode = false;    // Check if sport mode is activated

bool Animation_done = false; // set to true to not do animation
bool DriverDoor = false;

// ambiance
bool EscState = false;
bool LastEscState = false;
int ambiance = 0x4E; // default value at startup, 0E= off, 8E=relax ambiance,  4E= boost ambiance
int theme = 0x01;    // default value at startup, 01= blue, 02=bronze
int Theme1A9Send = 0;
unsigned long ESCtimer = 0;

MCP2515 mcp2515(10); //

void setup() // Set up serial interface and CAN bus interface
{
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  ambiance = EEPROM.read(0);

  canAmbiance.can_id = 0x2E9; // Assign the FMUX panel id
  canAmbiance.can_dlc = 4;    // Specify the length of the packet
  canAmbiance.data[0] = 0x00; // Empty data
  canAmbiance.data[1] = 6;    // Empty data
  canAmbiance.data[2] = 0x00; // Empty data
  canAmbiance.data[3] = 0x00; // Empty data

  bitSet(canAmbiance.data[1], 6); // Set red theme

  canAmbiance.can_id = 0x2E9;
  canAmbiance.can_dlc = 4;
}

void loop() // Start reading data loop from the CAN bus
{
  // Serial.println("BOMBOU");
  mcp2515.readMessage(&canMsg);
  Serial.println(canMsg.can_id);
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) // If there are no errors, continue
  {

    Serial.println("BOMBOU dentro");
    if (canMsg.can_id == 0x00E)
    { // door state
      DriverDoor = bitRead(canMsg.data[1], 6);
      Serial.print("DriverDoor is  :  ");
      Serial.println(DriverDoor);
    }
    if (canMsg.can_id == 0x236)
    { // ANIMATION
      if (!Animation_done && DriverDoor)
      {                      // 5s timeout
        new_canMsg = canMsg; // copy frame
        new_canMsg.data[5] = bitWrite(new_canMsg.data[5], 6, 1);
        mcp2515.sendMessage(&new_canMsg);
        Animation_done = true;
      }
    }

    if (canMsg.can_id == 0x1A9)
    {
      if (Theme1A9Send >= 1)
      {
        Theme1A9Send = Theme1A9Send - 1;
        canTheme = canMsg; // copy frame
        canTheme.data[6] = bitWrite(canTheme.data[6], 5, 1);
        mcp2515.sendMessage(&canTheme);
        Serial.print("Theme1A9sent (70), new number is:  ");
        Serial.println(Theme1A9Send, DEC);
      }
    }

    if (canMsg.can_id == 0x2E9)
    {
      // Requested ambiance change                                                                                                               //copy frame
      canAmbiance.data[0] = ((canAmbiance.data[0] & 0xFC) | (theme & 0x03));                           //theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
      canAmbiance.data[1] = ((canAmbiance.data[1] & 0x3F) | (ambiance & 0xC0));                        //ambiance value is only in bit 6&7=0xC0 mask
      // if ((canMsg.data[0] != canAmbiance.data[0]) || (canMsg.data[1] != canAmbiance.data[1]))
      // { // diffrent value received from NAC, we need to request the new value
        mcp2515.sendMessage(&canAmbiance);
        Serial.print("2E9 sent with theme:  ");
        Serial.println(theme, HEX);
        Serial.print("2E9 sent with ambiance:  ");
        Serial.println(canAmbiance.data[1], HEX);
      // }
    }

    if (canMsg.can_id == 0xA2)
    { // VCI state lower right (ESC)
      LastEscState = EscState;
      EscState = bitRead(canMsg.data[1], 4); // ESC key
      // Serial.println(EscState);
      if (EscState && !LastEscState)
      { // is pushed and wasnot pushed before
        Serial.println("ESC pressed");
        ESCtimer = millis();
      }
      if (!EscState && LastEscState)
      {
        Serial.println("ESC released");
        if ((millis() - ESCtimer) >= 1000)
        {
          Serial.println("ESC long press");
          Theme1A9Send = 2; // number of time to send 70 value (2time on nac)
          switch (theme)
          {
          case 0x01:
            theme = 0x02;
            break; // Switch blue to bzonze
          case 0x02:
            theme = 0x01;
            break; // Switch bzonze to blue
          default:
            ambiance = 0x01;
            break; // return to off for any other value
          }

          Serial.println(theme);

          canTheme = canMsg;
          // copy frame
          canTheme.data[0] = ((canTheme.data[0] & 0xFC) | (theme & 0x03)); // theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
                                                                           //  if ((canTheme.data[0] != canTheme.data[0]) || (canTheme.data[1] != canTheme.data[1])) {  //diffrent value received from NAC, we need to request the new value
          mcp2515.sendMessage(&canTheme);

          Serial.print("Theme is  :  ");
          Serial.println(theme, HEX);
        }
        else
        {
          Serial.println("ESC short press");
          switch (ambiance)
          {
          case 0x0E:
            ambiance = 0x4E;
            break; // Switch off to boost
          case 0x4E:
            ambiance = 0x8E;
            break; // Switch boost to relax
          case 0x8E:
            ambiance = 0x0E;
            break; // Switch relax to off
          default:
            ambiance = 0x0E;
            break; // return to off for any other value
          }

          EEPROM.update(0, ambiance);

          Serial.print("ambiance is  :  ");
          Serial.println(ambiance, HEX);
        }
      }
    }

    if (sportMode)
    {
        Serial.println("Sport mode ON");
          bitWrite(can217msg2.data[2], 6, 1);

        mcp2515.sendMessage(&can217msg2);
    }
    if (canMsg.can_id == 0x128) {  // IF TRANSMITION THEME
      if (canMsg.data[2] == 32) {
        sportMode = true;
      } 
      else {
        sportMode = false;
      }
    }
    if (canMsg.can_id == 0x217)
    { // IF MSG TRANSMITION LIGHT
      can217msg2 = canMsg;
    }
  }
}