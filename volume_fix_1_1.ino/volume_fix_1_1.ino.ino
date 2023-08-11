/**
   1.1 - Corrigido LIGAR/DESLIGAR ESP se perdendo (logica invertida)

   ID:
   0x128 - Dados do cambio (SPORT/NEVE)
   0x217 - ID para ativar ESP / PARK / PONTO CEGO... (ver doc git)
   0x2E9 - Ambiance

*/

#include <can.h>      // Include CAN library
#include <mcp2515.h>  // Include MCP2515 library
#include <SPI.h>      // Include SPI library

struct can_frame canMsg;      // Create a structure for receiving CAN packet
struct can_frame new_canMsg;  // Create a structure for sending CAN packet

struct can_frame can128msg;  // Create a structure for sending CAN packet

struct can_frame can217msg;
struct can_frame can217msg2;

struct can_frame can2E9Send;
struct can_frame can2E9Rcv;

struct can_frame can227msg;

bool sportModeInit = false;
bool sportModeDeInit = false;

bool ecoModeInit = false;
bool ecoModeDeInit = false;

bool SAMstatus = false;

bool ignition = false;
unsigned long ignitiontimer = 0;


MCP2515 mcp2515(53);  //

void setup()  // Set up serial interface and CAN bus interface
{
  Serial.begin(115200);
  SPI.begin();

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

int front_panel_command;  // Declare a variable for FMUX panel command (5 bytes - numbering from 0)
int steer_key_1;          // Declare a variable for steering wheel button command
int steer_key_2;          // Declare a variable for steering wheel button command
int volume;               // Declare a variable for current volume level
bool sportMode = false;   // Check if sport mode is activated
bool ecoMode = false;     // Check if sport mode is activated

bool Animation_done = false;  //set to true to not do animation
bool DriverDoor = false;

//Bool for avoiding duplicate push
bool SAMsend = true;
bool SAM_NAC = false;
bool lastSAM_NAC = false;

//ambiance
bool EscState = false;
bool LastEscState = false;
int ambiance = 0x0E;  //default value at startup, 0E= off, 8E=relax ambiance,  4E= boost ambiance
int theme = 0x02;     //default value at startup, 01= blue, 02=bronze
int Theme1A9Send = 0;
unsigned long ESCtimer = 0;
bool EngineBeenStarted = false;
unsigned long EngineBeenStartedTimer = 0;
bool SSdesactivationDone = false;
bool Lastingnition = false;

void loop()  // Start reading data loop from the CAN bus
{
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)  // If there are no errors, continue
  {

    if (canMsg.can_id == 0xF6) {  // If for turning indicator ignition
      Lastingnition = ignition;
      ignition = bitRead(canMsg.data[0], 3);
      if (ignition && !Lastingnition) { ignitiontimer = millis(); }  //ignition switched to ON
      if (!ignition) {
        SSdesactivationDone = false;  //request a new SS desactivation  if ignition is off
        EngineBeenStarted = false;    // reset EngineBeenStarted (if ignition off engine can't be running) If not reseted, on "warm" start (arduino not powered off between 2 engine start) SS will deactivate when as soon as ignition is on
      }
    }

    if (canMsg.can_id == 0x00E) {  //door state
      DriverDoor = bitRead(canMsg.data[1], 6);
      Serial.print("DriverDoor is  :  ");
      Serial.println(DriverDoor);
    }
    if (canMsg.can_id == 0x236) {           //ANIMATION
      if (!Animation_done && DriverDoor) {  //5s timeout
        new_canMsg = canMsg;                //copy frame
        new_canMsg.data[5] = bitWrite(new_canMsg.data[5], 6, 1);
        mcp2515.sendMessage(&new_canMsg);
        Animation_done = true;
      }
    }

    if (canMsg.can_id == 0x1A9) {                //NAC message
      if (SAM_NAC && !lastSAM_NAC) {  //is pushed and wasnot pushed before
        SAMsend = true;
        Serial.println("SAMsend asked ");
      }

      if (Theme1A9Send >= 1) {
        Theme1A9Send = Theme1A9Send - 1;
        can217msg2 = canMsg;  //copy frame
        can217msg2.data[6] = bitWrite(can217msg2.data[6], 5, 1);
        mcp2515.sendMessage(&can217msg2);
        Serial.print("Theme1A9sent (70), new number is:  "); Serial.println(Theme1A9Send, DEC);
      }
    }

    if (canMsg.can_id == 0x217 && SAMsend) {  //217 received and something need to be send
      //Serial.println("217 received");
      can217msg = canMsg;  //copy frame
      can217msg.data[3] = bitWrite(can217msg.data[3], 3, SAMsend);
      SAMsend = false;
      Serial.println("SAM struture written and SAMsend reset");
      mcp2515.sendMessage(&can217msg);
      //Serial.println("217 sent");
    }

    if (canMsg.can_id == 0x2E9) {  //Requested ambiance change
      // Serial.print("RCVdata1 is ");
      // Serial.println(canMsgRcv.data[1], HEX);
      // Serial.print("ambiance is  :  ");
      // Serial.println(ambiance, HEX);
      // Serial.print("RCVdata0 is ");
      // Serial.println(canMsgRcv.data[0], HEX);
      // Serial.print("theme is  :  ");
      // Serial.println(theme, HEX);

      can217msg = canMsg;  //copy frame                                                                       //copy frame
      can217msg.data[0] = ((can217msg.data[0] & 0xFC) | (theme & 0x03));                           //theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
      can217msg.data[1] = ((can217msg.data[1] & 0x3F) | (ambiance & 0xC0));                        //ambiance value is only in bit 6&7=0xC0 mask
      if ((canMsg.data[0] != can217msg.data[0]) || (canMsg.data[1] != can217msg.data[1])) {  //diffrent value received from NAC, we need to request the new value
        mcp2515.sendMessage(&can217msg);
        Serial.print("2E9 sent with theme:  "); Serial.println(theme, HEX);
        Serial.print("2E9 sent with ambiance:  "); Serial.println(can217msg.data[1], HEX);
      }
    }

    if (canMsg.can_id == 0xA2) {  //VCI state lower right (ESC)
      can128msg = canMsg;
      LastEscState = EscState;
      EscState = bitRead(canMsg.data[1], 4);  //ESC key
      // Serial.println(EscState);
      if (EscState && !LastEscState) {  //is pushed and wasnot pushed before
        Serial.println("ESC pressed");
        ESCtimer = millis();
      }
      if (!EscState && LastEscState) {
        Serial.println("ESC released");
        if ((millis() - ESCtimer) >= 1000) {
          Serial.println("ESC long press");
          Theme1A9Send = 4;  //number of time to send 70 value (2time on nac)
          switch (theme) {
            case 0x01: theme = 0x02; break;   //Switch blue to bzonze
            case 0x02: theme = 0x01; break;   //Switch bzonze to blue
            default: ambiance = 0x01; break;  //return to off for any other value
          }

          // Serial.println(theme);
          
          can217msg2 = canMsg;    
                                             //copy frame
          can217msg2.data[0] = ((can217msg2.data[0] & 0xFC) | (theme & 0x03));  //theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
                                                                              // if ((can128msg.data[0] != can128msg.data[0]) || (can128msg.data[1] != can128msg.data[1])) {  //diffrent value received from NAC, we need to request the new value
          mcp2515.sendMessage(&can217msg2);

          // Serial.print("Theme is  :  ");
          // Serial.println(theme, HEX);

        } else {
          Serial.println("ESC short press");
          can128msg = canMsg;
          switch (ambiance) {
            case 0x0E: ambiance = 0x8E; break;  //Switch off to relax
            case 0x8E: ambiance = 0x4E; break;  //Switch relax to boost
            case 0x4E: ambiance = 0x0E; break;  //Switch boost to off
            default: ambiance = 0x0E; break;    //return to off for any other value
          }

          if (ambiance == 0x8E) {
            ecoMode = true;
            ecoModeDeInit = true;
          } else {
            ecoMode = false;
            ecoModeInit = true;
          }
          if (ambiance == 0x4E) {
            sportMode = true;
            sportModeDeInit = true;
          } else {
            sportMode = false;
            sportModeInit = true;
          }

          // Serial.print("ambiance is  :  ");
          // Serial.println(ambiance, HEX);
        }
      }
    }

    if (ignition == 0)
    {
      sportMode = false;
      sportModeInit = false;
      sportModeDeInit = false;
      ecoMode = false;
      ecoModeInit = false;
      ecoModeDeInit = false;
    }
    

    if (sportMode) {
      can2E9Send = canMsg;
      bitSet(can2E9Send.data[1], 6);  // Set red theme

      can2E9Send.can_id = 0x2E9;
      can2E9Send.can_dlc = 4;

      mcp2515.sendMessage(&can2E9Send);  // Send the sport theme to Cirocco

      if (sportModeInit) {
        sportModeInit = false;
        Serial.println("Sport mode ON");
        // if (bitRead(can227msg.data[0], 4) == 0) {
          bitWrite(can217msg2.data[2], 6, 1);
        // }
        // bitWrite(can217msg2.data[3], 3, 1);

        mcp2515.sendMessage(&can217msg2);
      }
    } else {
      if (sportModeDeInit) {
        sportModeDeInit = false;
        Serial.println("Sport mode OFF");

        // if (bitRead(can227msg.data[0], 4) == 1) {
          bitWrite(can217msg2.data[2], 6, 1);
        // }
        // bitWrite(can217msg2.data[3], 3, 1);

        mcp2515.sendMessage(&can217msg2);
      }
    }
    if (ecoMode) {
      can2E9Send = canMsg;
      bitSet(can2E9Send.data[1], 7);  // Set relax theme

      can2E9Send.can_id = 0x2E9;
      can2E9Send.can_dlc = 4;

      mcp2515.sendMessage(&can2E9Send);  // Send the red theme to Cirocco

      if (ecoModeInit) {
        ecoModeInit = false;
        Serial.println("eco mode ON");
        // bitWrite(can217msg.data[3], 3, 1);

        mcp2515.sendMessage(&can217msg);
      }
    } else {
      if (ecoModeDeInit) {
        ecoModeDeInit = false;
        Serial.println("Eco mode OFF");
        // bitWrite(can217msg.data[3], 3, 1);

        mcp2515.sendMessage(&can217msg);
      }
    }
    if (canMsg.can_id == 0x128) {  // VALIDATE CLUTCH MODE
      can128msg = canMsg;
      if (ambiance == 0x0E) {
        if (canMsg.data[2] == 96) {
          ecoMode = true;
          ecoModeDeInit = true;
        } else {
          ecoMode = false;
          ecoModeInit = true;
        }
        if (canMsg.data[2] == 32) {
          sportMode = true;
          sportModeDeInit = true;
        } else {
          sportMode = false;
          sportModeInit = true;
        }
      }
    }
    if (canMsg.can_id == 0x217) {  // IF MSG TRANSMITION LIGHT
                                   // Serial.println("LIGHT TRANSMISSION");
      can217msg = canMsg;
      can217msg2 = canMsg;
    }

    if (canMsg.can_id == 0x227) {
      can227msg = canMsg;
    }

    if (canMsg.can_id == 0x2D1) {  //frame for SAM state (turn on cirocco line)
      SAMstatus = bitRead(canMsg.data[0], 2);
      if (!SAMstatus && ignition) {
        Serial.println("SAM ON");
        //        digitalWrite(SAMLED_PIN, HIGH);  //turn on led
      } else {
        Serial.println("SAM OFF");
        //        digitalWrite(SAMLED_PIN, LOW);  //turn off led
      }

      if (SAMstatus == 0) {  //SAM active
        new_canMsg.can_id = 0x321;
        new_canMsg.can_dlc = 5;
        new_canMsg.data[0] = 0x0;
        new_canMsg.data[1] = 0x0;
        new_canMsg.data[2] = 0x0;
        new_canMsg.data[3] = 0x0;
        new_canMsg.data[4] = 0x0;
        mcp2515.sendMessage(&new_canMsg);  //send 0x321 frame to turn on indicator
      }
    }
    if (canMsg.can_id == 0x122)  // If the packet is from the FMUX panel
    {
      if (canMsg.data[5] == 0)  // This condition is true when the car wakes up and no volume adjustment is used on the FMUX panel or steering wheel (via Arduino)
      {
        steer_key_1 = 0x08;            // Assign the volume up button value to the variable
        steer_key_2 = 0x04;            // Assign the volume down button value to the variable
        front_panel_command = 0x02;    // Assign the value 0 to the variable (5th byte)
      } else if (canMsg.data[5] == 2)  // This condition is true when volume adjustment is already used on the FMUX panel or steering wheel (via Arduino)
      {
        steer_key_1 = 0x04;          // Assign the volume down button value to the variable
        steer_key_2 = 0x08;          // Assign the volume up button value to the variable
        front_panel_command = 0x00;  // Assign the value 0 to the variable (5th byte)
      } else {
        // Do nothing, no other data available
      }
      volume = canMsg.data[6];  // Assign the current volume control position of the FMUX panel
    }

    if (canMsg.can_id == 0x21F)  // If the packet is from the steering wheel buttons
    {
      if (canMsg.data[0] == steer_key_1)  // Determine which button is pressed
      {
        if (volume == 255) {
          volume = 0;
        } else {
          volume++;
        }

        new_canMsg.can_id = 0x122;                 // Assign the FMUX panel id
        new_canMsg.can_dlc = 8;                    // Specify the length of the packet
        new_canMsg.data[0] = 0x00;                 // Empty data
        new_canMsg.data[1] = 0x00;                 // Empty data
        new_canMsg.data[2] = 0x00;                 // Empty data
        new_canMsg.data[3] = 0x00;                 // Empty data
        new_canMsg.data[4] = 0x00;                 // Empty data
        new_canMsg.data[5] = front_panel_command;  // Panel command
        new_canMsg.data[6] = volume;               // Assign the new volume level
        new_canMsg.data[7] = 0x00;                 // Empty data

        mcp2515.sendMessage(&new_canMsg);  // Send the new volume level command

        delay(120);  // Delay 0.15 seconds
      }

      if (canMsg.data[0] == steer_key_2)  // Determine which button is pressed
      {
        if (volume == 0) {
          volume = 255;
        } else {
          volume--;
        }

        new_canMsg.can_id = 0x122;                 // Assign the FMUX panel id
        new_canMsg.can_dlc = 8;                    // Specify the length of the packet
        new_canMsg.data[0] = 0x00;                 // Empty data
        new_canMsg.data[1] = 0x00;                 // Empty data
        new_canMsg.data[2] = 0x00;                 // Empty data
        new_canMsg.data[3] = 0x00;                 // Empty data
        new_canMsg.data[4] = 0x00;                 // Empty data
        new_canMsg.data[5] = front_panel_command;  // Panel command
        new_canMsg.data[6] = volume;               // Assign the new volume level
        new_canMsg.data[7] = 0x00;                 // Empty data

        mcp2515.sendMessage(&new_canMsg);  // Send the new volume level command

        delay(120);  // Delay 0.12 seconds
      }
    }
  }
}
