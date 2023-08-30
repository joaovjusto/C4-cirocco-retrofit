/*
C4-cirocco-retrofit
*/

/////////////////////
//    Libraries    //
/////////////////////

#include <SPI.h>
#include <mcp2515.h>  // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 53  //pin connected to NAC/cirocco CAN bus module
#define CS_PIN_CAN1 53  //pin connected to CVM CAN bus module
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS  // Diagnostic CAN bus - High Speed
#define CAN_FREQ MCP_8MHZ      // Switch to 16MHZ if you have a 16Mhz module


MCP2515 CAN0(CS_PIN_CAN0);  // NAC/cirocco CAN bus
MCP2515 CAN1(CS_PIN_CAN1);  // CVM CAN bus


////////////////////
//   Variables    //
////////////////////

// My variables
bool debugCAN0 = false;  //Debug for NAC/cirocco CAN bus
bool debugCAN1 = false;  //Debug CVM CAN bus
bool SerialEnabled = true;

// CAN-BUS Messages
struct can_frame canMsgSnd;
struct can_frame canMsgRcv;
bool IDChanged = false;

//Bool for avoiding duplicate push
bool SAMsend = false;
bool SAM_NAC = false;
bool lastSAM_NAC = false;

bool left = false;
bool right = false;

bool Animation_done = false;  //set to true to not do animation
bool DriverDoor = false;


//Stop Start deletion
bool ignition = false;

int SAMledState = LOW;      // the current state of LED
int SAMlastButtonState;     // the previous state of button
int SAMcurrentButtonState;  // the current state of button
bool SAMstatus = false;
unsigned long ButtonTimer = 0;

//ambiance
bool EscState = false;
bool LastEscState = false;
int ambiance = 0x0E;  //default value at startup, 0E= off, 8E=relax ambiance,  4E= boost ambiance
int theme = 0x01;     //default value at startup, 01= blue, 02=bronze
int Theme1A9Send = 0;
unsigned long ESCtimer = 0;

int front_panel_command;  // Declare a variable for FMUX panel command (5 bytes - numbering from 0)
int steer_key_1;          // Declare a variable for steering wheel button command
int steer_key_2;          // Declare a variable for steering wheel button command
int volume;               // Declare a variable for current volume level

struct can_frame canMsg;      // Create a structure for receiving CAN packet
struct can_frame new_canMsg;  // Create a structure for sending CAN packet

struct can_frame can128msg;  // Create a structure for sending CAN packet

struct can_frame can217msg;
struct can_frame can217msg2;

struct can_frame can227msg;
struct can_frame can227msg2;

bool sportModeInit = false;
bool sportModeDeInit = false;

bool ecoModeInit = false;
bool ecoModeDeInit = false;
bool sportMode = false;  // Check if sport mode is activated
bool ecoMode = false;    // Check if sport mode is activated



void setup() {
  ;
  if (SerialEnabled) {
    // Initalize Serial for debug
    Serial.begin(SERIAL_SPEED);
    // CAN-BUS to CAN2010 device(s)
    Serial.println("Initialization CAN");
  }

  CAN0.reset();
  CAN0.setBitrate(CAN_SPEED, CAN_FREQ);
  CAN0.setNormalMode();
  while (CAN0.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("can0 ok");
  }


  CAN1.reset();
  CAN1.setBitrate(CAN_SPEED, CAN_FREQ);
  CAN1.setNormalMode();
  //CAN1.setListenOnlyMode();
  while (CAN1.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("can1 ok");
  }
}

void loop() {
  // Receive CAN messages from the car
  if (CAN0.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (id == 0x122)  // If the packet is from the FMUX panel
    {
      if (canMsgRcv.data[5] == 0)  // This condition is true when the car wakes up and no volume adjustment is used on the FMUX panel or steering wheel (via Arduino)
      {
        steer_key_1 = 0x08;               // Assign the volume up button value to the variable
        steer_key_2 = 0x04;               // Assign the volume down button value to the variable
        front_panel_command = 0x02;       // Assign the value 0 to the variable (5th byte)
      } else if (canMsgRcv.data[5] == 2)  // This condition is true when volume adjustment is already used on the FMUX panel or steering wheel (via Arduino)
      {
        steer_key_1 = 0x04;          // Assign the volume down button value to the variable
        steer_key_2 = 0x08;          // Assign the volume up button value to the variable
        front_panel_command = 0x00;  // Assign the value 0 to the variable (5th byte)
      } else {
        // Do nothing, no other data available
      }
      volume = canMsgRcv.data[6];  // Assign the current volume control position of the FMUX panel
    }

    if (id == 0x21F)  // If the packet is from the steering wheel buttons
    {
      if (canMsgRcv.data[0] == steer_key_1)  // Determine which button is pressed
      {
        if (volume == 255) {
          volume = 0;
        } else {
          volume++;
        }

        canMsgSnd.can_id = 0x122;                 // Assign the FMUX panel id
        canMsgSnd.can_dlc = 8;                    // Specify the length of the packet
        canMsgSnd.data[0] = 0x00;                 // Empty data
        canMsgSnd.data[1] = 0x00;                 // Empty data
        canMsgSnd.data[2] = 0x00;                 // Empty data
        canMsgSnd.data[3] = 0x00;                 // Empty data
        canMsgSnd.data[4] = 0x00;                 // Empty data
        canMsgSnd.data[5] = front_panel_command;  // Panel command
        canMsgSnd.data[6] = volume;               // Assign the new volume level
        canMsgSnd.data[7] = 0x00;                 // Empty data

        CAN0.sendMessage(&canMsgSnd);  // Send the new volume level command

        // delay(120);  // Delay 0.15 seconds
      }

      if (canMsgRcv.data[0] == steer_key_2)  // Determine which button is pressed
      {
        if (volume == 0) {
          volume = 255;
        } else {
          volume--;
        }

        canMsgSnd.can_id = 0x122;                 // Assign the FMUX panel id
        canMsgSnd.can_dlc = 8;                    // Specify the length of the packet
        canMsgSnd.data[0] = 0x00;                 // Empty data
        canMsgSnd.data[1] = 0x00;                 // Empty data
        canMsgSnd.data[2] = 0x00;                 // Empty data
        canMsgSnd.data[3] = 0x00;                 // Empty data
        canMsgSnd.data[4] = 0x00;                 // Empty data
        canMsgSnd.data[5] = front_panel_command;  // Panel command
        canMsgSnd.data[6] = volume;               // Assign the new volume level
        canMsgSnd.data[7] = 0x00;                 // Empty data

        CAN0.sendMessage(&canMsgSnd);  // Send the new volume level command

        // delay(120);  // Delay 0.12 seconds
      }
    }

    if (id == 0x1A9) {                //NAC message
      if (SAM_NAC && !lastSAM_NAC) {  //is pushed and wasnot pushed before
        SAMsend = true;
        Serial.println("SAMsend asked ");
      }

      if (Theme1A9Send >= 1) {
        Theme1A9Send = Theme1A9Send - 1;
        canMsgSnd = canMsgRcv;  //copy frame
        canMsgSnd.data[6] = bitWrite(canMsgSnd.data[6], 5, 1);
        CAN0.sendMessage(&canMsgSnd);
        // Serial.print("Theme1A9sent (70), new number is:  "); Serial.println(Theme1A9Send, DEC);
      }
    }

    if (id == 0x217 && SAMsend) {  //217 received and something need to be send
      //Serial.println("217 received");
      canMsgSnd = canMsgRcv;  //copy frame
      canMsgSnd.data[3] = bitWrite(canMsgSnd.data[3], 3, SAMsend);
      SAMsend = false;
      Serial.println("SAM struture written and SAMsend reset");
      CAN0.sendMessage(&canMsgSnd);
      Serial.println("217 sent");
    }

    // if (id == 0x2D1) {  //frame for SAM state (turn on cirocco line)
    //   SAMstatus = bitRead(canMsgRcv.data[0], 2);
    //   if (!SAMstatus && ignition) {
    //     Serial.println("SAMON");
    //   } else {
    //     Serial.println("SAMOFF");
    //   }

    //   if (SAMstatus == 0) {  //SAM active
    //     canMsgSnd.can_id = 0x321;
    //     canMsgSnd.can_dlc = 5;
    //     canMsgSnd.data[0] = 0x0;
    //     canMsgSnd.data[1] = 0x0;
    //     canMsgSnd.data[2] = 0x0;
    //     canMsgSnd.data[3] = 0x0;
    //     canMsgSnd.data[4] = 0x0;
    //     CAN0.sendMessage(&canMsgSnd);  //send 0x321 frame to turn on indicator
    //   }
    // }

    // if (id == 0x236) {                      //ANIMATION
    //   if (!Animation_done && DriverDoor) {  //5s timeout
    //     canMsgSnd = canMsgRcv;              //copy frame
    //     canMsgSnd.data[5] = bitWrite(canMsgSnd.data[5], 6, 1);
    //     CAN0.sendMessage(&canMsgSnd);
    //     Animation_done = true;
    //   }
    // }

    if (id == 0x2E9) {  //Requested ambiance change
      Serial.print("RCVdata1 is ");
      Serial.println(canMsgRcv.data[1], HEX);
      Serial.print("ambiance is  :  ");
      Serial.println(ambiance, HEX);
      Serial.print("RCVdata0 is ");
      Serial.println(canMsgRcv.data[0], HEX);
      Serial.print("theme is  :  ");
      Serial.println(theme, HEX);

      canMsgSnd = canMsgRcv;                                                                       //copy frame
      canMsgSnd.data[0] = ((canMsgSnd.data[0] & 0xFC) | (theme & 0x03));                           //theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
      canMsgSnd.data[1] = ((canMsgSnd.data[1] & 0x3F) | (ambiance & 0xC0));                        //ambiance value is only in bit 6&7=0xC0 mask
      if ((canMsgRcv.data[0] != canMsgSnd.data[0]) || (canMsgRcv.data[1] != canMsgSnd.data[1])) {  //diffrent value received from NAC, we need to request the new value
        CAN0.sendMessage(&canMsgSnd);
        //Serial.print("2E9 sent with theme:  "); Serial.println(theme, HEX);
        //Serial.print("2E9 sent with ambiance:  "); Serial.println(canMsgSnd.data[1], HEX);
      }
    }
  }

  // Listen on CVM CAN BUS
  if (CAN1.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    // if (id == 0x00E) {  //door state
    //   DriverDoor = bitRead(canMsgRcv.data[1], 6);
    //   //Serial.print("DriverDoor is  :  ");Serial.println(DriverDoor);
    // }

    if (id == 0xA2) {  //VCI state lower right (ESC)
      LastEscState = EscState;
      EscState = bitRead(canMsgRcv.data[1], 4);  //ESC key
      Serial.println(EscState);
      if (EscState && !LastEscState) {  //is pushed and wasnot pushed before
        Serial.println("ESC pressed");
        ESCtimer = millis();
      }
      if (!EscState && LastEscState) {
        Serial.println("ESC released");
        if ((millis() - ESCtimer) >= 1000) {
          Serial.println("ESC long press");
          Theme1A9Send = 2;  //number of time to send 70 value (2time on nac)
          switch (theme) {
            case 0x01: theme = 0x02; break;   //Switch blue to bzonze
            case 0x02: theme = 0x01; break;   //Switch bzonze to blue
            default: ambiance = 0x01; break;  //return to off for any other value
          }
          Serial.print("Theme is  :  ");
          Serial.println(theme, HEX);

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
            if (ambiance == 0x4E) {
              sportModeInit = false;
              if (bitRead(can227msg.data[0], 4) == 0) {
                Serial.println("Sport mode ON");
                bitWrite(can217msg2.data[2], 6, 1);
              }
              bitWrite(can217msg2.data[3], 3, 1);

              CAN0.sendMessage(&can217msg2);
              sportMode = true;
              sportModeDeInit = true;
            } else {
              sportMode = false;
              sportModeInit = true;
            }
          }

          Serial.print("ambiance is  :  ");
          Serial.println(ambiance, HEX);
        }
      }
    }
    if (sportMode) {
      bitSet(can128msg.data[1], 6);  // Set red theme

      can128msg.can_id = 0x2E9;
      can128msg.can_dlc = 4;

      CAN0.sendMessage(&can128msg);  // Send the sport theme to Cirocco

      if (sportModeInit) {
        sportModeInit = false;
        if (bitRead(can227msg.data[0], 4) == 0) {
          Serial.println("Sport mode ON");
          bitWrite(can217msg2.data[2], 6, 1);
        }
        bitWrite(can217msg2.data[3], 3, 1);

        CAN0.sendMessage(&can217msg2);
      }
    } else {
      if (sportModeDeInit) {
        sportModeDeInit = false;
        if (bitRead(can227msg.data[0], 4) == 1) {
          Serial.println("Sport mode OFF");
          bitWrite(can217msg2.data[2], 6, 1);
        }
        bitWrite(can217msg2.data[3], 3, 1);

        CAN0.sendMessage(&can217msg2);
      }
    }
    if (ecoMode) {
      bitSet(can128msg.data[1], 7);  // Set relax theme

      can128msg.can_id = 0x2E9;
      can128msg.can_dlc = 4;

      CAN0.sendMessage(&can128msg);  // Send the red theme to Cirocco

      if (ecoModeInit) {
        ecoModeInit = false;
        Serial.println("eco mode ON");
        // bitWrite(can217msg.data[3], 3, 1);

        // CAN0.sendMessage(&can217msg);
      }
    } else {
      if (ecoModeDeInit) {
        ecoModeDeInit = false;
        Serial.println("Eco mode OFF");
        // bitWrite(can217msg.data[3], 3, 1);

        // CAN0.sendMessage(&can217msg);
      }
    }
    if (canMsg.can_id == 0x128) {  // IF MSG THEME
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
  }
}
