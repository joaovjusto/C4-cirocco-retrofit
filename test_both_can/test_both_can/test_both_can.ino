/////////////////////
//    Libraries    //
/////////////////////

#include <SPI.h>
#include <mcp2515.h>  // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast
#include <EEPROM.h>

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10  // pin connected to NAC/cirocco CAN bus module
#define CS_PIN_CAN1 9   // pin connected to CVM CAN bus module
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS  // Diagnostic CAN bus - High Speed
#define CAN_FREQ MCP_8MHZ      // Switch to 16MHZ if you have a 16Mhz module

MCP2515 CAN0(CS_PIN_CAN0);  // NAC/cirocco CAN bus
MCP2515 CAN1(CS_PIN_CAN1);  // CVM CAN bus

////////////////////
//   Variables    //
////////////////////

// My variables
bool debugCAN0 = false;  // Debug for NAC/cirocco CAN bus
bool debugCAN1 = false;  // Debug CVM CAN bus
bool SerialEnabled = true;

// CAN-BUS Messages

struct can_frame canAmbiance;  // Create a structure for sending CAN packet

struct can_frame canTheme;  // Create a structure for sending CAN packet

struct can_frame canLogo;  // Create a structure for sending not peugeot logo

struct can_frame canFakeIgnitionOn;  // Create a structure for sending not peugeot logo

struct can_frame canTransmition;  // Create a structure for sending not peugeot logo

struct can_frame canSrc;  // Create a structure for sending not peugeot logo

struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

// Bool for avoiding duplicate push
bool sendSRC = false;
bool sportMode = false;
bool SAMsend = false;
bool SAM_NAC = false;
bool lastSAM_NAC = false;

bool Animation_done = false;  // set to true to not do animation
bool DriverDoor = false;

// Stop Start deletion
bool EngineBeenStarted = false;
bool SSdesactivationDone = false;
bool SSrequest = false;
bool ignition = false;

// MEM fix
bool Lastingnition = false;
unsigned long ignitiontimer = 0;
bool SAMstatus = false;

// ambiance
bool EscState = false;
bool LastEscState = false;
int ambiance = 0x0E;  // default value at startup, 0E= off, 8E=relax ambiance,  4E= boost ambiance
int theme = 0x01;     // default value at startup, 01= blue, 02=bronze
int Theme1A9Send = 0;
int volume = 0;
int front_panel_command;  // Declare a variable for FMUX panel command (5 bytes - numbering from 0)
int steer_key_1;          // Declare a variable for steering wheel button command
int steer_key_2;          // Declare a variable for steering wheel button command
unsigned long ESCtimer = 0;

void setup() {
  ambiance = EEPROM.read(0);

  canAmbiance.can_id = 0x2E9;      // Assign the FMUX panel id
  canAmbiance.can_dlc = 4;         // Specify the length of the packet
  canAmbiance.data[0] = 0x01;      // Empty data
  canAmbiance.data[1] = ambiance;  // Empty data
  canAmbiance.data[2] = 0x58;      // Empty data
  canAmbiance.data[3] = 0x00;      // Empty data

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
  // CAN1.setListenOnlyMode();
  while (CAN1.setNormalMode() != MCP2515::ERROR_OK) {
    delay(100);
    Serial.println("can1 ok");
  }
}

void loop() {
  // CIROCCO
  if (CAN0.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {

    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    CAN1.sendMessage(&canMsgRcv);

    if (id == 0x217) {  // IF MSG TRANSMITION LIGHT
      canTransmition = canMsgRcv;
    }

    // if (id == 0x217 && SAMsend) {  // 217 received and something need to be send
    //   Serial.println("217 received");
    //   canMsgSnd = canMsgRcv;  // copy frame
    //   canMsgSnd.data[3] = bitWrite(canMsgSnd.data[3], 3, SAMsend);
    //   SAMsend = false;
    //   Serial.println("SAM struture written and SAMsend reset CIROCCO");
    //   CAN0.sendMessage(&canMsgSnd);
    //   CAN1.sendMessage(&canMsgSnd);
    //   Serial.println("217 sent");
    // }
  }

  // Listen CAR MESSAGES
  if (CAN1.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (id == 0x122)  // If the packet is from the FMUX panel
    {
      if(sendSRC) {
        Serial.println("MANDANDO COMANDO");
        sendSRC = false;

        canMsgSnd = canMsgRcv;

        canMsgSnd.data[1] = 32;
        // canMsgSnd.data[1] = 20; -- ESSE VALOR MANDA PRO CARPLAY
        CAN1.sendMessage(&canMsgSnd);
      }
    }

    if (id == 0x21F)  // If the packet is from the steering wheel buttons
    {
      if(canMsgRcv.data[2] == 64) {
        sendSRC = true;
      }
    }

    if (sportMode) {
      Serial.println("Sport mode ON");
      bitWrite(canTransmition.data[2], 6, 1);

      CAN1.sendMessage(&canTransmition);
    }

    if (id == 0x128) {  // IF TRANSMITION THEME
      if (canMsgRcv.data[2] == 32) {
        sportMode = true;
      } else {
        sportMode = false;
      }
    }

    if (id == 0x236) {                      // ANIMATION
      if (!Animation_done && DriverDoor) {  // 5s timeout
        canMsgSnd = canMsgRcv;              // copy frame
        canMsgSnd.data[5] = bitWrite(canMsgSnd.data[5], 6, 1);
        CAN0.sendMessage(&canMsgSnd);
        Animation_done = true;
      }
    }

    // Block to Jump peugeot Logo
    if (ignition == false) {
      // FAKE IGNITION ON
      if (id == 0xF6) {
        canFakeIgnitionOn.data[0] = 0x88;

        CAN0.sendMessage(&canFakeIgnitionOn);
      } else if (id == 0x2E9) {
        canAmbiance.data[1] = ambiance;
        CAN0.sendMessage(&canAmbiance);
        canTheme = canMsgRcv;
      } else
        // FAKE DARK MODE
        if (id == 0x036) {
          canLogo = canMsgRcv;
          canLogo.data[3] = 0x35;
          CAN0.sendMessage(&canLogo);
        } else {
          CAN0.sendMessage(&canMsgRcv);
        }
    } else {
      // Normal flux
      if (id == 0x2E9) {
        canAmbiance.data[1] = ambiance;
        CAN0.sendMessage(&canAmbiance);
        canTheme = canMsgRcv;
      } else {
        CAN0.sendMessage(&canMsgRcv);
      }
    }

    // if (id == 0x2D1) {  // frame for SAM state (turn on cirocco line)
    //   SAMstatus = bitRead(canMsgRcv.data[0], 2);
    //   if (!SAMstatus && ignition) {
    //     Serial.println("SAM ON");
    //   } else {
    //     Serial.println("SAM OFF");
    //   }

    //   if (SAMstatus == 0) {  // SAM active
    //     canMsgSnd.can_id = 0x321;
    //     canMsgSnd.can_dlc = 5;
    //     canMsgSnd.data[0] = 0x0;
    //     canMsgSnd.data[1] = 0x0;
    //     canMsgSnd.data[2] = 0x0;
    //     canMsgSnd.data[3] = 0x0;
    //     canMsgSnd.data[4] = 0x0;
    //     CAN1.sendMessage(&canMsgSnd);  // send 0x321 frame to turn on indicator
    //   }
    // }

    if (id == 0xF6) {  // If for turning indicator ignition

      Lastingnition = ignition;
      ignition = bitRead(canMsgRcv.data[0], 3);
      if (ignition && !Lastingnition) {
        ignitiontimer = millis();
      }  // ignition switched to ON
      if (!ignition) {
        canFakeIgnitionOn = canMsgRcv;
        canFakeIgnitionOn.data[0] = 0x88;

        CAN0.sendMessage(&canFakeIgnitionOn);
        SSdesactivationDone = false;  // request a new SS desactivation  if ignition is off
        EngineBeenStarted = false;    // reset EngineBeenStarted (if ignition off engine can't be running) If not reseted, on "warm" start (arduino not powered off between 2 engine start) SS will deactivate when as soon as ignition is on
      }
    }

    if (id == 0x1A9) {
      if (Theme1A9Send >= 1) {
        Theme1A9Send = Theme1A9Send - 1;
        canTheme = canMsgRcv;  // copy frame
        canTheme.data[6] = bitWrite(canTheme.data[6], 5, 1);
        CAN0.sendMessage(&canTheme);
        CAN1.sendMessage(&canTheme);
        Serial.print("Theme1A9sent (70), new number is:  ");
        Serial.println(Theme1A9Send, DEC);
      }
    }

    if (id == 0x00E) {  // door state
      DriverDoor = bitRead(canMsgRcv.data[1], 6);
    }

    if (id == 0xA2) {  // VCI state lower right (ESC)
      LastEscState = EscState;
      EscState = bitRead(canMsgRcv.data[1], 4);  // ESC key
      // Serial.println(EscState);
      if (EscState && !LastEscState) {  // is pushed and wasnot pushed before
        Serial.println("ESC pressed");
        ESCtimer = millis();
      }
      if (!EscState && LastEscState) {
        Serial.println("ESC released");
        if ((millis() - ESCtimer) >= 1000) {
          Serial.println("ESC long press");
          Theme1A9Send = 2;  // number of time to send 70 value (2time on nac)
          switch (theme) {
            case 0x01:
              theme = 0x02;
              break;  // Switch blue to bzonze
            case 0x02:
              theme = 0x01;
              break;  // Switch bzonze to blue
            default:
              ambiance = 0x01;
              break;  // return to off for any other value
          }

          Serial.println(theme);

          // canTheme = canMsgRcv;
          // copy frame
          canTheme.data[0] = ((canTheme.data[0] & 0xFC) | (theme & 0x03));  // theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
                                                                            //  if ((canTheme.data[0] != canTheme.data[0]) || (canTheme.data[1] != canTheme.data[1])) {  //diffrent value received from NAC, we need to request the new value
          CAN1.sendMessage(&canTheme);
          CAN0.sendMessage(&canTheme);

          Serial.print("Theme is  :  ");
          Serial.println(theme, HEX);
        } else {
          Serial.println("ESC short press");
          switch (ambiance) {
            case 0x0E:
              ambiance = 0x4E;
              break;  // Switch off to boost
            case 0x4E:
              ambiance = 0x8E;
              break;  // Switch boost to relax
            case 0x8E:
              ambiance = 0x0E;
              break;  // Switch relax to off
            default:
              ambiance = 0x0E;
              break;  // return to off for any other value
          }

          EEPROM.update(0, ambiance);

          Serial.print("ambiance is  :  ");
          Serial.println(ambiance, HEX);
        }
      }
    }
  }
}