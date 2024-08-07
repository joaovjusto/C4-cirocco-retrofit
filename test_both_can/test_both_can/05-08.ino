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

struct can_frame canOff;  // Create a structure for sending CAN packet

struct can_frame canAmbiance;  // Create a structure for sending CAN packet

struct can_frame canTheme;  // Create a structure for sending CAN packet

struct can_frame canLogo;  // Create a structure for sending not peugeot logo

struct can_frame canFakeIgnitionOn;  // Create a structure for sending not peugeot logo

struct can_frame canSrc;  // Create a structure for sending not peugeot logo

struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

// Stop Start deletion
bool ignition = false;

// MEM fix
bool Lastingnition = false;
unsigned long ignitiontimer = 0;


// ambiance
bool EscState = false;
bool LastEscState = false;
int ambiance = 0x0E;  // default value at startup, 0E= off, 8E=relax ambiance,  4E= boost ambiance
int theme = 0x01;     // default value at startup, 01= blue, 02=bronze
int Theme1A9Send = 0;
unsigned long ESCtimer = 0;

void setup() {
  ambiance = EEPROM.read(0);

  canAmbiance.can_id = 0x2E9;      // Assign the FMUX panel id
  canAmbiance.can_dlc = 4;         // Specify the length of the packet
  canAmbiance.data[0] = theme;     // Empty data
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

  CAN1.reset();
  CAN1.setBitrate(CAN_SPEED, CAN_FREQ);
  CAN1.setNormalMode();
}

void loop() {
  // CIROCCO
  if (CAN0.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {

    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    CAN1.sendMessage(&canMsgRcv);

    if(!ignition && Lastingnition) {
      CAN0.sendMessage(&canOff);
    }
  }

  // Listen CAR MESSAGES
  if (CAN1.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    // Block to Jump peugeot Logo
    if (!ignition) {
        if (id == 0x128) {

          canOff = canMsgRcv;

          canOff.data[0] = 0;
          canOff.data[1] = 0;
          canOff.data[2] = 0;
          canOff.data[3] = 0;
          canOff.data[4] = 0;
          canOff.data[5] = 0;
          canOff.data[6] = 0;
          canOff.data[7] = 0;

          CAN0.sendMessage(&canOff);
        } else {
          // CAN0.sendMessage(&canOff);
          canFakeIgnitionOn.data[0] = 0x88;
          // CAN0.sendMessage(&canFakeIgnitionOn);
          CAN0.sendMessage(&canMsgRcv);
        }
    } else {
      // Normal flux
      if (id == 0x2E9) {
        canAmbiance.data[0] = theme;
        canAmbiance.data[1] = ambiance;
        CAN0.sendMessage(&canAmbiance);
        canTheme = canAmbiance;
      } else {
        CAN0.sendMessage(&canMsgRcv);
      }
    }

    if (id == 0xF6) {  // If for turning indicator ignition
      Lastingnition = ignition;
      ignition = bitRead(canMsgRcv.data[0], 3);
      if (ignition && !Lastingnition) {
        ignitiontimer = millis();
      }  // ignition switched to ON
      if (!ignition) {
        // CAN0.sendMessage(&canOff);
        canLogo.data[3] = 0x35;
        CAN0.sendMessage(&canLogo);

        canFakeIgnitionOn = canMsgRcv;
        canFakeIgnitionOn.data[0] = 0x88;
        CAN0.sendMessage(&canFakeIgnitionOn);
      }
    }

    if (id == 0x1A9) {
      if (Theme1A9Send >= 1) {
        Theme1A9Send = Theme1A9Send - 1;
        canTheme = canMsgRcv;  // copy frame
        canTheme.data[6] = bitWrite(canTheme.data[6], 5, 1);
        CAN0.sendMessage(&canTheme);
        CAN1.sendMessage(&canTheme);
      }
    }

    if (id == 0xA2) {  // VCI state lower right (ESC)
      LastEscState = EscState;
      EscState = bitRead(canMsgRcv.data[1], 4);  // ESC key
      if (EscState && !LastEscState) {  // is pushed and wasnot pushed before
        ESCtimer = millis();
      }
      if (!EscState && LastEscState) {
        if ((millis() - ESCtimer) >= 1000) {
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

          canTheme.data[0] = ((canTheme.data[0] & 0xFC) | (theme & 0x03));  // theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
                                                                            //  if ((canTheme.data[0] != canTheme.data[0]) || (canTheme.data[1] != canTheme.data[1])) {  //diffrent value received from NAC, we need to request the new value
          CAN1.sendMessage(&canTheme);
          CAN0.sendMessage(&canTheme);
        } else {
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
        }
      }
    }
  }
}