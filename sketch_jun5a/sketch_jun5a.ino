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

struct can_frame canTransmition;  // Create a structure for sending not peugeot logo

struct can_frame canSrc;  // Create a structure for sending not peugeot logo

struct can_frame canMsgSnd;
struct can_frame canMsgRcv;

// Bool for avoiding duplicate push
bool sendSRC = false;
bool sportMode = false;

// Stop Start deletion
bool ignition = false;

unsigned long previousMillis = 0UL;
unsigned long interval = 5000UL;
int ignitionCounter = 0;

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
  unsigned long currentMillis = millis();
  // CIROCCO
  if (CAN0.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {

    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    // Serial.println("CAN0");


    CAN1.sendMessage(&canMsgRcv);

    // if (id == 0x217) {  // IF MSG TRANSMITION LIGHT
    //   canTransmition = canMsgRcv;
    // }
  }

  // Listen CAR MESSAGES
  if (CAN1.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    // Serial.println("CAN1");
    // Serial.println(id);

    // if (id == 0x122)  // If the packet is from the FMUX panel
    // {
    //   if (sendSRC) {
    //     // Serial.println("MANDANDO COMANDO");
    //     sendSRC = false;

    //     canMsgSnd = canMsgRcv;

    //     canMsgSnd.data[1] = 32;
    //     CAN1.sendMessage(&canMsgSnd);
    //   }
    // }

    // if (id == 0x21F)  // If the packet is from the steering wheel buttons
    // {
    //   if (canMsgRcv.data[2] == 64) {
    //     sendSRC = true;
    //   }
    // }
    // if (!ignition) {
    //   // FAKE DARK MODE
    //   if (id == 0x128) {
    //     canOff = canMsgRcv;
    //     canOff.data[6] = 0;
    //   }
    // } else {
    // Normal flux
    // if (id == 0x2E9) {
    //   canAmbiance.data[0] = theme;
    //   canAmbiance.data[1] = ambiance;
    //   CAN0.sendMessage(&canAmbiance);
    //   canTheme = canAmbiance;
    // } else {
    CAN0.sendMessage(&canMsgRcv);
    // }
    // }

    // if (id == 0xF6) {  // If for turning indicator ignition
    //   Serial.println(ignitionCounter);
    //   if (bitRead(canMsgRcv.data[0], 3) == 1) {
    //     if (currentMillis - previousMillis > interval) {
    //       if (ignitionCounter > 4) {
    //         ignition = bitRead(canMsgRcv.data[0], 3);
    //         Serial.print("setando ignition para ");
    //         Serial.println(ignition);

    //         previousMillis = currentMillis;
    //       }
    //       ignitionCounter = ignitionCounter + 1;
    //     }

    //   } else {
    //     ignition = 0;
    //     ignitionCounter = 0;
    //   }

    //   if (!ignition) {
    //     CAN0.sendMessage(&canOff);
    //   }
    // }

    // if (id == 0x1A9) {
    //   if (Theme1A9Send >= 1) {
    //     Theme1A9Send = Theme1A9Send - 1;
    //     canTheme = canMsgRcv;  // copy frame
    //     canTheme.data[6] = bitWrite(canTheme.data[6], 5, 1);
    //     CAN0.sendMessage(&canTheme);
    //     CAN1.sendMessage(&canTheme);
    //     // Serial.print("Theme1A9sent (70), new number is:  ");
    //     // Serial.println(Theme1A9Send, DEC);
    //   }
    // }

    // if (id == 0xA2) {  // VCI state lower right (ESC)
    //   LastEscState = EscState;
    //   EscState = bitRead(canMsgRcv.data[1], 4);  // ESC key
    //   // Serial.println(EscState);
    //   if (EscState && !LastEscState) {  // is pushed and wasnot pushed before
    //     // Serial.println("ESC pressed");
    //     ESCtimer = millis();
    //   }
    //   if (!EscState && LastEscState) {
    //     // Serial.println("ESC released");
    //     if ((millis() - ESCtimer) >= 1000) {
    //       // Serial.println("ESC long press");
    //       Theme1A9Send = 2;  // number of time to send 70 value (2time on nac)
    //       switch (theme) {
    //         case 0x01:
    //           theme = 0x02;
    //           break;  // Switch blue to bzonze
    //         case 0x02:
    //           theme = 0x01;
    //           break;  // Switch bzonze to blue
    //         default:
    //           ambiance = 0x01;
    //           break;  // return to off for any other value
    //       }

    //       // Serial.println(theme);

    //       canTheme.data[0] = ((canTheme.data[0] & 0xFC) | (theme & 0x03));  // theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
    //                                                                         //  if ((canTheme.data[0] != canTheme.data[0]) || (canTheme.data[1] != canTheme.data[1])) {  //diffrent value received from NAC, we need to request the new value
    //       CAN1.sendMessage(&canTheme);
    //       CAN0.sendMessage(&canTheme);

    //       // Serial.print("Theme is  :  ");
    //       // Serial.println(theme, HEX);
    //     } else {
    //       // Serial.println("ESC short press");
    //       switch (ambiance) {
    //         case 0x0E:
    //           ambiance = 0x4E;
    //           break;  // Switch off to boost
    //         case 0x4E:
    //           ambiance = 0x8E;
    //           break;  // Switch boost to relax
    //         case 0x8E:
    //           ambiance = 0x0E;
    //           break;  // Switch relax to off
    //         default:
    //           ambiance = 0x0E;
    //           break;  // return to off for any other value
    //       }

    //       EEPROM.update(0, ambiance);

    //       // Serial.print("ambiance is  :  ");
    //       // Serial.println(ambiance, HEX);
    //     }
    //   }
    // }
  }
}