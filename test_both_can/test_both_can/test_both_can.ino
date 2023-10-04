/////////////////////
//    Libraries    //
/////////////////////

#include <SPI.h>
#include <mcp2515.h>  // https://github.com/autowp/arduino-mcp2515 + https://github.com/watterott/Arduino-Libs/tree/master/digitalWriteFast
#include <EEPROM.h>

/////////////////////
//  Configuration  //
/////////////////////

#define CS_PIN_CAN0 10  //pin connected to NAC/cirocco CAN bus module
#define CS_PIN_CAN1 9   //pin connected to CVM CAN bus module
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

struct can_frame canAmbiance;  // Create a structure for sending CAN packet

struct can_frame canTheme;  // Create a structure for sending CAN packet

struct can_frame canLogo;  // Create a structure for sending not peugeot logo

struct can_frame canFakeIgnitionOn;  // Create a structure for sending not peugeot logo

struct can_frame canMsgSnd;
struct can_frame canMsgRcv;
bool IDChanged = false;

//Bool for avoiding duplicate push
bool SAMsend = false;
bool SAM_NAC = false;
bool lastSAM_NAC = false;

//Speed display
int progspeed = 0xff;
int Lastprogspeed = 0xff;
int NAVspeed = 0xff;
int CVMspeed = 0xff;
bool CVMreliabity = false;
bool LastCVMreliabity = false;
bool CustomCVMreliabity = false;
int RefNAVspeed = 0xff;
bool CVMendspeed = false;
bool boolendspeed = false;
unsigned long lastendspeed;  //timer for stop display of speed limit
int EndspeedDelay = 5000;

// AFIL
////////////////////////
// left line
bool fix_left_line = false;
int type_left_line = 0;
int distance_to_left_line = 0;
// right line
bool fix_right_line = false;
int type_right_line = 0;
int distance_to_right_line = 0;
int AFIL_distance_threshold;  //AFIL threshold (include car half width  (1300mm = 300m form line if car width  is 2m)

bool left = false;
bool right = false;
int vehicle_speed = 0;    // current vehicle speed
int speedthreshold = 53;  //thresold to activate afil light (include 3km/h correction)

bool Animation_done = false;  //set to true to not do animation
bool DriverDoor = false;


//Stop Start deletion
int rpm = 0;
bool EngineBeenStarted = false;
unsigned long EngineBeenStartedTimer = 0;
bool SSstatus = false;
bool SSdesactivationDone = false;
bool SSrequest = false;
unsigned long SSrequestTimer = 0;
bool ignition = false;

//MEM fix
bool MEMfixDoneLimiter = false;
bool MEMfixDoneCruise = false;
bool Lastingnition = false;
unsigned long ignitiontimer = 0;
struct can_frame LimitMsg;
struct can_frame CruiseMsg;

// Button for arduino
const int SAMBUTTON_PIN = A0;  // Arduino pin connected to button's pin
const int SAMLED_PIN = A3;     // Arduino pin connected to LED's pin
const int ECOBUTTON_PIN = A1;  // Arduino pin connected to button's pin
const int ECOLED_PIN = A4;     // Arduino pin connected to LED's pin
const int AASBUTTON_PIN = A2;  // Arduino pin connected to button's pin
const int AASLED_PIN = A5;     // Arduino pin connected to LED's pin

int ECOledState = LOW;      // the current state of LED
int ECOlastButtonState;     // the previous state of button
int ECOcurrentButtonState;  // the current state of button
int SAMledState = LOW;      // the current state of LED
int SAMlastButtonState;     // the previous state of button
int SAMcurrentButtonState;  // the current state of button
int AASledState = LOW;      // the current state of LED
int AASlastButtonState;     // the previous state of button
int AAScurrentButtonState;  // the current state of button
bool SAMstatus = false;
bool AASstatus = false;
bool AFILstatus = false;
unsigned long ButtonTimer = 0;
bool ECOsend = false;
bool AASsend = false;

//setting cruise/limit control
bool MemState = false;
bool LastMemState = false;
bool SetLimiter = false;
int Limiterspeed = 0xff;
bool CheckLimiter = false;
unsigned long CheckLimiterTimer = 0;
bool LimiterCkecked = false;
unsigned long MemStateTimer = 0;
bool MemSend = false;
unsigned long MemSendTimer = 0;      //timer between virtual mem press send and 1A9 sending
unsigned long MemSendWaitTimer = 0;  //timer between sending 19b/1db and sending mem press

bool FixMemLimit = false;
unsigned long FixMemLimitTimmer = 0;

bool SetCruise = false;
int Cruisespeed = 0xff;
bool CheckCruise = false;
unsigned long CheckCruiseTimer = 0;
bool CruiseCkecked = false;
bool FixMemCruise = false;
unsigned long FixMemCruiseTimmer = 0;
int RXXpaused = false;

bool LimitMode = false;
bool CruiseMode = false;
bool PauseSend = false;
//unsigned long PauseSendWaitTimer = 0; //timer between sending 1a9 (setting rxx value) and sending pause press
int DisplayMemLogo = 0;
bool Speedvalid = false;
bool PlusSend = false;
//unsigned long PlusSendWaitTimer = 0; //timer between double MEM press and sending + press to set any speed in cruise mode
bool PauseSendDisable = false;
bool WiperActive = false;

//ambiance
bool EscState = false;
bool LastEscState = false;
int ambiance = 0x0E;  //default value at startup, 0E= off, 8E=relax ambiance,  4E= boost ambiance
int theme = 0x01;     //default value at startup, 01= blue, 02=bronze
int Theme1A9Send = 0;
unsigned long ESCtimer = 0;

unsigned long aastimer = 0;
unsigned long ecotimer = 0;
bool vp2videotoogle = false;
bool vp2forcerear = false;
bool vp2forcefront = false;
bool VP2videoshowing = false;
bool ObstacleDetected = false;
bool LastObstacleDetected = false;
bool reargear = false;
bool lastreargear = false;



void setup() {
  ambiance = EEPROM.read(0);

  canAmbiance.can_id = 0x2E9;      // Assign the FMUX panel id
  canAmbiance.can_dlc = 4;         // Specify the length of the packet
  canAmbiance.data[0] = 0x01;      // Empty data
  canAmbiance.data[1] = ambiance;  // Empty data
  canAmbiance.data[2] = 0x58;      // Empty data
  canAmbiance.data[3] = 0x00;      // Empty data

  pinMode(ECOBUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  pinMode(ECOLED_PIN, OUTPUT);           // set arduino pin to output mode
  pinMode(SAMBUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  pinMode(SAMLED_PIN, OUTPUT);           // set arduino pin to output mode
  pinMode(AASBUTTON_PIN, INPUT_PULLUP);  // set arduino pin to input pull-up mode
  pinMode(AASLED_PIN, OUTPUT);           // set arduino pin to output mode

  digitalWrite(ECOLED_PIN, LOW);
  digitalWrite(SAMLED_PIN, LOW);
  digitalWrite(AASLED_PIN, LOW);

  ECOcurrentButtonState = digitalRead(ECOBUTTON_PIN);
  SAMcurrentButtonState = digitalRead(SAMBUTTON_PIN);
  AAScurrentButtonState = digitalRead(AASBUTTON_PIN);



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

  //Mem default Value


  LimitMsg.can_id = 0x19B;
  LimitMsg.can_dlc = 7;
  LimitMsg.data[0] = 0x32;
  LimitMsg.data[1] = 0x48;
  LimitMsg.data[2] = 0x53;
  LimitMsg.data[3] = 0x5D;
  LimitMsg.data[4] = 0x71;
  LimitMsg.data[5] = 0x85;
  LimitMsg.data[6] = 0x80;  //limiter value sorted ascending (but optional)


  CruiseMsg.can_id = 0x1DB;
  CruiseMsg.can_dlc = 7;
  CruiseMsg.data[0] = 0x32;
  CruiseMsg.data[1] = 0x48;
  CruiseMsg.data[2] = 0x53;
  CruiseMsg.data[3] = 0x5D;
  CruiseMsg.data[4] = 0x71;
  CruiseMsg.data[5] = 0x85;
  CruiseMsg.data[6] = 0x80;  //Cruise value sorted ascending (but optional)
}

void loop() {
  // CIROCCO
  if (CAN0.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {

    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    CAN1.sendMessage(&canMsgRcv);

    if (id == 0x217 && SAMsend) {  //217 received and something need to be send
      //Serial.println("217 received");
      canMsgSnd = canMsgRcv;  //copy frame
      canMsgSnd.data[3] = bitWrite(canMsgSnd.data[3], 3, SAMsend);
      SAMsend = false;
      // Serial.println("SAM struture written and SAMsend reset");
      CAN0.sendMessage(&canMsgSnd);
      //Serial.println("217 sent");
    }

    if (id == 0x2D1) {  //frame for SAM state (turn on cirocco line)
      SAMstatus = bitRead(canMsgRcv.data[0], 2);
      if (!SAMstatus && ignition) {
        digitalWrite(SAMLED_PIN, HIGH);  //turn on led
      } else {
        digitalWrite(SAMLED_PIN, LOW);  //turn off led
      }

      if (SAMstatus == 0) {  //SAM active
        canMsgSnd.can_id = 0x321;
        canMsgSnd.can_dlc = 5;
        canMsgSnd.data[0] = 0x0;
        canMsgSnd.data[1] = 0x0;
        canMsgSnd.data[2] = 0x0;
        canMsgSnd.data[3] = 0x0;
        canMsgSnd.data[4] = 0x0;
        CAN0.sendMessage(&canMsgSnd);  //send 0x321 frame to turn on indicator
      }
    }

    if (id == 0x227) {  // ID for BSI status (AAS, ESP,AFIL, StopStart)
      SSstatus = bitRead(canMsgRcv.data[3], 2);
      if (SSstatus && ignition) {
        digitalWrite(ECOLED_PIN, HIGH);  //turn on led
      } else {
        digitalWrite(ECOLED_PIN, LOW);  //turn off
      }

      AASstatus = bitRead(canMsgRcv.data[0], 6);
      if (AASstatus && ignition) {
        digitalWrite(AASLED_PIN, HIGH);  //turn on led
      } else {
        digitalWrite(AASLED_PIN, LOW);  //turn off
      }

      AFILstatus = bitRead(canMsgRcv.data[1], 4);

      //Serial.print("canMsgRcv.data[3] ");Serial.print(canMsgRcv.data[3]);Serial.print(" SSstatus is "); Serial.println(SSstatus);
    }
  }


  // Listen CAR MESSAGES
  if (CAN1.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (id == 0x236) {                      //ANIMATION
    Serial.println("ARRODASIJDIJSDJIOz");
      if (!Animation_done && DriverDoor) {  //5s timeout
        canMsgSnd = canMsgRcv;              //copy frame
        canMsgSnd.data[5] = bitWrite(canMsgSnd.data[5], 6, 1);
        CAN0.sendMessage(&canMsgSnd);
        Animation_done = true;
      }
    }

  // Jump peugeot Logo
  if(ignition == false) {
      // FAKE IGNITION ON
      canFakeIgnitionOn.data[0] = 0x88;

      // FAKE DARK MODE
      canLogo.data[3] = 0x35;

      CAN0.sendMessage(&canFakeIgnitionOn);
      CAN0.sendMessage(&canLogo);
  }
    if (id == 0x036 && ignition == false) {
      // FAKE IGNITION ON
      canFakeIgnitionOn.data[0] = 0x88;

      canLogo = canMsgRcv;

      // FAKE DARK MODE
      canLogo.data[3] = 0x35;

      CAN0.sendMessage(&canFakeIgnitionOn);
      CAN0.sendMessage(&canLogo);
    } else 
    if (id == 0x2E9) {
      canAmbiance.data[1] = ambiance;
      CAN0.sendMessage(&canAmbiance);
      canTheme = canMsgRcv;
    } else {
      CAN0.sendMessage(&canMsgRcv);
    }

    if (id == 0xF6) {  // If for turning indicator ignition
      // Serial.println(ignition);
      left = bitRead(canMsgRcv.data[7], 1);
      //Serial.print(" left: "); Serial.println(left);

      right = bitRead(canMsgRcv.data[7], 0);
      // Serial.print(" right: "); Serial.println(right);

      Lastingnition = ignition;
      ignition = bitRead(canMsgRcv.data[0], 3);
      if (ignition && !Lastingnition) { ignitiontimer = millis(); }  //ignition switched to ON
      if (!ignition) {
        canFakeIgnitionOn = canMsgRcv;
        for (int i = 0; i < canMsgRcv.can_dlc; i++) {  // print the data

          Serial.print(canMsgRcv.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println(" ");
        SSdesactivationDone = false;  //request a new SS desactivation  if ignition is off
        EngineBeenStarted = false;    // reset EngineBeenStarted (if ignition off engine can't be running) If not reseted, on "warm" start (arduino not powered off between 2 engine start) SS will deactivate when as soon as ignition is on
      } else {
        for (int i = 0; i < canMsgRcv.can_dlc; i++) {  // print the data

          Serial.print(canMsgRcv.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println(" ");
      }
    }

    if (id == 0x1A9) {                          //NAC message
                                                // Serial.println("basetheme");
      lastSAM_NAC = SAM_NAC;                    //store previous state
      SAM_NAC = bitRead(canMsgRcv.data[3], 5);  //SAM toogle read
      if (SAM_NAC && !lastSAM_NAC) {            //is pushed and wasnot pushed before
        SAMsend = true;
        //  Serial.println("SAMsend asked ");
      }

      if (!SSdesactivationDone) {                                                                 //do stuff to  deactivate SS at right time and check it worked
        if (EngineBeenStarted && !SSrequest && (millis() - EngineBeenStartedTimer > 5 * 1000)) {  //ask deactivation if conditions are meet and note time
          canMsgSnd = canMsgRcv;                                                                  //copy frame
          canMsgSnd.data[6] = bitWrite(canMsgSnd.data[6], 7, 1);                                  //D0(push)  50 not pushed  NAC toogle key for stop & start
          CAN1.sendMessage(&canMsgSnd);
          SSrequest = true;           //we note we asked
          SSrequestTimer = millis();  //time we asked
        }

        if (SSrequest && (millis() - SSrequestTimer > 1 * 1000)) {  //we check if it worked 1s after request
          SSrequest = false;                                        //cancel request because we are checking it (so if it did not worked we will ask again
          if (SSstatus) {                                           //if it worked
            SSdesactivationDone = true;                             //descativation of full loop
          }
        }
      }

      if (ECOsend || AASsend || vp2videotoogle) {  //something need to be send
        canMsgSnd = canMsgRcv;                     //copy frame
        canMsgSnd.data[6] = bitWrite(canMsgSnd.data[6], 7, ECOsend);
        ECOsend = false;  //write ECOsend value to frame
        canMsgSnd.data[3] = bitWrite(canMsgSnd.data[3], 2, AASsend);
        AASsend = false;  //write AASend value to frame
        canMsgSnd.data[7] = bitWrite(canMsgSnd.data[7], 7, vp2videotoogle);
        vp2videotoogle = false;  //write vp2videotoogle value to frame
        CAN1.sendMessage(&canMsgSnd);
      }

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

    if (id == 0x00E) {  //door state
      DriverDoor = bitRead(canMsgRcv.data[1], 6);
      //Serial.print("DriverDoor is  :  ");Serial.println(DriverDoor);
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