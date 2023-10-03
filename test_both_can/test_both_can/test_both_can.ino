/*
DS4-cirocco-retrofit

New updated sketch see Cirocco_retrofit\Cirocco_retrofit_completev1.ino:



Sketch by Nico1080
This sketch was tested on a DS4, and should also work on any C4 gen 2 (hatchback, sedan, etc)

Look at my profile to see all modification I brought to the car:
https://www.drive2.ru/r/citroen/ds4/609447751477897832/

Many thanks to:
- Keryan (https://www.forum-peugeot.com/Forum/members/keryan.126947/) for code insipration
- Cesenate (https://www.drive2.ru/users/cesenate) for the HBA cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)
- Infizer for many items  (https://github.com/infizer91/can_extender  , https://www.drive2.ru/users/infizer/ , https://telecoder.ru/index.php )
And all I have forgotten

Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it)

The function of this sketch:
- Make SAM (Surveillance Angles Mort, blind spots monitor) NAC toogle switch work
- AAS (Aide Au Stationement, parking sensor) need BSI to be coded to listen to NAC and not cluster anymore (some code could be wrote to have both working)
- Speed limit display from CVM and NAV data (see comment)
- Fix SAM light on cirocco (frame 0x321)  (Thanks to Infizer)
- Fix animation on cirocco (frame 0x236)  (Thanks to Infizer)
- Display green/orange line on cirrocco (AFIL and LKA need to be activated in cirocco) (Thanks to Infizer)
- Make AFIL indicator flash when lines are being crossed without turn indicator (see comment)(Thanks to Infizer)
- Deactivate Stop & Start 5s after engine start
- Fix MEM returning to default value for all setting (40km/h) (Thanks to Infizer)
- Double pressing MEM will set and activate cruise control/ speed limiter to a corrected speed value (offset+ rain, see comment)
- Physical connection with AAS/ECO/SAM button/led to make them work again and add other function (see comment)
- Display front/rear camera (VisioPark2) by pressing AAS/ECO button (see comment)
- Display front camera(VisioPark2) when front obstacle is detected by AAS (see comment)
- Change Cirocco ambiance/theme by pressing the ESC button on the wheel (see comment)

For speed limit display it follow this logic:
- show end of speed-limit sign for 3s when it is read by CVM
- show CVM speed when it is reliable (red sign)
- keep showing CVM speed in red sign after CVM loose reliability. Only if NAV speed have the same value and has not changed since it lost CVM reliability (CVM reliability is easily lost so I "extend" it)
- show NAV speed (grey sign)
- if no NAV speed is available it display last read sign (grey sign and remove 1km/h. Example last sign=70km/h display 69 in grey sign)
- If no previous sign (and no nav speed) it display nothing

For AFIL  (Alerte Franchissement Involontaire de Ligne, line detection):
- Display green orange line as soon as they are detected at any time (require LKA activated in cirocco)
- Flash AFIL light when one line become orange and if one(and not 2) turning indicator is ON  and when speed is over 50km/h (require AFIL activated in Cirocco)
- When left/right lines are both detected: margin calculation for AFIL threshold (large road= big margin small road= small margin) default value is 1300.

For Stop and start deactivation:
- It require NAC toggle key working for stop and start.  BSI need to be coded to telematic in engine menu (Type d'acquisition du contacteur stop and start) , other choices are BSI (original value on my DS4) and cluster.
- If stop & start is not already deactivated, it will send request (Id 1A9) to do it 5s after engine start (rpm>500) and check if it worked (if it didn't, it will try again)

For MEM FIX:
- sketch  will send 19b(limit) & 1DB (cruise) on CAN-DIV after ignition with speed value (BSI will send the same frame several times (6 for limit, 3 for cruise) sorted in ascending order
- Some code could be written to check if driver changed setting and store new values inside EEPROM

For setting cruise control/ speed limiter:
- the sketch will program the speed value into the MEM setting and emulate the needed button press (pause, MEM etc) to activate it.
- The wheels button need only to be set the cruise or limiter mode.
- The set speed is offseted by 2 or 3km/h (between 70/79 and above 80)
- If wiper are active (auto mode) the speed will also be decrease by 20 for 130 and 10 for 90/110 (French speed limit are lowered when raining)

For AAS/ECO/SAM button connection: 
- Button pin need to connected to arduino input pin (no need for resistor as INPUT_PULLUP is activated
- When button are pushed sketch will send request on ID 1A9 (ECO and AAS, NAC emulation, BSI need to be coded to listen to NAC) or 217 (SAM, Cluster emulation as BSI can not be coded for other source)
- For AAS/ECO/SAM led connection: a level shifter is required (led use 12v logic and arduino is only 5v) I used a UDN2981.
- Sketch listen to ID 227 (ECO and AAS) and 2D1 (SAM). It will turn on the led only if ignition is on.
- For button backlight I kept the original wire from car(265 generated by BSI, connected on button pin 4).  Some code could be written to integrate it on arduino and avoid extra wiring
- See https://www.drive2.ru/l/633915458608714452/

For VisioPark2 the sketch will:
- Show front video when front obstacle is detected by AAS
- Show/hide front video when AAS button is short pressed (<800ms)
- Show/hide rear video when ECO button is short pressed (<800ms)
- Long press (>800ms) on AAS/ECO button will make normal operation (ID 1A9 ECO and AAS NAC emulation)
- In any case video will automatically:
   - 	Disappear when going over 25km/h (setting built inside NAC, no way around it)
   - 	Rear video will show when rear gear is engaged 	


For changing Cirocco ambiance/theme:
- A short press on ESC button will toggle ambiance between: No ambiance/relax/boost ambiance,  This setting will disappear after car is shut off.  (It is a shortcut for Icockpit amplify in NAC menu)
- A long press (>1sec) on ESC button will change theme on Cirocco (blue or bronze) without changing NAC theme. However when restarting the car, NAC will change his theme to match the cirocco theme.

After many try on my car I figured the following logic:
- Cirocco have 2 themes activated: 1=Blue and 2=Bronze
- NAC have 3 themes: 1=purple (AMETHYST), 2=red (RUBY), 3=yellow(GOLD)  (+unlisted NACblue theme)

When changing theme on NAC it also change Cirocco theme: 1purple-->1blue, 2red-->2bronze and 3Yellow-->2bronze.

When changing theme on Cirocco with ESC button, it will change NAC theme after restart: 1blue--> 1purple and 2bronze-->2red  (3yellow is not accessible)


If I try activating more themes in cirocco/NAC I have weird behaviour: NAC reboot to unlisted NACblue theme, and NAC theme selection menu disappear.cluster CAN ID from DS5  (ID 0x217 Byte 3, value 0x80)


Feel free to use this code and improve it. In return I only ask that you share your improvement (and obviously that you don't sell it or try to make money from it)
*/

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

struct can_frame canTheme; // Create a structure for sending CAN packet

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

  canAmbiance.can_id = 0x2E9;  // Assign the FMUX panel id
  canAmbiance.can_dlc = 4;     // Specify the length of the packet
  canAmbiance.data[0] = 0x01;  // Empty data
  canAmbiance.data[1] = ambiance;  // Empty data
  canAmbiance.data[2] = 0x58;  // Empty data
  canAmbiance.data[3] = 0x00;  // Empty data

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
  // Receive CAN messages from the car
  if (CAN0.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {

    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    CAN1.sendMessage(&canMsgRcv);

    // Serial.println("CANN0");
    // Serial.println(id);

    // if (debugCAN0) {
    //   Serial.print("FRAME:ID=0x");
    //   Serial.print(id, HEX);
    //   Serial.print(":LEN=");
    //   Serial.print(len);

    //   char tmp[3];
    //   for (int i = 0; i < len; i++) {
    //     Serial.print(":");

    //     snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);

    //     Serial.print(tmp);
    //   }

    //   Serial.println();
    // }

    // if (id == 0x168) {  //wiper state
    //   WiperActive = bitRead(canMsgRcv.data[1], 3);
    //   //Serial.print("WiperActive is  :  ");Serial.println(WiperActive);
    // }

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


  // Listen on CVM CAN BUS
  if (CAN1.readMessage(&canMsgRcv) == MCP2515::ERROR_OK) {
    int id = canMsgRcv.can_id;
    int len = canMsgRcv.can_dlc;

    if (id == 0x2E9) {
      canAmbiance.data[1] = ambiance;
      CAN0.sendMessage(&canAmbiance);
      canTheme = canMsgRcv;
    } else {
      CAN0.sendMessage(&canMsgRcv);
    }


    if (id == 0x1A9) {                          //NAC message
    Serial.println("basetheme");
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

      if (Theme1A9Send >= 1)
      {
        Theme1A9Send = Theme1A9Send - 1;
        canTheme = canMsgRcv; // copy frame
        canTheme.data[6] = bitWrite(canTheme.data[6], 5, 1);
        CAN0.sendMessage(&canTheme);
        CAN1.sendMessage(&canTheme);
        Serial.print("Theme1A9sent (70), new number is:  ");
        Serial.println(Theme1A9Send, DEC);
      }
    }



    // Serial.println("CANN1");
    // Serial.println(id);

    // if (debugCAN1) {
    //   Serial.print("FRAME:ID=0x");
    //   Serial.print(id, HEX);
    //   Serial.print(":LEN=");
    //   Serial.print(len);

    //   char tmp[3];
    //   for (int i = 0; i < len; i++) {
    //     Serial.print(":");

    //     snprintf(tmp, 3, "%02X", canMsgRcv.data[i]);

    //     Serial.print(tmp);
    //   }
    //   Serial.println();
    // }

    if (id == 0x00E) {  //door state
      DriverDoor = bitRead(canMsgRcv.data[1], 6);
      //Serial.print("DriverDoor is  :  ");Serial.println(DriverDoor);
    }

    if (id == 0xA2)
    { // VCI state lower right (ESC)
      LastEscState = EscState;
      EscState = bitRead(canMsgRcv.data[1], 4); // ESC key
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

          // canTheme = canMsgRcv;
          // copy frame
          canTheme.data[0] = ((canTheme.data[0] & 0xFC) | (theme & 0x03)); // theme value is only in bit0&1 =0x03 mask  0xFC is reseting SndData -->  DDDDDD00 | 000000TT   D=original data, T=theme
                                                                           //  if ((canTheme.data[0] != canTheme.data[0]) || (canTheme.data[1] != canTheme.data[1])) {  //diffrent value received from NAC, we need to request the new value
          CAN1.sendMessage(&canTheme);
          CAN0.sendMessage(&canTheme);

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
  }
}