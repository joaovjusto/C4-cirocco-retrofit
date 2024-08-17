#include "CanBus.h"
#include "MessageHandler.h"
#include "AmbianceController.h"

#define CS_PIN_CAN0 10
#define CS_PIN_CAN1 9
#define SERIAL_SPEED 115200
#define CAN_SPEED CAN_125KBPS
#define CAN_FREQ MCP_8MHZ

CanBus can0(CS_PIN_CAN0, true);
CanBus can1(CS_PIN_CAN1, false);
AmbianceController ambianceController(&can0, &can1);
MessageHandler messageHandler(&can0, &can1, &ambianceController);

void setup() {
    Serial.begin(SERIAL_SPEED);
    can0.init(CAN_SPEED, CAN_FREQ);
    can1.init(CAN_SPEED, CAN_FREQ);
    ambianceController.updateAmbiance();
}

void loop() {
    messageHandler.handleMessages();
}
