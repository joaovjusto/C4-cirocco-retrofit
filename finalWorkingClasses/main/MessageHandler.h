#ifndef MESSAGEHANDLER_H
#define MESSAGEHANDLER_H

#include "CanBus.h"
#include "AmbianceController.h"

class MessageHandler {
  private:
    CanBus *can0;
    CanBus *can1;
    AmbianceController *ambianceController;
    struct can_frame canOff;
    struct can_frame canFakeIgnitionOn;
    bool ignition;
    bool Lastingnition;
    bool escState;
    bool lastEscState;
    unsigned long ignitionTimer;

  public:
    MessageHandler(CanBus *can0, CanBus *can1, AmbianceController *ambianceController);
    void handleMessages();
    void handleCan0Message(struct can_frame message);
    void handleCan1Message(struct can_frame message);
};

#endif
