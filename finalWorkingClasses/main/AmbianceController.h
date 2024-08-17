#ifndef AMBIANCECONTROLLER_H
#define AMBIANCECONTROLLER_H

#include <EEPROM.h>
#include "CanBus.h"

class AmbianceController {
  private:
    int ambiance;
    int theme;
    struct can_frame canAmbiance;
    unsigned long ESCtimer;
    CanBus *can0;
    CanBus *can1;

  public:
    AmbianceController(CanBus *can0, CanBus *can1);
    void updateAmbiance();
    void handleESCButton(bool escState, bool lastEscState);
    void switchTheme();
    void switchAmbiance();
};

#endif
