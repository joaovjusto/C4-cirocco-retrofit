#include "AmbianceController.h"

AmbianceController::AmbianceController(CanBus *can0, CanBus *can1)
  : ambiance(0x0E), theme(0x01), ESCtimer(0), can0(can0), can1(can1) {
    canAmbiance.can_id = 0x2E9;
    canAmbiance.can_dlc = 4;
    canAmbiance.data[0] = theme;
    canAmbiance.data[1] = ambiance;
    canAmbiance.data[2] = 0x58;
    canAmbiance.data[3] = 0x00;
}

void AmbianceController::updateAmbiance() {
    canAmbiance.data[0] = theme;
    canAmbiance.data[1] = ambiance;
    can0->sendMessage(&canAmbiance);
}

void AmbianceController::handleESCButton(bool escState, bool lastEscState) {
    if (escState && !lastEscState) {
        ESCtimer = millis();
    }
    if (!escState && lastEscState) {
        if ((millis() - ESCtimer) >= 1000) {
            switchTheme();
        } else {
            switchAmbiance();
        }
    }
}

void AmbianceController::switchTheme() {
    theme = (theme == 0x01) ? 0x02 : 0x01;
    canAmbiance.data[0] = theme;
    can1->sendMessage(&canAmbiance);
    can0->sendMessage(&canAmbiance);
}

void AmbianceController::switchAmbiance() {
    switch (ambiance) {
        case 0x0E:
            ambiance = 0x4E;
            break;
        case 0x4E:
            ambiance = 0x8E;
            break;
        case 0x8E:
            ambiance = 0x0E;
            break;
        default:
            ambiance = 0x0E;
            break;
    }
    EEPROM.update(0, ambiance);
}
