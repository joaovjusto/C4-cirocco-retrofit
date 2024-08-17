#include "MessageHandler.h"

MessageHandler::MessageHandler(CanBus *can0, CanBus *can1, AmbianceController *ambianceController)
  : can0(can0), can1(can1), ambianceController(ambianceController),
    ignition(false), Lastingnition(false), ignitionTimer(0),
    escState(false), lastEscState(false) {}

void MessageHandler::handleMessages() {
    struct can_frame canMsgRcv;
    if (can0->readMessage(&canMsgRcv)) {
        handleCan0Message(canMsgRcv);
    }
    if (can1->readMessage(&canMsgRcv)) {
        handleCan1Message(canMsgRcv);
    }
}

void MessageHandler::handleCan0Message(struct can_frame message) {
    can1->sendMessage(&message);

    if (!ignition && Lastingnition) {
        can0->sendMessage(&canOff);
    }
}

void MessageHandler::handleCan1Message(struct can_frame message) {
    int id = message.can_id;

    if (!ignition) {
        if (id == 0x128) {
            canOff = message;
            memset(canOff.data, 0, sizeof(canOff.data));
            can0->sendMessage(&canOff);
        } else {
            canFakeIgnitionOn.data[0] = 0x88;
            can0->sendMessage(&message);
        }
    } else {
        // Se o ID for 0x2E9, atualiza o ambiente
        if (id == 0x2E9) {
            ambianceController->updateAmbiance();
        } else {
            can0->sendMessage(&message);
        }
    }

    // Handle ignition state (ID 0xF6)
    if (id == 0xF6) {
        Lastingnition = ignition;
        ignition = bitRead(message.data[0], 3);
        if (ignition && !Lastingnition) {
            ignitionTimer = millis();
        } else if (!ignition) {
            can0->sendMessage(&canOff);
        }
    }

    // Handle theme update (ID 0x1A9)
    if (id == 0x1A9) {
        ambianceController->switchTheme();
    }

    // Handle ESC button and ambiance update (ID 0xA2)
    if (id == 0xA2) {
        lastEscState = escState;
        escState = bitRead(message.data[1], 4);  // ESC button is in bit 4
        ambianceController->handleESCButton(escState, lastEscState);
    }
}
