#include "CanBus.h"

CanBus::CanBus(int csPin, bool debug) : can(csPin), debug(debug) {}

void CanBus::init(uint8_t speed, uint8_t freq) {
    can.reset();
    can.setBitrate(speed, freq);
    can.setNormalMode();
}

bool CanBus::readMessage(struct can_frame *message) {
    return (can.readMessage(message) == MCP2515::ERROR_OK);
}

void CanBus::sendMessage(struct can_frame *message) {
    can.sendMessage(message);
    if (debug) {
        Serial.println("Message sent!");
    }
}
