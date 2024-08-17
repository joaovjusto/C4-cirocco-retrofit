#ifndef CANBUS_H
#define CANBUS_H

#include <SPI.h>
#include <mcp2515.h>

class CanBus {
  private:
    MCP2515 can;
    bool debug;
  public:
    CanBus(int csPin, bool debug = false);
    void init(uint8_t speed, uint8_t freq);
    bool readMessage(struct can_frame *message);
    void sendMessage(struct can_frame *message);
};

#endif
