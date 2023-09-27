#include <SPI.h>
#include <mcp2515.h>

struct can_frame canAmbiance;
MCP2515 mcp2515(10);


void setup() {
  canAmbiance.can_id = 0x2E9;
  canAmbiance.can_dlc = 4;
  canAmbiance.data[0] = 0xFF;
  canAmbiance.data[1] = (0x4E & 0xC0);
  canAmbiance.data[2] = 0xFF;
  canAmbiance.data[3] = 0xFF;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.clearInterrupts();
  mcp2515.setNormalMode();
  Serial.println("Example: Write to CAN");
}

void loop() {
  mcp2515.clearInterrupts();
  mcp2515.clearTXInterrupts();
  mcp2515.clearMERR();

  mcp2515.sendMessage(&canAmbiance);

  Serial.println("Messages sent");

  // delay(5);
}
