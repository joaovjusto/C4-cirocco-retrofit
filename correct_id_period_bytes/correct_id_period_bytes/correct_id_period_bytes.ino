#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;

struct can_frame canMsg;
int cntr = 0;
unsigned long oldTime = 0;

MCP2515 mcp2515(10);


void setup() {
  canMsg1.can_id = 0x2E9;
  canMsg1.can_dlc = 4;
  canMsg1.data[0] = 01;
  canMsg1.data[1] = 0x4E ;
  canMsg1.data[2] = 58;
  canMsg1.data[3] = 00;

  while (!Serial);
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial.println("Example: Write to CAN");
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    for (int x = 0; x < 200; x++) {          
      mcp2515.sendMessage(&canMsg1);
      cntr++;
    }
  }

  if ((millis() - oldTime) > 1000) {
    oldTime = millis();
    Serial.print(cntr);
    Serial.println(" msg/sec");
    cntr = 0;
  }
}
