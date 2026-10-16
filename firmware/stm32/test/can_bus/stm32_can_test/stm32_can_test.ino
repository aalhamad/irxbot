#include <STM32_CAN.h>

STM32_CAN Can(CAN1, DEF);
static CAN_message_t txMsg;

#define CAN_PACKET_SET_RPM 3
#define VESC_ID 1

void setup() {
  Serial.begin(115200);
  Can.begin();
  Can.setBaudRate(500000);
  Serial.println("CAN_INIT_OK");
}

void loop() {
  int32_t rpm = 0;  // 0 RPM = safe, won't spin the motor

  txMsg.id = 0x7FF000;
  txMsg.flags.extended = 1;   // VESC uses extended (29-bit) IDs
  txMsg.len = 4;
  txMsg.buf[0] = (rpm >> 24) & 0xFF;
  txMsg.buf[1] = (rpm >> 16) & 0xFF;
  txMsg.buf[2] = (rpm >> 8) & 0xFF;
  txMsg.buf[3] = rpm & 0xFF;

  if (Can.write(txMsg)) {
    Serial.println("CAN_TX_OK");
  } else {
    Serial.println("CAN_TX_FAIL");
  }
  delay(50);  // match your firmware's 50ms send rate
}