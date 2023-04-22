#include <Arduino.h>
#include <SPI.h>
#include <MotorModule.hpp>
#include <mcp2515.h>


#define CAN0_INT 3  // Set CAN INT pin

MCP2515 CAN0(53);
can_frame frame;
can_frame rx_frame;

long unsigned int rxId;
unsigned char len = 0;

MotorStruct motor;

volatile bool interrupt = false;

void irqHandler() {
    interrupt = true;
}


void setup()
{
  Serial.begin(9600);

  // Initialise CAN
  CAN0.reset();
  CAN0.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  CAN0.setNormalMode();

  frame.can_id = 0x00;
  frame.can_dlc = 8;

  pinMode(CAN0_INT, INPUT);   // Set interrupt pin to be an input

  enable_motor(&motor, CAN0);     // Go into motor mode

  attachInterrupt(digitalPinToInterrupt(CAN0_INT), irqHandler, FALLING);

}

void loop() {

  // motor.control.p_des = 3;
  // motor.control.kp = 20;
  // motor.control.kd = 2;
  // motor.control.v_des = 2;
  // motor.control.i_ff = 0;
  // motor.control.id = 0;
  // pack_cmd(&motor);

  enable_motor(&motor,CAN0);

  // Add data to CAN frame to be sent
  for (int i = 0; i < 8; i++) {
    frame.data[i] = motor.txMsg[i];
    // Serial.print(frame.data[i], BIN);
    // Serial.print("  ");
    // Serial.println(motor.txMsg[i], BIN);
  }

  CAN0.sendMessage(&frame);

  if (interrupt) {
    interrupt = false;

    if (CAN0.readMessage(&rx_frame) == MCP2515::ERROR_OK) {
      motor.state.id = rx_frame.can_id;
      for (int i = 0; i < 6; i++) {
        motor.rxMsg[i] = rx_frame.data[i];
      }
      unpack_reply(&motor);
      Serial.println(motor.state.position);
    }
  }

  delay(1);

  // byte sndStat = CAN0.sendMsgBuf(0x0, 0, 8, motor.txMsg);

  // if(sndStat == CAN_OK){
  //   Serial.println("Message Sent Successfully!");
  // } else {
  //   Serial.println("Error Sending Message...");
  // }
}