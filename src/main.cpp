#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <MotorModule.hpp>


#define CAN0_INT 42  // Set CAN INT pin

MCP_CAN CAN0(53);     // Create CAN object and set its CS pin
long unsigned int rxId;
unsigned char len = 0;

MotorStruct motor;

void setup()
{
  Serial.begin(9600);

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
    while(1);
  } 

  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  pinMode(CAN0_INT, INPUT);   // Set interrupt pin to be an input


  // Test motor turn
  enable_motor(&motor, CAN0);


}

void loop() {
  if(!digitalRead(CAN0_INT))                      // If CAN0_INT pin is low, read receive buffer
  {
    Serial.println("Rx");
    CAN0.readMsgBuf(&rxId, &len, motor.rxMsg);    // Read data: len = data length, buf = data byte(s)
    unpack_reply(&motor);
  }

  Serial.print("Position: ");
  Serial.println(motor.state.position);

  motor.control.p_des = 3;
  pack_cmd(&motor);
  CAN0.sendMsgBuf(0x0, 0, 8, motor.txMsg);

}