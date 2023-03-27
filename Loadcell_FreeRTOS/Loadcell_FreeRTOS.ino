//Library for RTOS
// #if CONFIG_FREERTOS_UNICORE
// #define ARDUINO_RUNNING_CORE 0
// #else
// #define ARDUINO_RUNNING_CORE 1
// #endif

TaskHandle_t task_dxl, task_input;

void taskFunction1(void* pvParameters);
void taskFunction2(void* pvParameters);

//Library for Dynamixel Slave and HX711
#include <Dynamixel2Arduino.h>
#include "HX711.h"
#define DXL_SERIAL Serial2
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 4;

//Difine id each loadcell
HX711 cell1, cell2, cell3, cell4;

DYNAMIXEL::SerialPortHandler dxl_port(DXL_SERIAL, DXL_DIR_PIN);
// Create a Slave object to communicate with the master.
const float DXL_PROTOCOL_VER_1_0 = 1.0;
const float DXL_PROTOCOL_VER_2_0 = 2.0;
const uint16_t DXL_MODEL_NUM = 0x0140;  // Modify it to what you want.
DYNAMIXEL::Slave dxl(dxl_port, DXL_MODEL_NUM);

// Declare the address of the Slave control item you want
// to register and the variable (size is also important)
// to store its data.

// Read Address
uint16_t addr_force_1 = 66;
uint16_t addr_force_2 = 68;
uint16_t addr_force_3 = 70;
uint16_t addr_force_4 = 72;

// Write Address
uint8_t Reset_addr = 80;

//Address for LOADCELL
uint16_t dxl_data[10];

//Declared float for each loadcell value
// float q1;
// float q2;
// float q3;
// float q4;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  Serial.begin(115200);
  disableCore0WDT();
  disableCore1WDT();
  
  xTaskCreatePinnedToCore(taskFunction1, "Task1", 65536, NULL, 3, NULL,   0);
  xTaskCreatePinnedToCore(taskFunction2, "Task2", 65536, NULL, 1, NULL,   1);
}

void loop() {
  // put your main code here, to run repeatedly:
}

void taskFunction1(void* pvParameters) {
  (void)pvParameters;
  // Speed setting for communication (not necessary for USB)
  dxl_port.begin(1000000);
  //Setup Dynamixel product ID
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER_1_0);
  dxl.setFirmwareVersion(1);
  dxl.setID(23);

  dxl.addControlItem(addr_force_1, dxl_data[0]);
  dxl.addControlItem(addr_force_2, dxl_data[1]);
  dxl.addControlItem(addr_force_3, dxl_data[2]);
  dxl.addControlItem(addr_force_4, dxl_data[3]);
  dxl.addControlItem(Reset_addr, dxl_data[4]);

  dxl.setReadCallbackFunc(read_callback_func);
  dxl.setWriteCallbackFunc(write_callback_func);

  for (;;) {
    if (dxl.processPacket() == false) {
      DEBUG_SERIAL.print("Last lib err code: ");
      DEBUG_SERIAL.print(dxl.getLastLibErrCode());
      DEBUG_SERIAL.print(", ");
      DEBUG_SERIAL.print("Last status packet err code: ");
      DEBUG_SERIAL.println(dxl.getLastStatusPacketError());
      DEBUG_SERIAL.println();
    }
  }
}

void read_callback_func(uint16_t item_addr, uint8_t& dxl_err_code, void* arg) {
  (void)dxl_err_code, (void)arg;

  if (item_addr == addr_force_1) {
    dxl_data[0];
  }

  else if (item_addr == addr_force_2) {
    dxl_data[1];
  }

  else if (item_addr == addr_force_3) {
    dxl_data[2];
  }

  else if (item_addr == addr_force_4) {
    dxl_data[3];
  }
}

void write_callback_func(uint16_t item_addr, uint8_t& dxl_err_code, void* arg) {
  (void)dxl_err_code, (void)arg;

  if (item_addr == Reset_addr) {
    if (dxl_data[4] == 1) {
      ESP.restart();
    }
  }
}

void taskFunction2(void* pvParameters) {
  (void)pvParameters;
  //Setup pin Dout&SCK for each loadcell
  cell1.begin(14, 12);
  cell2.begin(26, 27);
  cell3.begin(32, 33);
  cell4.begin(23, 22);

  // define callibration scale for each loadcell
  cell1.set_scale(69.8);  // this value is obtained by dividing the calibration value with the known weight;
  cell2.set_scale(-68.5);
  cell3.set_scale(-68.5);
  cell4.set_scale(67.89);

  //Tare the loadcell to 0 gram
  cell1.tare();  //
  cell2.tare();
  cell3.tare();
  cell4.tare();

  for (;;) {
    // dxl_data[0] = random(200);
    // dxl_data[1] = random(100);
    // dxl_data[2] = random(300);
    // dxl_data[3] = random(200);
    dxl_data[0] = cell1.get_units() + 1000;
    dxl_data[1] = cell2.get_units() + 1000;
    dxl_data[2] = cell3.get_units() + 1000;
    dxl_data[3] = cell4.get_units() + 1000;

    // Serial.print(dxl_data[0]);
    // Serial.print(" || ");
    // Serial.print(dxl_data[1]);
    // Serial.print(" || ");
    // Serial.print(dxl_data[2]);
    // Serial.print(" || ");
    // Serial.println(dxl_data[3]);

    // Serial.println(dxl_data[0]);
    // DEBUG_SERIAL.print("1");
    // DEBUG_SERIAL.print("\t");  //DEBUG_SERIAL.print(q2);  DEBUG_SERIAL.print("\t");
    // //  DEBUG_SERIAL.print(q3); DEBUG_SERIAL.print("\t"); DEBUG_SERIAL.print(q4);  DEBUG_SERIAL.print("\t");
    // //  DEBUG_SERIAL.print(s);  DEBUG_SERIAL.print("\t");
    // DEBUG_SERIAL.println("");
  }
}
