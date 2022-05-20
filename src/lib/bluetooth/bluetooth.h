#pragma once

#include "common.h"
#include "device.h"
#include "msp.h"
#include "CRSF.h"
#include "BluetoothSerial.h"

extern BluetoothSerial SerialBT;
extern bool SerialBTInit;
extern bool MSPSerialReadyToSend;
extern device_t Bluetooth_device;
extern MSP msp;
extern CRSF crsf;
