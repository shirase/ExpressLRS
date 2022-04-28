#if defined(PLATFORM_ESP32) && defined(USE_INNER_BLUETOOTH)

#include "bluetooth.h"
#include "config.h"
#include "telemetry_serial.h"
#include "BluetoothSerial.h"
#include "hwTimer.h"
#include "logging.h"

BluetoothSerial SerialBT;
bool SerialBTInit = false;

extern hwTimer hwTimer;

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    TelemetrySerial::start(&SerialBT);
  } 
  /*else if (event == ESP_SPP_SRV_STOP_EVT) {
    TelemetrySerial::stop();
  }*/
}

static int start()
{
    return 1000;
}

static int timeout()
{
  if (config.GetTelemetrySerialOut()) {
    if (!SerialBTInit) {
      SerialBTInit = true;
      hwTimer.stop();
      SerialBT.begin("ESP32");
      SerialBT.register_callback(callback);
      hwTimer.resume();
      return 1000;
    }
  } else {
    if (SerialBTInit) {
      SerialBTInit = false;
      hwTimer.stop();
      TelemetrySerial::stop();
      SerialBT.end();
      hwTimer.resume();
      return 1000;
    }
  }

  return 1000;
}

device_t Bluetooth_device = {
  .initialize = NULL,
  .start = start,
  .event = NULL,
  .timeout = timeout
};

#endif