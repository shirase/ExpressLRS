#include "bluetooth.h"

#if defined(PLATFORM_ESP32) && defined(USE_INNER_BLUETOOTH)

BluetoothSerial SerialBT;
bool SerialBTInit = false;
bool MSPSerialReadyToSend = false;

static void ProcessMSPPacket(mspPacket_t *packet)
{
    crsf.AddMspMessage(packet);
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    CRSF::PortSecondary = &SerialBT;
  } else
  if (event == ESP_SPP_CLOSE_EVT) {
    CRSF::PortSecondary = NULL;
  }
}

static int start()
{
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    if (MSPSerialReadyToSend) {
      MSPSerialReadyToSend = false;
      if (!SerialBTInit) {
        SerialBTInit = true;
        SerialBT.begin("ESP32");
        SerialBT.register_callback(callback);
      }
      return 1000;
    }

    if (!SerialBTInit) {
      return 1000;
    }

    if (SerialBT.available())
    {
        if (msp.processReceivedByte(SerialBT.read()))
        {
            // Finished processing a complete packet
            ProcessMSPPacket(msp.getReceivedPacket());
            msp.markPacketReceived();
        }
    }

    return DURATION_IMMEDIATELY;
}

device_t Bluetooth_device = {
  .initialize = NULL,
  .start = start,
  .event = NULL,
  .timeout = timeout
};

#endif