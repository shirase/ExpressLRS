#include "bluetooth.h"

#if defined(PLATFORM_ESP32) && defined(USE_INNER_BLUETOOTH)

BluetoothSerial SerialBT;

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
    SerialBT.begin("ESP32");
    SerialBT.register_callback(callback);
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
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

static int event()
{
    if (connectionState == bluetooth) {
        return DURATION_IMMEDIATELY;
    }
    return DURATION_IMMEDIATELY;
}

device_t Bluetooth_device = {
  .initialize = NULL,
  .start = start,
  .event = event,
  .timeout = timeout
};

#endif