#include "ibus.h"
#include "targets.h"

static uint8_t ibus[IBUS_BUFFSIZE] = { 0, };
static uint8_t ibusChannelOffset = 2;
static uint32_t ibusChannelData[IBUS_MAX_CHANNEL];
static uint8_t rxBytesToIgnore;
static uint8_t ibusFrameSize;

static bool isValidIa6bIbusPacketLength(uint8_t length)
{
    return (length == IBUS_TELEMETRY_PACKET_LENGTH) || (length == IBUS_SERIAL_RX_PACKET_LENGTH);
}

static void updateChannelData(void) {
    uint8_t i;
    uint8_t offset;
    
    for (i = 0, offset = ibusChannelOffset; i < IBUS_MAX_SLOTS; i++, offset += 2) {
        ibusChannelData[i] = ibus[offset] + ((ibus[offset + 1] & 0x0F) << 8);
    }
    //latest IBUS recievers are using prviously not used 4 bits on every channel to incresse total channel count
    for (i = IBUS_MAX_SLOTS, offset = ibusChannelOffset + 1; i < IBUS_MAX_CHANNEL; i++, offset += 6) {
        ibusChannelData[i] = ((ibus[offset] & 0xF0) >> 4) | (ibus[offset + 2] & 0xF0) | ((ibus[offset + 4] & 0xF0) << 4);
    }
}

uint16_t ibusCalculateChecksum(const uint8_t *ibusPacket, size_t packetLength)
{
    uint16_t checksum = 0xFFFF;
    for (size_t i = 0; i < packetLength - IBUS_CHECKSUM_SIZE; i++) {
        checksum -= ibusPacket[i];
    }

    return checksum;
}

bool ibusIsChecksumOkIa6b(const uint8_t *ibusPacket, const uint8_t length)
{
    uint16_t calculatedChecksum = ibusCalculateChecksum(ibusPacket, length);

    // Note that there's a byte order swap to little endian here
    return (calculatedChecksum >> 8) == ibusPacket[length - 1]
           && (calculatedChecksum & 0xFF) == ibusPacket[length - 2];
}

uint16_t ibusReadRawRC(uint8_t chan)
{
    return ibusChannelData[chan];
}

bool ibusReceivePacket(uint8_t c)
{
    uint32_t ibusTime;
    static uint32_t ibusTimeLast;
    static uint8_t ibusFramePosition;

    ibusTime = micros();

    if ((ibusTime - ibusTimeLast) > IBUS_FRAME_GAP) {
        ibusFramePosition = 0;
        rxBytesToIgnore = 0;
    } else if (rxBytesToIgnore) {
        rxBytesToIgnore--;
        return false;
    }

    ibusTimeLast = ibusTime;

    if (ibusFramePosition == 0) {
        if (isValidIa6bIbusPacketLength(c)) {
            ibusFrameSize = c;
        } else {
            return false;
        }
    }

    ibus[ibusFramePosition] = c;

    if (ibusFramePosition == ibusFrameSize - 1) {
        ibusFramePosition = 0;

        if (ibusIsChecksumOkIa6b(ibus, ibusFrameSize)) {
            updateChannelData();
            return true;
        }
    } else {
        ibusFramePosition++;
    }

    return false;
}