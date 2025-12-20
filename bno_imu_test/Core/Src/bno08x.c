// bno08x.c
#include "bno08x.h"
#include <string.h>

// ---- low-level helpers ---------------------------------------------------

static HAL_StatusTypeDef bno08x_i2c_write(BNO08x_Handle_t *dev,
                                          uint8_t *data, uint16_t len)
{
    return HAL_I2C_Master_Transmit(dev->hi2c, BNO08X_I2C_ADDR, data, len, 100);
}

static HAL_StatusTypeDef bno08x_i2c_read(BNO08x_Handle_t *dev,
                                         uint8_t *data, uint16_t len)
{
    return HAL_I2C_Master_Receive(dev->hi2c, BNO08X_I2C_ADDR, data, len, 100);
}

// Send one SHTP packet on a given channel
static bool bno08x_sendPacket(BNO08x_Handle_t *dev,
                              uint8_t channel,
                              const uint8_t *payload,
                              uint16_t payloadLen)
{
    if (payloadLen + 4 > BNO08X_MAX_PACKET_LEN) return false;

    uint8_t buf[BNO08X_MAX_PACKET_LEN];

    uint16_t totalLen = payloadLen + 4;
    buf[0] = (uint8_t)(totalLen & 0xFF);
    buf[1] = (uint8_t)((totalLen >> 8) & 0x7F);  // MSB, cont. flag = 0
    buf[2] = channel;
    buf[3] = dev->seq[channel]++;

    memcpy(&buf[4], payload, payloadLen);

    return bno08x_i2c_write(dev, buf, totalLen) == HAL_OK;
}

// Read one SHTP packet (blocking, simple polling version)
static bool bno08x_readPacket(BNO08x_Handle_t *dev,
                              uint8_t *channelOut,
                              uint16_t *payloadLenOut)
{
    uint8_t hdr[4];

    // Read header
    if (bno08x_i2c_read(dev, hdr, 4) != HAL_OK) {
        return false;
    }

    uint16_t totalLen = (uint16_t)hdr[0] | ((uint16_t)hdr[1] << 8);
    totalLen &= 0x7FFF; // clear continuation flag if set

    if (totalLen < 4 || totalLen > BNO08X_MAX_PACKET_LEN) {
        return false;
    }

    uint16_t payloadLen = totalLen - 4;

    if (bno08x_i2c_read(dev, dev->rxBuf, payloadLen) != HAL_OK) {
        return false;
    }

    *channelOut     = hdr[2];
    *payloadLenOut  = payloadLen;
    return true;
}

// ---- high-level API ------------------------------------------------------

bool BNO08x_Init(BNO08x_Handle_t *dev, I2C_HandleTypeDef *hi2c)
{
    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;

    // Give BNO08x time to boot
    HAL_Delay(300);

    return true;
}

// Send Set Feature command to enable calibrated gyro (report 0x02)
bool BNO08x_EnableGyro(BNO08x_Handle_t *dev, uint32_t reportInterval_us)
{
    uint8_t payload[17] = {0};

    payload[0] = BNO08X_REPORTID_SET_FEATURE;      // 0xFD
    payload[1] = BNO08X_REPORTID_GYRO_CALIBRATED;  // feature report ID
    payload[2] = 0x00;                             // feature flags (non-wake)
    payload[3] = 0x00;                             // change sensitivity LSB
    payload[4] = 0x00;                             // change sensitivity MSB

    // Report interval in microseconds, little-endian
    payload[5]  = (uint8_t)(reportInterval_us & 0xFF);
    payload[6]  = (uint8_t)((reportInterval_us >> 8) & 0xFF);
    payload[7]  = (uint8_t)((reportInterval_us >> 16) & 0xFF);
    payload[8]  = (uint8_t)((reportInterval_us >> 24) & 0xFF);

    // Batch interval (disabled = 0)
    payload[9]  = 0;
    payload[10] = 0;
    payload[11] = 0;
    payload[12] = 0;

    // Sensor-specific config (unused for basic gyro)
    payload[13] = payload[14] = payload[15] = payload[16] = 0;

    if (!bno08x_sendPacket(dev, BNO08X_CH_CONTROL, payload, sizeof(payload))) {
        return false;
    }

    // You could wait for a "Get Feature Response" here to confirm, but
    // for a minimal example we'll just return true.
    return true;
}

// Convert raw int16 to float (you'll fill in correct scaling from Q-point)
static float gyroRawToDps(int16_t raw)
{
    // TODO: use the Q-point from the SH-2 metadata.
    // Here I just assume some scaling factor as a placeholder.
    const float SCALE = 1.0f; // <-- replace with correct value
    return raw * SCALE;
}

// Poll for a gyro report and convert to floats
bool BNO08x_ReadGyro(BNO08x_Handle_t *dev, float *gz)
{
    uint8_t channel;
    uint16_t len;

    if (!bno08x_readPacket(dev, &channel, &len)) {
        return false;
    }

    if (channel != BNO08X_CH_REPORTS) { // Not a sensor report, ignore
        return false;
    }

    if (len < 10) { // need at least 10 bytes for gyro report
        return false;
    }

    uint8_t *p = dev->rxBuf;
    uint8_t reportId = p[0];

    if (reportId != BNO08X_REPORTID_GYRO_CALIBRATED) {
        // Some other sensor, ignore for now
        return false;
    }

    int16_t gz_raw = (int16_t)((uint16_t)p[8] | ((uint16_t)p[9] << 8));

    *gz = gyroRawToDps(gz_raw);

    return true;
}
