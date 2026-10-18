#include "Gyro_QMI8658.h"

IMUdata Accel;
IMUdata Gyro;

uint8_t Device_addr;
acc_scale_t acc_scale = ACC_RANGE_4G;
gyro_scale_t gyro_scale = GYR_RANGE_512DPS;
acc_odr_t acc_odr = acc_odr_norm_250;
gyro_odr_t gyro_odr = gyro_odr_norm_250;
sensor_state_t sensor_state = sensor_default;
lpf_t acc_lpf;

float accelScales, gyroScales;
uint8_t readings[12];
uint32_t reading_timestamp_us;

void QMI8658_Init(void)
{
    uint8_t buf[1] = {0};
    Device_addr = QMI8658_L_SLAVE_ADDRESS;
    esp_err_t idErr = I2C_Read(Device_addr, QMI8658_REVISION_ID, buf, 1);
    if (idErr == ESP_OK) {
        printf("QMI8658 ID: %x\r\n", buf[0]);
        if (buf[0] != QMI8658_REVISION_ID_VALUE) {
            printf("QMI8658 unexpected revision 0x%02X (want 0x%02X)\r\n",
                   buf[0], (unsigned)QMI8658_REVISION_ID_VALUE);
        }
    } else {
        printf("QMI8658 probe failed err=%d addr 0x%02X\r\n", (int)idErr, Device_addr);
    }
    setState(sensor_running);

    setAccScale(acc_scale);
    setAccODR(acc_odr);
    setAccLPF(LPF_MODE_0);
    switch (acc_scale) {
        case ACC_RANGE_2G:  accelScales = 2.0 / 32768.0; break;
        case ACC_RANGE_4G:  accelScales = 4.0 / 32768.0; break;
        case ACC_RANGE_8G:  accelScales = 8.0 / 32768.0; break;
        case ACC_RANGE_16G: accelScales = 16.0 / 32768.0; break;
    }

    setGyroScale(gyro_scale);
    setGyroODR(gyro_odr);
    setGyroLPF(LPF_MODE_3);
    switch (gyro_scale) {
        case GYR_RANGE_16DPS:   gyroScales = 16.0 / 32768.0; break;
        case GYR_RANGE_32DPS:   gyroScales = 32.0 / 32768.0; break;
        case GYR_RANGE_64DPS:   gyroScales = 64.0 / 32768.0; break;
        case GYR_RANGE_128DPS:  gyroScales = 128.0 / 32768.0; break;
        case GYR_RANGE_256DPS:  gyroScales = 256.0 / 32768.0; break;
        case GYR_RANGE_512DPS:  gyroScales = 512.0 / 32768.0; break;
        case GYR_RANGE_1024DPS: gyroScales = 1024.0 / 32768.0; break;
    }
}

void QMI8658_Loop(void)
{
    getAccelerometer();
}

void QMI8658_transmit(uint8_t addr, uint8_t data)
{
    I2C_Write(Device_addr, addr, &data, 1);
}

uint8_t QMI8658_receive(uint8_t addr)
{
    uint8_t retval;
    I2C_Read(Device_addr, addr, &retval, 1);
    return retval;
}

void QMI8658_CTRL9_Write(uint8_t command)
{
    QMI8658_transmit(QMI8658_CTRL9, command);
    // Bounded wait: an unbounded poll here wedges the calling task forever if
    // the sensor never clears STATUSINT (bus fault, unpowered peripheral).
    // vTaskDelay yields so the watchdog and other tasks keep running.
    for (int attempt = 0; attempt < 100; attempt++) {
        if ((QMI8658_receive(QMI8658_STATUSINT) & 0x80) != 0x00) return;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    printf("QMI8658: CTRL9 command 0x%02X did not complete (timeout)\r\n", command);
}

void setAccODR(acc_odr_t odr)
{
    if (sensor_state != sensor_default) {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_AODR_MASK;
        ctrl2 |= odr;
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_odr = odr;
}

void setGyroODR(gyro_odr_t odr)
{
    if (sensor_state != sensor_default) {
        uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
        ctrl3 &= ~QMI8658_GODR_MASK;
        ctrl3 |= odr;
        QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_odr = odr;
}

void setAccScale(acc_scale_t scale)
{
    if (sensor_state != sensor_default) {
        uint8_t ctrl2 = QMI8658_receive(QMI8658_CTRL2);
        ctrl2 &= ~QMI8658_ASCALE_MASK;
        ctrl2 |= scale << QMI8658_ASCALE_OFFSET;
        QMI8658_transmit(QMI8658_CTRL2, ctrl2);
    }
    acc_scale = scale;
}

void setGyroScale(gyro_scale_t scale)
{
    if (sensor_state != sensor_default) {
        uint8_t ctrl3 = QMI8658_receive(QMI8658_CTRL3);
        ctrl3 &= ~QMI8658_GSCALE_MASK;
        ctrl3 |= scale << QMI8658_GSCALE_OFFSET;
        QMI8658_transmit(QMI8658_CTRL3, ctrl3);
    }
    gyro_scale = scale;
}

void setAccLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default) {
        uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
        // Must be ~ (bitwise NOT), not ! (logical NOT): "!mask" evaluates to 0
        // for any nonzero mask, so the old code cleared the whole register
        // instead of just the ALPF field.
        ctrl5 &= (uint8_t)~QMI8658_ALPF_MASK;
        ctrl5 |= lpf << QMI8658_ALPF_OFFSET;
        ctrl5 |= 0x01;
        QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
    acc_lpf = lpf;
}

void setGyroLPF(lpf_t lpf)
{
    if (sensor_state != sensor_default) {
        uint8_t ctrl5 = QMI8658_receive(QMI8658_CTRL5);
        // ~ not ! — see setAccLPF() for why this matters.
        ctrl5 &= (uint8_t)~QMI8658_GLPF_MASK;
        ctrl5 |= lpf << QMI8658_GLPF_OFFSET;
        ctrl5 |= 0x10;
        QMI8658_transmit(QMI8658_CTRL5, ctrl5);
    }
}

void setState(sensor_state_t state)
{
    uint8_t ctrl1;
    switch (state)
    {
    case sensor_running:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        ctrl1 &= 0xFE;
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);
        QMI8658_transmit(QMI8658_CTRL7, 0x43);
        QMI8658_transmit(QMI8658_CTRL6, 0x00);
        break;
    case sensor_power_down:
        QMI8658_transmit(QMI8658_CTRL7, 0x00);
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        ctrl1 |= 0x01;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);
        break;
    case sensor_locking:
        ctrl1 = QMI8658_receive(QMI8658_CTRL1);
        ctrl1 &= 0xFE;
        ctrl1 |= 0x40;
        QMI8658_transmit(QMI8658_CTRL1, ctrl1);
        QMI8658_transmit(QMI8658_CTRL7, 0x83);
        QMI8658_transmit(QMI8658_CTRL6, 0x00);
        QMI8658_transmit(QMI8658_CAL1_L, 0x01);
        QMI8658_CTRL9_Write(0x12);
        QMI8658_transmit(QMI8658_CAL1_L, 0x00);
        QMI8658_CTRL9_Write(0x12);
        break;
    default:
        break;
    }
    sensor_state = state;
}

// Holds the last good reading so a transient I2C failure never injects
// uninitialised stack bytes into a recording as if they were real samples.
static int16_t s_lastImu[6] = {0, 0, 0, 0, 0, 0};
static bool    s_haveImu    = false;

// Host-side per-axis bias trim, in raw LSB. Applied to the RAW values that
// BurstRead returns, so everything downstream (BLE packets, SD CSV, the
// tremor DSP) sees trimmed data without touching its own scaling code.
// The static-capture log that motivated this showed a consistent gyro bias
// of roughly -48/-43/+13 LSB (~-0.75/-0.66/+0.20 dps at 512 dps range):
// small, but it is DC error right in the middle of the 4-12 Hz tremor band's
// baseline estimate and it inflates any Allan-variance/zero-rate calculation.
static int16_t s_biasTrim[6] = {0, 0, 0, 0, 0, 0};

void QMI8658_SetBiasTrim(const int16_t trim[6])
{
    for (int i = 0; i < 6; i++) s_biasTrim[i] = trim[i];
}

bool QMI8658_GetBiasTrim(int16_t trim[6])
{
    if (!trim) return false;
    for (int i = 0; i < 6; i++) trim[i] = s_biasTrim[i];
    return true;
}

// Mean-of-N idle calibration. Must be called with the device STILL and
// level. Reads straight from the sensor registers (not through the trim)
// so repeated calibrations converge instead of double-subtracting. Returns
// the number of samples actually averaged; 0 means every I2C read failed
// and no trim was applied. Runs on the caller's task — call it from setup()
// or an idle moment, never from the 200 Hz IMU task (each sample costs one
// blocking I2C burst plus a 5 ms wait).
uint32_t QMI8658_CalibrateIdle(int samples)
{
    if (samples < 16) samples = 16;
    if (samples > 2000) samples = 2000;

    int32_t sum[6] = {0, 0, 0, 0, 0, 0};
    int     n      = 0;
    uint8_t buf[12];

    for (int i = 0; i < samples; i++) {
        if (I2C_Read(Device_addr, QMI8658_AX_L, buf, 12) == ESP_OK) {
            int16_t v[6];
            for (int a = 0; a < 6; a++)
                v[a] = (int16_t)((buf[a * 2 + 1] << 8) | buf[a * 2]);
            // Reject gross outliers: a bump or table vibration during the
            // "idle" window must not poison the mean. Anything beyond ~0.5 g
            // / ~8 dps away from the running mean is skipped.
            bool ok = true;
            if (n >= 16) {
                for (int a = 0; a < 6; a++) {
                    int32_t mean = sum[a] / n;
                    int32_t lim  = (a < 3) ? 4096 : 512;   // raw LSB thresholds
                    if (abs((int32_t)v[a] - mean) > lim) { ok = false; break; }
                }
            }
            if (ok) {
                for (int a = 0; a < 6; a++) sum[a] += v[a];
                n++;
            }
        }
        delay(5);   // one ODR period at 200-250 Hz
    }

    if (n < 16) return 0;   // too few good samples to trust

    int16_t trim[6];
    for (int a = 0; a < 6; a++) {
        // Accel Z should read +1 g at rest (gravity), so only trim the
        // deviation from the expected gravity vector per axis. For X/Y at
        // level that expectation is 0; for Z we keep the 1 g component.
        // A full attitude-aware gravity model belongs in post-processing;
        // here we trim gyro fully and accel X/Y fully, and leave accel Z
        // alone unless it is clearly offset (> 0.05 g from ±32768·g-axis).
        int32_t m = sum[a] / n;
        if (a == 2) {
            int32_t expect = 32768;   // +1 g at ACC_RANGE_.../32768 LSB scale
            int32_t dev    = m - expect;
            trim[a] = (abs(dev) > 1638) ? (int16_t)dev : 0;   // >0.05 g?
        } else {
            trim[a] = (int16_t)m;
        }
    }
    QMI8658_SetBiasTrim(trim);
    printf("QMI8658 idle cal: n=%lu trim ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\r\n",
           (unsigned long)n, trim[0], trim[1], trim[2], trim[3], trim[4], trim[5]);
    return (uint32_t)n;
}

void QMI8658_BurstRead(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[12] = {0};
    // Compare against ESP_OK explicitly: I2C_Read returns esp_err_t, and while
    // ESP_OK happens to be 0, spelling it out keeps this consistent with every
    // other call site and survives any future error-code redefinition.
    if (I2C_Read(Device_addr, QMI8658_AX_L, buf, 12) != ESP_OK) {
        if (s_haveImu) {
            *ax = s_lastImu[0]; *ay = s_lastImu[1]; *az = s_lastImu[2];
            *gx = s_lastImu[3]; *gy = s_lastImu[4]; *gz = s_lastImu[5];
        }
        return;
    }
    s_lastImu[0] = (int16_t)(((buf[1] << 8) | buf[0]) - s_biasTrim[0]);
    s_lastImu[1] = (int16_t)(((buf[3] << 8) | buf[2]) - s_biasTrim[1]);
    s_lastImu[2] = (int16_t)(((buf[5] << 8) | buf[4]) - s_biasTrim[2]);
    s_lastImu[3] = (int16_t)(((buf[7] << 8) | buf[6]) - s_biasTrim[3]);
    s_lastImu[4] = (int16_t)(((buf[9] << 8) | buf[8]) - s_biasTrim[4]);
    s_lastImu[5] = (int16_t)(((buf[11] << 8) | buf[10]) - s_biasTrim[5]);
    s_haveImu = true;
    *ax = s_lastImu[0];
    *ay = s_lastImu[1];
    *az = s_lastImu[2];
    *gx = s_lastImu[3];
    *gy = s_lastImu[4];
    *gz = s_lastImu[5];
}

void getRawReadings(int16_t* buf)
{
    uint8_t raw[12];
    I2C_Read(Device_addr, QMI8658_AX_L, raw, 12);
    for (int i = 0; i < 6; i++) {
        buf[i] = (int16_t)((raw[i * 2 + 1] << 8) | raw[i * 2]);
    }
}

void getAccelerometer(void)
{
    uint8_t buf[6];
    esp_err_t ret = I2C_Read(Device_addr, QMI8658_AX_L, buf, 6);
    if (ret != ESP_OK)
        printf("QMI8658: Acc read fail\r\n");
    else {
        Accel.x = (float)((int16_t)((buf[1] << 8) | (buf[0])));
        Accel.y = (float)((int16_t)((buf[3] << 8) | (buf[2])));
        Accel.z = (float)((int16_t)((buf[5] << 8) | (buf[4])));
        Accel.x = Accel.x * accelScales;
        Accel.y = Accel.y * accelScales;
        Accel.z = Accel.z * accelScales;
    }
}

void getGyroscope(void)
{
    uint8_t buf[6];
    esp_err_t ret = I2C_Read(Device_addr, QMI8658_GX_L, buf, 6);
    if (ret != ESP_OK)
        printf("QMI8658: Gyro read fail\r\n");
    else {
        Gyro.x = (float)((int16_t)((buf[1] << 8) | (buf[0])));
        Gyro.y = (float)((int16_t)((buf[3] << 8) | (buf[2])));
        Gyro.z = (float)((int16_t)((buf[5] << 8) | (buf[4])));
        Gyro.x = Gyro.x * gyroScales;
        Gyro.y = Gyro.y * gyroScales;
        Gyro.z = Gyro.z * gyroScales;
    }
}

float getAccX() { return Accel.x; }
float getAccY() { return Accel.y; }
float getAccZ() { return Accel.z; }
float getGyroX() { return Gyro.x; }
float getGyroY() { return Gyro.y; }
float getGyroZ() { return Gyro.z; }
