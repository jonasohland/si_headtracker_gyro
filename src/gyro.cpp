#define SI_IMPLEMENT_MOTIONAPPS
#include "gyro.h"
#undef SI_IMPLEMENT_MOTIONAPPS
#include <EEPROM.h>

uint8_t si_gy_software_version[] = { 0x00, 0x03, 0x00 };
const char* si_gy_hello_string   = "hello";
uint8_t si_gy_true_false_byte[]  = { 0, 1 };

si_serial_t* iserial;
si_device_state_t* idevice;

volatile uint8_t data_int = false;

Quaternion RotMat2Quat(imu::Matrix<3>& m)
{
    Quaternion ret;
    double tr = m.trace();

    double S;
    if (tr > 0) {
        S     = sqrt(tr + 1.0) * 2;
        ret.w = 0.25 * S;
        ret.x = (m(2, 1) - m(1, 2)) / S;
        ret.y = (m(0, 2) - m(2, 0)) / S;
        ret.z = (m(1, 0) - m(0, 1)) / S;
    }
    else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
        S     = sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
        ret.w = (m(2, 1) - m(1, 2)) / S;
        ret.x = 0.25 * S;
        ret.y = (m(0, 1) + m(1, 0)) / S;
        ret.z = (m(0, 2) + m(2, 0)) / S;
    }
    else if (m(1, 1) > m(2, 2)) {
        S     = sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
        ret.w = (m(0, 2) - m(2, 0)) / S;
        ret.x = (m(0, 1) + m(1, 0)) / S;
        ret.y = 0.25 * S;
        ret.z = (m(1, 2) + m(2, 1)) / S;
    }
    else {
        S     = sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
        ret.w = (m(1, 0) - m(0, 1)) / S;
        ret.x = (m(0, 2) + m(2, 0)) / S;
        ret.y = (m(1, 2) + m(2, 1)) / S;
        ret.z = 0.25 * S;
    }
    return ret;
}

VectorFloat cross(VectorFloat p, VectorFloat v)
{
    VectorFloat ret;
    ret.x = p.y * v.z - p.z * v.y;
    ret.y = p.z * v.x - p.x * v.z;
    ret.z = p.x * v.y - p.y * v.x;
    return ret;
}

void si_gy_reset()
{
    // "call" 0 and crash (not very elegant, but it works)
    ((void (*)()) 0)();
}

void si_gy_prepare(si_device_state_t* st, si_serial_t* serial, MPU6050* mpu)
{
    st->main_clk_tmt   = millis();
    st->mpu_sample_tmt = millis();
    st->serial         = serial;
    st->mpu            = mpu;
    iserial            = serial;
    idevice            = st;

    st->devid = EEPROM.read(0);

    st->mpu_status = SI_MPU_DISCONNECTED;

    Fastwire::setup(400, true);

    EEPROM.get(16, st->q_cal);
    si_gy_reset_orientation(st);
    si_gyro_init(mpu, st);
}

void si_gyro_check(MPU6050* mpu, si_device_state_t* st)
{
    uint8_t buf[MPU6050_WHO_AM_I_LENGTH];
    buf[0] = 0;

    uint8_t ret = I2Cdev::readBytes(MPU6050_DEFAULT_ADDRESS,
                                    MPU6050_RA_WHO_AM_I,
                                    MPU6050_WHO_AM_I_LENGTH,
                                    buf);

    if (ret < 1 || buf[0] != 104) { st->mpu_status = SI_MPU_DISCONNECTED; }
    else {
        if (st->mpu_status == SI_MPU_DISCONNECTED) {
            st->mpu_status = SI_MPU_FOUND;
        }
    }
}

void si_interrupt()
{
    ++idevice->interrupt_cnt;
    if (idevice->mpu_status != SI_MPU_CONNECTED) return;

    data_int = true;
}

void si_gyro_init(MPU6050* mpu, si_device_state_t* st)
{
    pinMode(2, INPUT);
    mpu->initialize();

    delay(500);

    if (!mpu->dmpInitialize()) {

        st->mpu_expected_packet_size = mpu->dmpGetFIFOPacketSize();

        mpu->setInterruptMode(true);
        mpu->setInterruptLatch(false);

        mpu->setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(2), si_interrupt, RISING);

        st->mpu_status = SI_MPU_CONNECTED;
    }
    else {
        st->mpu_status = SI_MPU_DISCONNECTED;
    }
}


void si_sample(MPU6050* mpu, si_device_state_t* st, si_serial_t* serial)
{
    if (!data_int) return;
    data_int = false;

    int fifocnt = idevice->mpu->getFIFOCount();

    if (fifocnt >= 28) {

        digitalWrite(LED_BUILTIN, HIGH);

        uint8_t buf[28];

        ++idevice->read_cnt;
        idevice->mpu->getFIFOBytes(buf, idevice->mpu_expected_packet_size);
        idevice->mpu->dmpGetQuaternion((Quaternion*) &idevice->q_raw, buf);
        idevice->mpu->dmpGetAccel((VectorInt16*) &idevice->g_raw, buf);
    }
}

void si_write_sample(si_device_state_t* st)
{
    Quaternion quat, steering;

    Quaternion* qCalLeft  = (Quaternion*) &st->q_cal_L;
    Quaternion* qCalRight = (Quaternion*) &st->q_cal_R;
    Quaternion* qRaw      = (Quaternion*) &st->q_raw;
    Quaternion* qIdleConj = (Quaternion*) &st->q_idle_conj;

    steering = qIdleConj->getProduct(*qRaw);
    quat     = qCalLeft->getProduct(steering);
    quat     = quat.getProduct(*qCalRight);

    if(st->gyro_flags & SI_FLAG_ST_INVERT_X)
        quat.x = -quat.x;

    if(st->gyro_flags & SI_FLAG_ST_INVERT_Y)
        quat.y = -quat.y;
    
    if(st->gyro_flags & SI_FLAG_ST_INVERT_Z)
        quat.z = -quat.z;

    si_serial_set_value(st->serial, SI_GY_QUATERNION_FLOAT, (uint8_t*) &quat);
}

void si_gy_calibration_progress(uint8_t prg, void* serial)
{
    si_serial_set_value((si_serial_t*) serial, SI_GY_CALIBRATE, &prg);
}

void si_gy_calibrate(MPU6050* mpu,
                     si_device_state_t* st,
                     si_serial_t* serial,
                     uint8_t lcnt)
{
    uint8_t restart = st->device_flags & SI_FLAG_SEND_DATA;

    mpu->CalibrateGyro(lcnt, serial, si_gy_calibration_progress);
    mpu->CalibrateAccel(lcnt, serial, si_gy_calibration_progress);

    if (restart) st->device_flags |= SI_FLAG_SEND_DATA;
}

void si_gy_reset_orientation(si_device_state_t* dev)
{
    *((Quaternion*) &dev->q_cal_L)
        = ((Quaternion*) &dev->q_cal)->getConjugate();
    memcpy(&dev->q_cal_R, &dev->q_cal, sizeof(qbuf));
}


void si_gy_init_begin(si_device_state_t* dev)
{
    *((Quaternion*) (&dev->q_idle_conj))
        = ((Quaternion*) (&dev->q_raw))->getConjugate();

    *((VectorFloat*) (&dev->g_grv_idle)) = VectorFloat(
        dev->g_raw.storage[0], dev->g_raw.storage[1], dev->g_raw.storage[2]);
}

void si_gy_init_finish(si_device_state_t* dev)
{
    VectorFloat g, gCal, x, y, z;

    *((VectorFloat*) &dev->v_grv_cal) = VectorFloat(
        dev->g_raw.storage[0], dev->g_raw.storage[1], dev->g_raw.storage[2]);

    g = *((VectorFloat*) &dev->g_grv_idle);
    z = g;
    z.normalize();

    gCal = VectorFloat(
        dev->v_grv_cal.storage[0], dev->v_grv_cal.storage[1], dev->v_grv_cal.storage[2]);
    y = cross(gCal, g);
    y.normalize();

    x = cross(y, z);
    x.normalize();

    imu::Matrix<3> rot;

    rot.cell(0, 0) = x.x;
    rot.cell(1, 0) = x.y;
    rot.cell(2, 0) = x.z;
    rot.cell(0, 1) = y.x;
    rot.cell(1, 1) = y.y;
    rot.cell(2, 1) = y.z;
    rot.cell(0, 2) = z.x;
    rot.cell(1, 2) = z.y;
    rot.cell(2, 2) = z.z;

    *((Quaternion*) &dev->q_cal) = RotMat2Quat(rot);
    EEPROM.put(16, dev->q_cal);
}

void si_gy_run(MPU6050* mpu, si_device_state_t* st, si_serial_t* serial)
{
    si_sample(mpu, st, serial);

    if (st->mpu_status == SI_MPU_CONNECTED
        && (st->device_flags & SI_FLAG_SEND_DATA)) {

        if (millis() - st->mpu_sample_tmt > 1000 / st->srate) {
            st->mpu_sample_tmt = millis();
            si_write_sample(st);
        }
    }
}

const uint8_t*
si_gy_on_req(void* dev, si_gy_values_t value, const uint8_t* data)
{
    si_device_state_t* device = (si_device_state_t*) dev;

    // clang-format off
    switch (value) {
        case SI_GY_SRATE: 
            return &device->srate;
        case SI_GY_VERSION: 
            return (const uint8_t*) &si_gy_software_version;
        case SI_GY_HELLO: 
            return (const uint8_t*) si_gy_hello_string;
        case SI_GY_ALIVE: 
            return &si_gy_true_false_byte[true];
        case SI_GY_FOUND:
            return &si_gy_true_false_byte[device->mpu_status == SI_MPU_FOUND];
        case SI_GY_CONNECTED:
            return &si_gy_true_false_byte[device->mpu_status == SI_MPU_CONNECTED];
        case SI_GY_ENABLE:
            return &si_gy_true_false_byte[device->device_flags & SI_FLAG_SEND_DATA];
        case SI_GY_RESET:
            si_serial_write_message(device->serial, SI_GY_RESP, SI_GY_RESET, si_gy_true_false_byte + 1);
            delay(2000);
            si_gy_reset();
            return &si_gy_true_false_byte[0];
        case SI_GY_INT_COUNT:
            return (uint8_t*) &device->interrupt_cnt;
        case SI_GY_QUATERNION_FLOAT:
            return &si_gy_true_false_byte[device->device_flags & SI_FLAG_OUTPUT_FLOAT];
        case SI_GY_ID:
            return &device->devid;
        case SI_GY_INIT_BEGIN:
            si_gy_init_begin(device);
            return si_gy_true_false_byte;
        case SI_GY_INIT_FINISH:
            si_gy_init_finish(device);
            return si_gy_true_false_byte;
        default: 
            return nullptr;
    }
    // clang-format on
}

void si_gy_on_set(void* dev, si_gy_values_t value, const uint8_t* data)
{
    si_device_state_t* device = (si_device_state_t*) dev;

    switch (value) {
        case SI_GY_SRATE: device->srate = data[0]; break;
        case SI_GY_ENABLE:
            (data[0]) ? device->device_flags |= SI_FLAG_SEND_DATA
                      : device->device_flags &= ~(SI_FLAG_SEND_DATA);
            break;
        case SI_GY_INV:
            device->gyro_flags = (device->gyro_flags & ~SI_FLAG_INVERT_BITMASK)
                                 | (*data & SI_FLAG_INVERT_BITMASK);
            break;
        case SI_GY_RESET_ORIENTATION:
            *((Quaternion*) &device->q_idle_conj)
                = ((Quaternion*) &device->q_raw)->getConjugate();
            break;
        case SI_GY_QUATERNION_FLOAT:
            (data[0]) ? device->gyro_flags |= SI_FLAG_OUTPUT_FLOAT
                      : device->gyro_flags &= ~(SI_FLAG_OUTPUT_FLOAT);
            break;
        case SI_GY_ID:
            device->devid = *data;
            EEPROM.write(0, device->devid);
            break;
        case SI_GY_CALIBRATE:
            si_gy_calibrate(device->mpu, device, device->serial, *data);
            break;
        default: break;
    }
}

void si_gy_on_notify(void* dev, si_gy_values_t val)
{
    // si_device_state_t* device = (si_device_state_t*) dev;

    switch (val) {
        default: break;
    }
}