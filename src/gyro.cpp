#define SI_IMPLEMENT_MOTIONAPPS
#include "gyro.h"
#undef SI_IMPLEMENT_MOTIONAPPS

uint8_t si_gy_software_version[] = { 0x00, 0x00, 0x04 };
const char* si_gy_hello_string   = "hello";
uint8_t si_gy_true_false_byte[]  = { 0, 1 };

si_serial_t* iserial;
si_device_state_t* idevice;

volatile uint8_t data_int = false;

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
        idevice->mpu->dmpGetQuaternion(
            (Quaternion*) &idevice->last_quaternion, buf);
    }

    /* uint8_t buf[64];

    int mpuIntStatus = mpu->getIntStatus();
    int fifocnt      = mpu->getFIFOCount();

    if (mpuIntStatus & _BV(0) && fifocnt >= 16) {

        Quaternion q;

        mpu->getFIFOBytes(buf, st->mpu_expected_packet_size);
        mpu->dmpGetQuaternion(&q, buf);
        mpu->resetFIFO();

        if (st->gyro_flags & SI_FLAG_RESET_ORIENTATION) {

            Quaternion current_rot = q.getConjugate();

            memcpy(&st->offset, &current_rot, sizeof(Quaternion));

            st->gyro_flags |= SI_FLAG_APPLY_OFFSETS;
            st->gyro_flags &= ~(SI_FLAG_RESET_ORIENTATION);

            si_serial_write_message(st->serial,
                                    SI_GY_NOTIFY,
                                    SI_GY_RESET_ORIENTATION,
                                    si_gy_true_false_byte);
        }

        if (st->gyro_flags & SI_FLAG_APPLY_OFFSETS)
            q = q.getProduct(*((Quaternion*) &st->offset));

        if (st->gyro_flags & SI_FLAG_ST_INVERT_X) q.x = -q.x;
        if (st->gyro_flags & SI_FLAG_ST_INVERT_Y) q.y = -q.y;
        if (st->gyro_flags & SI_FLAG_ST_INVERT_Z) q.z = -q.z;

        si_serial_set_value(serial, SI_GY_QUATERNION, (uint8_t*) &q);
    } */
}

void si_write_sample(si_device_state_t* st)
{
    // TODO: Replace this with int16 math. Float math is very slow on
    // ATMEGA328Ps
    if (st->gyro_flags & SI_FLAG_RESET_ORIENTATION) {

        Quaternion current_rot
            = ((Quaternion*) (&st->last_quaternion))->getConjugate();

        memcpy(&st->offset, &current_rot, sizeof(Quaternion));

        st->gyro_flags |= SI_FLAG_APPLY_OFFSETS;
        st->gyro_flags &= ~(SI_FLAG_RESET_ORIENTATION);

        si_serial_write_message(st->serial,
                                SI_GY_NOTIFY,
                                SI_GY_RESET_ORIENTATION,
                                si_gy_true_false_byte);
    }

    if (st->gyro_flags & SI_FLAG_APPLY_OFFSETS)
        *((Quaternion*) (&st->last_quaternion))
            = ((Quaternion*) (&st->last_quaternion))
                  ->getProduct(*((Quaternion*) (&st->offset)));

    if (st->gyro_flags & SI_FLAG_ST_INVERT_X)
        st->last_quaternion.x = -st->last_quaternion.x;
    if (st->gyro_flags & SI_FLAG_ST_INVERT_Y)
        st->last_quaternion.y = -st->last_quaternion.y;
    if (st->gyro_flags & SI_FLAG_ST_INVERT_Z)
        st->last_quaternion.z = -st->last_quaternion.z;


    if (st->device_flags & SI_GY_QUATERNION_FLOAT)
        si_serial_write_message(iserial,
                                SI_GY_SET,
                                SI_GY_QUATERNION_FLOAT,
                                (uint8_t*) &idevice->last_quaternion);
    else {
        IntQuat qint;

        qint.w = st->last_quaternion.w * 16384.0f;
        qint.x = st->last_quaternion.x * 16384.0f;
        qint.y = st->last_quaternion.y * 16384.0f;
        qint.z = st->last_quaternion.z * 16384.0f;

        si_serial_write_message(
            iserial, SI_GY_SET, SI_GY_QUATERNION_INT16, (uint8_t*) &qint);
    }
}

void si_gy_run(MPU6050* mpu, si_device_state_t* st, si_serial_t* serial)
{
    /* if (millis() - st->main_clk_tmt > 1000 / 2) {

        si_gyro_check(mpu, st);

        if (st->mpu_status != st->last_mpu_status)
            st->last_mpu_status = st->mpu_status;

        if (st->mpu_status == SI_MPU_FOUND) si_gyro_init(mpu, st);

        st->main_clk_tmt = millis();
    } */

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
            // si_gy_reset();
            return &si_gy_true_false_byte[0];
        case SI_GY_INT_COUNT:
            return (uint8_t*) &device->interrupt_cnt;
        case SI_GY_QUATERNION_FLOAT:
            return &si_gy_true_false_byte[device->device_flags & SI_FLAG_OUTPUT_FLOAT];
        case SI_GY_ID:
            return &device->devid;
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
            device->gyro_flags |= SI_FLAG_RESET_ORIENTATION;
            break;
        case SI_GY_QUATERNION_FLOAT:
            (data[0]) ? device->gyro_flags |= SI_FLAG_OUTPUT_FLOAT
                      : device->gyro_flags &= ~(SI_FLAG_OUTPUT_FLOAT);
            break;
        case SI_GY_ID:
            device->devid = *data;
            EEPROM.write(0, device->devid);
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