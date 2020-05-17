#define SI_IMPLEMENT_MOTIONAPPS
#include "gyro.h"
#undef SI_IMPLEMENT_MOTIONAPPS

uint8_t si_gy_software_version[] = { 0x00, 0x00, 0x01 };
const char* si_gy_hello_string   = "hello";
uint8_t si_gy_true_false_byte[]  = { 0, 1 };

void si_gy_reset()
{
    // "call" 0
    ((void (*)()) 0)();
}

void si_gy_prepare(si_device_state_t* st, si_serial_t* serial)
{
    st->main_clk_tmt   = millis();
    st->mpu_sample_tmt = millis();
    st->serial         = serial;

    st->mpu_status = SI_MPU_DISCONNECTED;

    Fastwire::setup(400, true);
    I2Cdev::readTimeout = 500;
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

void si_gyro_init(MPU6050* mpu, si_device_state_t* st)
{
    mpu->initialize();

    delay(500);

    if (!mpu->dmpInitialize()) {

        st->mpu_expected_packet_size = mpu->dmpGetFIFOPacketSize();
        st->mpu_status               = SI_MPU_CONNECTED;

        mpu->setDMPEnabled(true);
    }
    else {
        st->mpu_status = SI_MPU_DISCONNECTED;
    }
}

void si_sample(MPU6050* mpu, si_device_state_t* st, si_serial_t* serial)
{
    uint8_t buf[64];

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
    }
}

void si_gy_run(MPU6050* mpu, si_device_state_t* st, si_serial_t* serial)
{
    if (millis() - st->main_clk_tmt > 1000 / 2) {

        si_gyro_check(mpu, st);

        if (st->mpu_status != st->last_mpu_status)
            st->last_mpu_status = st->mpu_status;

        if (st->mpu_status == SI_MPU_FOUND) si_gyro_init(mpu, st);

        st->main_clk_tmt = millis();
    }

    if (st->mpu_status == SI_MPU_CONNECTED
        && (st->device_flags & SI_FLAG_SEND_DATA)) {

        if (millis() - st->mpu_sample_tmt > 1000 / st->srate) {
            st->mpu_sample_tmt = millis();
            si_sample(mpu, st, serial);
        }
    }
}
/*
void si_gy_handle_hello(si_device_state_t* state, si_serial_t* serial, const
char* buf)
{
    const char* hello = "hello";
    si_serial_set_value(serial, SI_GY_HELLO, hello);
}

void si_gy_handle_srate(si_device_state_t* state, uint8_t srate)
{
    state->srate = srate;
}

void si_gy_handle_alive(si_device_state_t* state, si_serial_t* serial)
{
    uint8_t one = 1;
    si_serial_set_value(serial, SI_GY_ALIVE, &one);
}

void si_gy_handle_enable(si_device_state_t* state, uint8_t enabled)
{
    if(enabled)
        state->device_flags |= SI_FLAG_SEND_DATA;
    else
        state->device_flags &= ~(SI_FLAG_SEND_DATA);
}
*/
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