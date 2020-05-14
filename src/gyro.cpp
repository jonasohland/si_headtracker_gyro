#define SI_IMPLEMENT_MOTIONAPPS
#include "gyro.h"
#undef SI_IMPLEMENT_MOTIONAPPS

uint8_t si_gy_software_version[] = { 0x00, 0x00, 0x01 };
const char* si_gy_hello_string   = "hello";
uint8_t si_gy_true_false_byte[]  = { 0, 1 };

void si_gy_prepare(si_device_state_t* st, si_serial_t* serial)
{
    st->main_clk_tmt   = millis();
    st->mpu_sample_tmt = millis();
    st->serial         = serial;

    st->mpu_status = SI_MPU_DISCONNECTED;

    Fastwire::setup(100, true);
    I2Cdev::readTimeout = 100;
}

void si_gyro_check(MPU6050* mpu, si_device_state_t* st)
{
    uint8_t buf[MPU6050_WHO_AM_I_LENGTH];
    buf[0] = 0;

    uint8_t ret = I2Cdev::readBytes(MPU6050_DEFAULT_ADDRESS,
                                    MPU6050_RA_WHO_AM_I,
                                    MPU6050_WHO_AM_I_LENGTH,
                                    buf);

    if (ret < 1 || buf[0] != 104)
        st->mpu_status = SI_MPU_DISCONNECTED;
    else {
        if (st->mpu_status == SI_MPU_DISCONNECTED)
            st->mpu_status = SI_MPU_FOUND;
    }

    digitalWrite(LED_BUILTIN, st->mpu_status == SI_MPU_CONNECTED);
    /*
        if (!(SI_CONF_FLAG(conf, SI_FLAG_CFG_STR_ENABLED))
            || st->mpu_status == SI_MPU_DISCONNECTED)
            digitalWrite(SI_GY_STATUS_PIN, st->mpu_status !=
       SI_MPU_DISCONNECTED);
    */
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

void si_sample(MPU6050* mpu, si_device_state_t* st)
{
    uint8_t buf[64];

    int mpuIntStatus = mpu->getIntStatus();
    int fifocnt      = mpu->getFIFOCount();

    if (mpuIntStatus & _BV(0) && fifocnt >= 16) {

        // si_data_packet_t pck;

        Quaternion_d q;

        mpu->getFIFOBytes(buf, st->mpu_expected_packet_size);
        mpu->dmpGetQuaternion((Quaternion*) &q, buf);
        mpu->resetFIFO();

        if (st->gyro_flags & SI_FLAG_RESET_ORIENTATION) {

            Quaternion current_rot(st->current.w / 16384.f,
                                   st->current.x / 16384.f,
                                   st->current.y / 16384.f,
                                   st->current.z / 16384.f);

            current_rot = current_rot.getConjugate();

            memcpy(&st->offset, &current_rot, sizeof(Quaternion));

            st->gyro_flags |= SI_FLAG_APPLY_OFFSETS;
            st->gyro_flags &= ~(SI_FLAG_RESET_ORIENTATION);
        }

        if (st->gyro_flags & SI_FLAG_APPLY_OFFSETS) {

            Quaternion current_rot(st->current.w / 16384.f,
                                   st->current.x / 16384.f,
                                   st->current.y / 16384.f,
                                   st->current.z / 16384.f);

            Quaternion nq
                = current_rot.getProduct(*((Quaternion*) &st->offset));

            st->current.w = nq.w * 16384;
            st->current.x = nq.x * 16384;
            st->current.y = nq.y * 16384;
            st->current.z = nq.z * 16384;
        }

        if (st->gyro_flags & SI_FLAG_ST_INVERT_X)
            st->current.x = -st->current.x;

        if (st->gyro_flags & SI_FLAG_ST_INVERT_Y)
            st->current.y = -st->current.y;

        if (st->gyro_flags & SI_FLAG_ST_INVERT_Z)
            st->current.z = -st->current.z;
    }
}

void si_gy_run(MPU6050* mpu, si_device_state_t* st)
{
    /* if (millis() - st->main_clk_tmt > 1000 / 2) {

        si_gyro_check(mpu, st);

        if (st->mpu_status != st->last_mpu_status)
            st->last_mpu_status = st->mpu_status;

        if (st->mpu_status == SI_MPU_FOUND) si_gyro_init(mpu, st);

        st->main_clk_tmt = millis();
    } */

    if (/* st->mpu_status == SI_MPU_CONNECTED
        && */
        st->device_flags
        & SI_FLAG_SEND_DATA) {

        if (millis() - st->mpu_sample_tmt > 1000 / st->srate) {

            Quaternion q;

            st->mpu_sample_tmt = millis();
            si_serial_set_value(st->serial, SI_GY_QUATERNION, (uint8_t*)&q);
            // si_sample(mpu, st);
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
const uint8_t* si_gy_on_req(void* dev, si_gy_values_t value, const uint8_t* data)
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
        default: 
            return nullptr;
    }
    // clang-format on
}

void si_gy_on_set(void* dev, si_gy_values_t value, const uint8_t* data)
{
    si_device_state_t* device = (si_device_state_t*) dev;

    switch (value) {
        case SI_GY_SRATE: 
            device->srate = data[0];
            si_serial_set_value(device->serial, SI_GY_SRATE, &device->srate); 
            break;
        case SI_GY_ENABLE:
            (data[0])
                ? device->device_flags |= SI_FLAG_SEND_DATA
                : device->device_flags &= ~(SI_FLAG_SEND_DATA);
            si_serial_set_value(device->serial, SI_GY_ENABLE, &device->device_flags);
            break;
        default: break;
    }
}

void si_gy_on_notify(void* dev, si_gy_values_t val)
{
    si_device_state_t* device = (si_device_state_t*) dev;

    switch (val) {
        case SI_GY_ALIVE: si_serial_notify(device->serial, SI_GY_ALIVE);
        default: break;
    }
}