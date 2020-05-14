#pragma once

#include "shared/serial.h"
// #include "net.h"
#define I2CDEV_SERIAL_DEBUG
#ifdef SI_IMPLEMENT_MOTIONAPPS
#    include <MPU6050_6Axis_MotionApps20.h>
#else
typedef struct VectorInt16 VectorInt16;
typedef struct Quaternion Quaternion;
typedef struct VectorFloat VectorFloat;
#    define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#    include <MPU6050.h>
#endif

#include "shared/serial.h"

struct Quaternion_d {
    float w;
    float x;
    float y;
    float z;
};

typedef enum si_mpu_status {
    SI_MPU_DISCONNECTED,
    SI_MPU_CONNECTED,
    SI_MPU_FOUND
} si_mpu_status_t;

typedef struct si_device_state {

    uint8_t led_states = 0;

    si_mpu_status_t mpu_status;
    si_mpu_status_t last_mpu_status;

    uint8_t gyro_flags;
    uint8_t device_flags;

    uint8_t srate;

    uint16_t mpu_expected_packet_size = 16;

    uint64_t main_clk_tmt;
    uint64_t mpu_sample_tmt;

    si_serial_t* serial;

    Quaternion_d offset;
    Quaternion_d current;

} si_device_state_t;

void si_gy_prepare(si_device_state_t*, si_serial_t*);

void si_gyro_check(MPU6050* mpu, si_device_state* state);

void si_gy_run(MPU6050* mpu, si_device_state_t* state, si_serial_t*);

const uint8_t* si_gy_on_req(void*, si_gy_values_t, const uint8_t*);
void si_gy_on_set(void*, si_gy_values_t, const uint8_t*);
void si_gy_on_notify(void*, si_gy_values_t);