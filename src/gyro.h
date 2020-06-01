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

#define SI_SELF_RESET_PIN A0

#include "imumaths/imumaths.h"
#include "shared/serial.h"


typedef enum si_mpu_status {
    SI_MPU_DISCONNECTED,
    SI_MPU_CONNECTED,
    SI_MPU_FOUND
} si_mpu_status_t;

struct qbuf {
    float w; float x; float y; float z;
};

struct v16buf {
    int16_t storage[3];
};

struct vfloatbuf {
    float storage[3];
};

typedef struct si_device_state {

    uint8_t devid = 0;

    si_mpu_status_t mpu_status;
    si_mpu_status_t last_mpu_status;

    uint8_t gyro_flags;
    uint8_t device_flags;

    uint8_t srate;

    uint16_t mpu_expected_packet_size = 16;

    uint64_t main_clk_tmt;
    uint64_t mpu_sample_tmt;

    uint32_t interrupt_cnt;
    uint32_t read_cnt;

    si_serial_t* serial;

    qbuf q_raw;

    qbuf q_cal;
    qbuf q_cal_L;
    qbuf q_cal_R;

    qbuf q_grav_cal;
    qbuf q_idle_cal;
    qbuf q_idle_conj;

    v16buf g_raw;
    vfloatbuf g_grv_idle;
    vfloatbuf v_grv_cal;

    volatile uint8_t data_ready = 0;

    MPU6050* mpu;

} si_device_state_t;

void si_gy_reset();

void si_gy_prepare(si_device_state_t*, si_serial_t*, MPU6050*);

void si_gyro_check(MPU6050* mpu, si_device_state* state);
void si_gyro_init(MPU6050* mpu, si_device_state_t* st);

void si_interrupt();

void si_write_sample(si_device_state_t*);

void si_gy_calibrate(MPU6050* mpu,
                     si_device_state_t* st,
                     si_serial_t* serial,
                     uint8_t lcnt);

void si_gy_reset_orientation(si_device_state_t* d);
void si_gy_init_begin(si_device_state_t* d);
void si_gy_init_finish(si_device_state_t* d);

void si_gy_run(MPU6050* mpu, si_device_state_t* state, si_serial_t*);

const uint8_t* si_gy_on_req(void*, si_gy_values_t, const uint8_t*);
void si_gy_on_set(void*, si_gy_values_t, const uint8_t*);
void si_gy_on_notify(void*, si_gy_values_t);