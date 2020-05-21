

#include "gyro.h"
#include "shared/serial.h"
#include <Arduino.h>

si_serial_t serial;
si_device_state_t device;
MPU6050 mpu;

void setup()
{
    digitalWrite(SI_SELF_RESET_PIN, HIGH);

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);

    si_serial_init(&Serial,
                   &serial,
                   &device,
                   { si_gy_on_req, si_gy_on_set, si_gy_on_notify });

    si_gy_prepare(&device, &serial, &mpu);
}

void loop()
{
    si_gy_run(&mpu, &device, &serial);
    si_serial_run(&serial);
}