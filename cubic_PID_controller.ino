#include <Arduino.h>
#include "Cubic1.7.h"
#include "PID_controller.h"
Cubic_encoder encoder;
Cubic_motor motor;

void setup()
{
    motor.begin(0);
    encoder.begin(0);
    Serial.begin(115200);
}

void loop()
{
    static PID_controller controller(encoder,motor,200,0.3,0.0003,0.003,200.0);
    controller.compute(false, true);
    delay(4);
}