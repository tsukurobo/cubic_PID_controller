#include <Arduino.h>
#include "cubic.ver1.8.h"
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
    static PID_velocity_controller controller(encoder, motor, 100, 0.002, 0.6, 0.0003, -5000.0, false, 512);
    controller.compute(true, true);
    Cubic_motor::send();

    // /*
        static int i = 0;
        if (i == 200)
        {
            controller.setTarget(900.0);
            i = -200;
        }
        if (i == 0)
        {
            controller.setTarget(-600.0);
        }
        i++;
    //  */
    /*
        static bool flag = 1;
        if (flag)
        {
            controller.setTargetRotationPerSecond(1.0);
            flag = 0;
        }
    */

    delay(5);
}