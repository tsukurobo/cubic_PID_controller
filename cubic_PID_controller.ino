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
    static int i = 0;
    static PID_controller controller(encoder,motor,200,0.003,0.000003,0.000003,-5000.0, false);
    controller.compute(true, true);
    Cubic_motor::send();

    if(i==200){
        controller.setTarget(5000.0);
        i = -200;
    }
    if(i==0){
        controller.setTarget(2000.0);
    }
        i++;
    delay(50);
}