#include "PID_controller.h"

PID_controller::PID_controller(Cubic_encoder &encoder, Cubic_motor &motor, int capableDuty, double Kp, double Ki, double Kd, double target, bool direction )
    : Kp(Kp), Ki(Ki), Kd(Kd), encoder(encoder), capableDuty(abs(limitInPermitedDutyRange(capableDuty))), motor(motor), target(target), direction(direction)
{
  preMicros = micros();
  preDiff = 0;
  integral = 0;
  preEncoderVal = (encoder >> encoderVal);
}

int PID_controller::compute(const bool ifPut, const bool ifPrint)
{
  /* Update dt */
  unsigned long nowMicros = micros();
  if (nowMicros < preMicros)
  {
    dt = MAX_MICROSECONDS - preMicros + nowMicros;
  }
  else
  {
    dt = nowMicros - preMicros;
  }
  dt /= 1000000.0;
  preMicros = nowMicros;

  /* Read the Encoder */
  encoder >> encoderVal;
  double velocity = (encoderVal - preEncoderVal) / dt;
  double diff = target - velocity;
  preEncoderVal = encoderVal;

  if(!direction){
  diff *= -1.0;
  }

  /* Compute duty */
  integral += (diff + preDiff) * dt / 2.0;
  duty += Kp * diff + Ki * integral + Kd * (diff - preDiff) / dt;
    Serial.print("duty: ");
    Serial.print(duty);
    Serial.print(",");

  duty = dutyLimiter();

  if (ifPut)
  {
    motor.put(duty);
  }

  if (ifPrint)
  {
    Serial.print("encoder: ");
    Serial.print(encoderVal);
    Serial.print(",target: ");
    Serial.print(target);
    Serial.print(",velocity: ");
    Serial.print(velocity);
    Serial.print(",diff: ");
    Serial.print(diff);
    Serial.print(",duty: ");
    Serial.println(duty);
  }

  return duty;
}