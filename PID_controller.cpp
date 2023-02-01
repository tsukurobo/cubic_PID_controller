#include "PID_controller.h"

PID_controller::PID_controller(Cubic_encoder &encoder, Cubic_motor &motor, unsigned int capableDuty, double Kp, double Ki, double Kd, double target, bool encoderDirection, int PPR)
    : PID_controller(encoder, motor, capableDuty, -1 * capableDuty, Kp, Ki, Kd, target, encoderDirection, PPR) {}

PID_controller::PID_controller(Cubic_encoder &encoder, Cubic_motor &motor, int capableMaxDuty, int capableMinDuty, double Kp, double Ki, double Kd, double target, bool encoderDirection, int PPR)
    : Kp(Kp), Ki(Ki), Kd(Kd), encoder(encoder), capableMaxDuty(limitInPermitedDutyRange(capableMaxDuty)), capableMinDuty(min(limitInPermitedDutyRange(capableMinDuty), this->capableMaxDuty)), motor(motor), target(target), direction(encoderDirection), PPR(PPR)
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

  if (!direction)
  {
    diff *= -1.0;
  }

  /* Compute duty */
  integral += (diff + preDiff) * dt / 2.0;
  duty = Kp * diff + Ki * integral + Kd * (diff - preDiff) / dt;

  duty = dutyLimiter();

  preDiff = diff;

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