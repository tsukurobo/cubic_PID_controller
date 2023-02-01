#include "PID_controller.h"

PID_controller::PID_controller(Cubic_encoder &encoder, Cubic_motor &motor, int capableDuty, double Kp, double Ki, double Kd, double target, bool encoderDirection , int PPR)
: Kp(Kp), Ki(Ki), Kd(Kd), encoder(encoder), capableDuty(abs(limitInPermitedDutyRange(capableDuty))), motor(motor), target(target), direction(encoderDirection), PPR(PPR)
{
  preMicros = micros();
  preDiff = 0;
  integral = 0;
  preEncoderVal = (encoder >> encoderVal);
}

int PID_controller::compute_PID(const bool ifPut, const bool ifPrint){

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


  if(!direction){
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
    Serial.print(",diff: ");
    Serial.print(diff);
    Serial.print(",duty: ");
    Serial.println(duty);
  }
  return duty;
}

int PID_velocity_controller::compute(const bool ifPut, const bool ifPrint)
{
  int32_t encoderVal = getEncoderVal();
  int32_t preEncoderVal = getPreEncoderVal();
  double velocity = (encoderVal - preEncoderVal) / dt;
  setDiff(getTarget() - velocity);
  setPreEncoderVal(encoderVal);

  int duty = compute_PID(ifPut, ifPrint);

  if (ifPrint)
  {
    Serial.print(",velocity: ");
    Serial.print(velocity);
  }

  return duty;
}


int PID_position_controller::compute(const bool ifPut, const bool ifPrint)
{
  int32_t encoderVal =getEncoderVal();
  setDiff(getTarget() - encoderVal);

  return compute_PID(ifPut, ifPrint);
}