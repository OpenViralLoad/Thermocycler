#include "Arduino.h"
#include "pid.h"

double M, kd, ki, kp, prev_time, prev_error, integral;

pid::pid() {
  M = 0;
  kp = 1;
  ki = 1;
  kd = 1;
  integral = 0;
  prev_error = 0;
  prev_time = -1;
}

void pid::setBiases(double p, double i, double d) {
  kp = p;
  ki = i;
  kd = d;
}

void pid::update(double sample, short target, double current_time) {
  if ( prev_time == -1 ) prev_time = current_time;
  double dt = current_time - prev_time; 
  double error = sample - target;
  double I = integral + error*dt;
  double D = (prev_error-error)/dt;
  M = error*kp + I*ki + D*kd;
  prev_error = error;
}

double pid::getM() {
  return M;
}
