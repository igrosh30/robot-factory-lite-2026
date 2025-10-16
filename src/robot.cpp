#include <Arduino.h>
#include "robot.h"
#include <math.h>
#include "pico4drive.h"

robot_t robot;

template <typename T>
int sign(T val)
{
  return (T(0) < val) - (val < T(0));
}

robot_t::robot_t()
{
  stoped = false;
  wheel_dist = 0.125;
  wheel_radius = 0.0689 / 2;
  dv_max = 5;
  dw_max = 10;
  dt = 0.04;

  p1e = 0;
  p2e = 0;

  follow_k = -0.15;
  follow_v = 0.20;

  i_lambda = 0;
  led = 0;

}

void robot_t::odometry(void)
{
  // Estimate wheels speed using the encoders
  w1e = enc1 * (TWO_PI / (64.0 * 2.0 * 1920.0));
  w2e = enc2 * (TWO_PI / (64.0 * 2.0 * 1920.0));

  v1e = w1e * wheel_radius;//wheel_radius is a paramether that we'll need to estimate so we have a good aproximation
  v2e = w2e * wheel_radius;

  p1e += v1e * dt;
  p2e += v2e * dt;

  // Estimate robot speed
  ve = (v1e + v2e) / 2.0;
  we = (v1e - v2e) / wheel_dist;

  // Estimate the distance and the turn angle
  ds = ve * dt;
  dtheta = we * dt;

  // Estimate pose
  xe += ds * cos(thetae + dtheta / 2);
  ye += ds * sin(thetae + dtheta / 2);
  thetae = thetae + dtheta;

  // Relative displacement
  rel_s += ds;
  rel_theta += dtheta;
}

void robot_t::setRobotVW(float Vnom, float Wnom)
{
  v_req = Vnom;
  w_req = Wnom;
}

void robot_t::accelerationLimit(void)
{
  float dv = v_req - v;
  dv = constrain(dv, -dv_max, dv_max);
  v += dv;

  float dw = w_req - w;
  dw = constrain(dw, -dw_max, dw_max);
  w += dw;
}

void robot_t::calcMotorsVoltage(void)
{
  if (control_mode == cm_voltage)
  {
    u1 = u1_req;
    u2 = u2_req;
    u3 = u3_req;
    u4 = u4_req;

    return;
  }
  else if (control_mode == cm_pid)
  {
    w1ref = w1_req;
    w2ref = w2_req;
  }
  else if (control_mode == cm_kinematics)
  {
    v1ref = v + w * wheel_dist / 2;
    v2ref = v - w * wheel_dist / 2;

    w1ref = v1ref / wheel_radius;
    w2ref = v2ref / wheel_radius;
  }
  else if (control_mode == cm_pos)
  {
    /*
    p1ref = p1_req;
    PID[0].last_pe = PID[0].pos_error;
    PID[0].pos_error = p1ref - p1e;
    w1ref = PID[0].ppars->Kc_p * PID[0].pos_error + PID[0].ppars->Kd_p * (PID[0].pos_error - PID[0].last_pe) / PID[0].ppars->dt;
    u1 = PID[0].calc(w1ref, w1e) + sign(w1ref) * PID[0].ppars->dead_zone;
    */
  }

 
  // PWM_2 = u2 / battery_voltage * 255;
  // PWM_1 = u1 / battery_voltage * 255;
}