
#include "trajectories.h"

trajectory_t traj;


float sqr(float x)
{
  return x * x;
}

float norm(float x, float y)
{
  return sqrt(x * x + y * y);
}

void normalize(float& x, float& y)
{
  float norm = sqrt(x * x + y * y);
  if (norm == 0) return;
  x = x / norm;
  y = y / norm;
}


float dist(float x0, float y0, float x1, float y1)
{
  return sqrt(sqr(x1 - x0) + sqr(y1 - y0));
}

// Normalize angle to the range of [-π, π]
float normalize_angle(float angle)
{
  if (angle >= 0) {
    angle = fmod(angle + PI, TWO_PI);
    return angle - PI;
  } else {
    angle = fmod(-angle + PI, TWO_PI);
    return -(angle - PI);
  }}


float dif_angle(float a0, float a1)
{
  return normalize_angle(normalize_angle(a0) - normalize_angle(a1));
}


trajectory_t::trajectory_t()
{

}

void trajectory_t::set_theta(void)
{
  robot.v_req = 0;
  float e_theta = dif_angle(thetat, robot.thetae);
  //if (fabs(e_theta) <  radians(2)) e_theta = 0;
  robot.w_req = ktheta * e_theta;
}

void trajectory_t::goto_xy(void)
{
  robot.v_req = v_nom;
  robot.w_req = 0;
}


void trajectory_t::follow_line(void)
{
  robot.v_req = v_nom;
  robot.w_req = 0;
}


void trajectory_t::follow_circle(void)
{
  robot.v_req = v_nom;
  robot.w_req = w_nom;
}

