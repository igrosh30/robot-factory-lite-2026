#ifndef trajectories_H
#define trajectories_H

#include "Arduino.h"
#include "robot.h"

class trajectory_t
{
  public:
    float xi, yi;
    float xt, yt, thetat;
    float v_nom, w_nom;
    float cx, cy;
    float e_xy, e_angle;
    float ktheta;
    
    trajectory_t();
    
    void set_theta(void);
    void goto_xy(void);
    void follow_line(void);
    void follow_circle(void);
};

float sqr(float x);
float norm(float x, float y);
void normalize(float& x, float& y);

float dist(float x0, float y0, float x1, float y1);
float normalize_angle(float ang);
float dif_angle(float a0, float a1);

extern trajectory_t traj;

#endif // trajectories_H
