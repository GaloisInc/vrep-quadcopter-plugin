// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// PID.h --- Simple PID controller.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#ifndef V_REP_EXT_QUADCOPTER_PID_H_INCLUDED
#define V_REP_EXT_QUADCOPTER_PID_H_INCLUDED

template <class T>
T constrain(T x, T min, T max)
{
  return (x < min ? min : (x > max ? max : x));
}

class PID
{
public:
  // Construct a new PID controller given parameters.
  PID(float kp, float ki, float kd, float outMin, float outMax)
    : m_kp(kp), m_ki(ki), m_kd(kd),
      m_outMin(outMin), m_outMax(outMax),
      m_iTerm(0.0f), m_lastErr(0.0f)
  {
  }

  // Run the PID controller.  Assumes we are called at a constant time
  // step.
  float run(float setpoint, float input)
  {
    float error = setpoint - input;
    m_iTerm += (m_ki * error);
    m_iTerm  = constrain(m_iTerm, m_outMin, m_outMax);
    float dErr = error - m_lastErr;

    float output = m_kp * error + m_iTerm + m_kd * dErr;
    output = constrain(output, m_outMin, m_outMax);
    m_lastErr = error;

    return output;
  }

  void reset()
  {
    m_iTerm   = 0.0f;
    m_lastErr = 0.0f;
  }

private:
  float m_kp;                   // proportional gain
  float m_ki;                   // integral gain
  float m_kd;                   // derivative gain
  float m_outMin;               // minimum output value
  float m_outMax;               // maximum output value

  float m_iTerm;                // integral term
  float m_lastErr;              // last error value
};

#endif   // !defined V_REP_EXT_QUADCOPTER_PID_H_INCLUDED
