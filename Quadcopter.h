// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// Quadcopter.h
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#ifndef V_REP_EXT_QUADCOPTER_QUADCOPTER_H_INCLUDED
#define V_REP_EXT_QUADCOPTER_QUADCOPTER_H_INCLUDED

class Quadcopter
{
public:
  // Return true if a scene object is a quadcopter.
  static bool query(int obj);

  // Construct a quadcopter from its object ID.
  explicit Quadcopter(int obj);

  // Called when the simulation is started.
  void simulationStarted();

  // Called when the simulation is stopped.
  void simulationStopped();

  // Called when the simulation is stepped.
  void simulationStepped();

  // Set the particle velocity for motor "n".
  void setMotorParticleVelocity(int n, float v);

  // Return the particle velocity for motor "n".  Returns 0.0f if an
  // error occurs or "n" is out of range.
  float getMotorParticleVelocity(int n) const;

private:
  // The associated quadcopter object in the scene.
  int m_obj;

  // Unique ID for the quadcopter's base object.
  int m_uniqueID;

  // The quadcopter body object.
  int m_body;

  // The quadcopter target object.  This may not be used in the
  // future.
  int m_target;

  // Object IDs of the quadcopter's four motors.
  int m_motors[4];

  // Object IDs of the quadcopter's sensors and cameras.
  int m_cameraDown;
  int m_cameraFront;

  // Timestamp of the last camera image save.
  float m_last_save_time;

  // PID control state.
  void pidControl();

  float m_cumul;
  float m_lastE;
  float m_pAlphaE;
  float m_pBetaE;
  float m_psp0;
  float m_psp1;
  float m_prevEuler;
};

#endif   // !defined V_REP_EXT_QUADCOPTER_QUADCOPTER_H_INCLUDED
