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

private:
  // The associated quadcopter object in the scene.
  int m_obj;

  // Object IDs of the quadcopter's four motors.
  int m_motors[4];

  // Object IDs of the quadcopter's sensors and cameras.
  int m_cameraDown;
  int m_cameraFront;

  // Timestamp of the last camera image save.
  float m_last_save_time;
};

#endif   // !defined V_REP_EXT_QUADCOPTER_QUADCOPTER_H_INCLUDED
