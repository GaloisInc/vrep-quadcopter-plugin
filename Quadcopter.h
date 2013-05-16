// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// Quadcopter.h
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#ifndef V_REP_EXT_QUADCOPTER_QUADCOPTER_H_INCLUDED
#define V_REP_EXT_QUADCOPTER_QUADCOPTER_H_INCLUDED

#include "Container.h"
#include "PID.h"
#include "SimGPS.h"

class Quadcopter
{
public:
  // Container of all quadcopters in the simulation.
  static GenericContainer<Quadcopter> all;

  // Return true if a scene object is a quadcopter.
  static bool query(int obj);

  // Perform one-time initialization for the Quadcopter plugin.  This
  // registers Lua functions with the simulator.  Returns true on
  // success, false on failure.
  static bool init();

  // Construct a quadcopter from its object ID.
  explicit Quadcopter(int obj);

  // Called when the simulation is started.
  void simulationStarted();

  // Called when the simulation is stopped.
  void simulationStopped();

  // Called when the simulation is stepped.
  void simulationStepped();

  // Set the accelerometer and gyro tube IDs.
  void setAccelTube(int id) { m_accelTube = id; }
  void setGyroTube(int id)  { m_gyroTube  = id; }

  // Read sensor data from the quadcopter.
  void readSensors();

  // Place the (x, y, z) values (in m/sec^2 XXX verify) of the latest
  // accelerometer reading into "out".
  void getAccel(float *out)
  {
    out[0] = m_accel[0];
    out[1] = m_accel[1];
    out[2] = m_accel[2];
  }

  // Place the (x, y, z) values (in rad/sec XXX verify) of the latest
  // gyro reading into "out".
  void getGyro(float *out)
  {
    out[0] = m_gyro[0];
    out[1] = m_gyro[1];
    out[2] = m_gyro[2];
  }

  // Return the latest GPS position.
  GPSPosition getGPSPosition() const { return m_gpsPosition; }

  // Run the PID controller and get the 4 motor velocities.
  void pidControl(float *motors_out);

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

  // Communication tubes for the accelerometer and gyro.
  int m_accelTube;
  int m_gyroTube;

  // Timestamp of the last camera image save.
  float m_lastSaveTime;

  // Latest sensor readings, recorded in "readSensors".
  float m_accel[3];
  float m_gyro[3];
  GPSPosition m_gpsPosition;

  // Read data from the accelerometer and gyro sensors.
  bool readAccelData(float *data_out);
  bool readGyroData(float *data_out);

  // Log file containing sensor information in CSV format.
  FILE *m_csvFile;

  // Simulated GPS sensor.
  GPSSimSensor m_gps;

  PID m_vertPID;
  PID m_alphaStabPID;
  PID m_alphaMovePID;
  PID m_betaStabPID;
  PID m_betaMovePID;
  PID m_rotPID;
};

#endif   // !defined V_REP_EXT_QUADCOPTER_QUADCOPTER_H_INCLUDED
