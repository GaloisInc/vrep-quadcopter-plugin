// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// SimGPS.h --- Simulated GPS position.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#ifndef V_REP_EXT_QUADCOPTER_SIM_GPS_H_INCLUDED
#define V_REP_EXT_QUADCOPTER_SIM_GPS_H_INCLUDED

#include "Noise.h"

// Position information returned by the simulated GPS.
struct GPSPosition
{
  GPSPosition() : lat(0.0), lon(0.0), altitude(0.0) {}

  double lat;                   // latitude (deg)
  double lon;                   // longitude (deg)
  double altitude;              // altitude (m)
};

// Configuration information for the simulated GPS.  We set the UTM
// zone and origin coordinates for position (0, 0, 0) in our world,
// and the characteristics of any simulated noise.
struct GPSSimConfig
{
  int    zone;                  // UTM zone number
  bool   isNorth;               // true if northern hemisphere
  double originX;               // origin X position (m)
  double originY;               // origin Y position (m)
  double originZ;               // origin Z position (m)
  double noiseMean;             // mean noise value
  double noiseStddev;           // standard deviation of noise
};

// GPS simulator object.  Requires a constant configuration object at
// construction time.
class GPSSimSensor
{
public:
  // Construct a simulated sensor.
  explicit GPSSimSensor(const GPSSimConfig& config);

  // Return the simulated GPS position of a simulator object given a
  // UTM zone and simulator configuration.
  GPSPosition getGPSPosition(int obj);

private:
  const GPSSimConfig& m_config;
  GaussianNoise m_noise;
};


#endif   // !defined V_REP_EXT_QUADCOPTER_SIM_GPS_H_INCLUDED
