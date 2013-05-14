// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// SimGPS.cpp --- Simulated GPS position.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#include <GeographicLib/UTMUPS.hpp>
#include "v_repLib.h"
#include "SimGPS.h"

using GeographicLib::UTMUPS;

GPSSimSensor::GPSSimSensor(const GPSSimConfig& config)
  : m_config(config),
    m_noise(config.noiseMean, config.noiseStddev)
{
}

GPSPosition GPSSimSensor::getGPSPosition(int obj)
{
  GPSPosition result;
  float pos[3];

  if (simGetObjectPosition(obj, -1, pos) == -1)
    return result;

  // Apply noise to the position before converting to lat/lon.
  pos[0] += m_noise.get();
  pos[1] += m_noise.get();
  pos[2] += m_noise.get();

  UTMUPS::Reverse(m_config.zone, m_config.isNorth,
                  m_config.originX + pos[0],
                  m_config.originY + pos[1],
                  result.lat, result.lon);

  result.altitude = m_config.originZ + pos[2];

  return result;
}
