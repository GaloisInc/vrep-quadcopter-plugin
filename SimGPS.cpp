// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// SimGPS.cpp --- Simulated GPS position.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#include <random>
#include <GeographicLib/UTMUPS.hpp>
#include "v_repLib.h"
#include "SimGPS.h"

using GeographicLib::UTMUPS;

GPSSimSensor::GPSSimSensor(const GPSSimConfig& config)
  : m_config(config), m_gen(),
    m_d(config.noiseMean, config.noiseStddev)
{
  std::random_device rd;
  m_gen.seed(rd());
}

GPSPosition GPSSimSensor::getGPSPosition(int obj)
{
  GPSPosition result;
  float pos[3];

  if (simGetObjectPosition(obj, -1, pos) == -1)
    return result;

  // Apply noise to the position before converting to lat/lon.
  pos[0] += m_d(m_gen);
  pos[1] += m_d(m_gen);
  pos[2] += m_d(m_gen);

  UTMUPS::Reverse(m_config.zone, m_config.isNorth,
                  m_config.originX + pos[0],
                  m_config.originY + pos[1],
                  result.lat, result.lon);

  result.altitude = m_config.originZ + pos[2];

  return result;
}
