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

static std::mt19937 g_gen;
static bool g_gen_initialized;

GPSPosition getGPSPosition(int obj, const GPSSimConfig& config)
{
  // Seed the random generator the first time we are called.
  if (!g_gen_initialized) {
    g_gen_initialized = true;
    std::random_device rd;
    g_gen.seed(rd());
  }

  GPSPosition result;
  float pos[3];

  if (simGetObjectPosition(obj, -1, pos) == -1)
    return result;

  // Apply noise to the position before converting to lat/lon.
  std::normal_distribution<> d(config.noiseMean, config.noiseStddev);
  pos[0] += d(g_gen);
  pos[1] += d(g_gen);
  pos[2] += d(g_gen);

  UTMUPS::Reverse(config.zone, config.isNorth,
                  config.originX + pos[0],
                  config.originY + pos[1],
                  result.lat, result.lon);

  result.altitude = config.originZ + pos[2];

  return result;
}
