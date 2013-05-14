// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// Noise.h --- Randomly generated Gaussian noise.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#ifndef V_REP_EXT_QUADCOPTER_NOISE_H_INCLUDED
#define V_REP_EXT_QUADCOPTER_NOISE_H_INCLUDED

#include <random>

class GaussianNoise
{
public:
  // Construct a noise generator given mean and standard deviation.
  GaussianNoise(double mean, double stddev)
    : m_gen(), m_dist(mean, stddev)
  {
    std::random_device rd;
    m_gen.seed(rd());
  }

  // Return a random noise value from this generator.
  double get()
  {
    return m_dist(m_gen);
  }

private:
  std::mt19937 m_gen;
  std::normal_distribution<double> m_dist;
};

#endif   // !defined V_REP_EXT_QUADCOPTER_NOISE_H_INCLUDED
