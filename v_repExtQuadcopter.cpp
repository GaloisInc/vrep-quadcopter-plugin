// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// v_repExtQuadcopter.cpp --- A simple quadcopter V-REP plugin.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "v_repLib.h"
#include "v_repExtQuadcopter.h"

#include "Container.h"
#include "Quadcopter.h"

#define PLUGIN_VERSION 1

// Return the current directory as a C++ string.
static std::string get_cur_dir()
{
  std::vector<char> buf(32);

  for (;;) {
    if (getcwd(&buf[0], buf.size()) == NULL) {
      if (errno == ERANGE) {
        buf.resize(buf.size() * 2);
      } else {
        throw std::runtime_error("getcwd failed");
      }
    } else {
      return std::string(&buf[0], strlen(&buf[0]));
    }
  }
}

// Return the name of the V-REP shared library.
static std::string get_lib_name()
{
#if defined(_WIN32)
  return "v_rep.dll";
#elif defined (__linux)
  return "libv_rep.so";
#elif defined (__APPLE__)
  return "libv_rep.dylib";
#else
# error "Don't know V-REP shared library on this OS."
#endif
}

//////////////////////////////////////////////////////////////////////
// Initialization

// V-REP library handle we dynamically load.
static LIBRARY g_vrepLib;

// Container of quadcopters in the scene.
GenericContainer<Quadcopter> g_quadcopters;

// Dynamically load and bind V-REP functions from the shared library.
// Use an environment variable or look in the current directory.
// Returns zero on failure.
static int vrep_init()
{
  std::string vrep_lib;
  char *env;

  if ((env = getenv("VREP_LIB")) != NULL) {
    vrep_lib.assign(env);
  } else {
    vrep_lib  = get_cur_dir();
    vrep_lib += "/";
    vrep_lib += get_lib_name();
  }

  g_vrepLib = loadVrepLibrary(vrep_lib.c_str());
  if (g_vrepLib == NULL) {
    fprintf(stderr, "Error: Unable to find V-REP library: %s\n",
            vrep_lib.c_str());
    return 0;
  }

  if (getVrepProcAddresses(g_vrepLib) == 0) {
    fprintf(stderr, "Error: Unable to load V-REP library: %s\n",
            vrep_lib.c_str());
    unloadVrepLibrary(g_vrepLib);
    return 0;
  }

  return 1;
}

//////////////////////////////////////////////////////////////////////
// Plug-in Entry Points

// Initialize this plugin when the V-REP application is started.
unsigned char v_repStart(void *p_arg, int i_arg)
{
  vrep_init();
  srand48(time(NULL));

  simLockInterface(1);
  Quadcopter::init();
  simLockInterface(0);

  return PLUGIN_VERSION;
}

// Shut down when the V-REP application is terminating.
void v_repEnd(void)
{
  g_quadcopters.clear();
  unloadVrepLibrary(g_vrepLib);
}

// Bit fields set in the "sim_message_eventcallback_instancepass"
// flags when the scene is changed.
#define SCENE_CHANGED  0x17f

// Handle a message from the V-REP simulator.
void *v_repMessage(int msg, int *adata, void *data, int *reply)
{
  void *result = NULL;
  int error_mode;

  simLockInterface(1);
  simGetIntegerParameter(sim_intparam_error_report_mode, &error_mode);
  simSetIntegerParameter(sim_intparam_error_report_mode,
                         sim_api_errormessage_ignore);

  if (msg == sim_message_eventcallback_instancepass) {
    int  flags = adata[0];
    bool scene_changed = (flags & SCENE_CHANGED) != 0;

    if (scene_changed) {
      fprintf(stderr, "quadcopter: scene content changed\n");
      g_quadcopters.rebuild();
    }
  }

  if (msg == sim_message_eventcallback_moduleopen) {
    if (data == NULL || !strcasecmp("quadcopter", (char *)data)) {
      fprintf(stderr, "quadcopter: simulation started\n");
      g_quadcopters.call(&Quadcopter::simulationStarted);
    }
  }

  if (msg == sim_message_eventcallback_modulehandle) {
    if (data == NULL || !strcasecmp("quadcopter", (char *)data)) {
      g_quadcopters.call(&Quadcopter::simulationStepped);
    }
  }

  if (msg == sim_message_eventcallback_moduleclose) {
    if (data == NULL || !strcasecmp("quadcopter", (char *)data)) {
      fprintf(stderr, "quadcopter: simulation stopped\n");
      g_quadcopters.call(&Quadcopter::simulationStopped);
    }
  }

  simSetIntegerParameter(sim_intparam_error_report_mode, error_mode);
  simLockInterface(0);
  return result;
}
