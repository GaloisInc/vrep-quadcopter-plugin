// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// v_repExtQuadcopter.h --- A quadcopter V-REP plugin.
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#ifndef V_REP_EXT_QUADCOPTER_H_INCLUDED
#define V_REP_EXT_QUADCOPTER_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

// Initialize this plugin when the V-REP application is started.
unsigned char v_repStart(void *p_arg, int i_arg);

// Shut down when the V-REP application is terminating.
void v_repEnd(void);

// Handle a message from the V-REP simulator.
void *v_repMessage(int msg, int *adata, void *data, int *reply);

#ifdef __cplusplus
}
#endif

#endif   // !defined V_REP_EXT_QUADCOPTER_H_INCLUDED
