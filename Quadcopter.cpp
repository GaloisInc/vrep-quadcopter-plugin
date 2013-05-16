// -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*-
//
// Quadcopter.cpp
//
// Copyright (C) 2013, Galois, Inc.
// All Rights Reserved.
//

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <deque>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "v_repLib.h"
#include "Container.h"
#include "PID.h"
#include "Quadcopter.h"
#include "SimGPS.h"

// Header number for our custom data.
#define DATA_ID 1000

// Field IDs for our custom data.
#define FIELD_QUADCOPTER     0
#define FIELD_MOTOR_0        1
#define FIELD_MOTOR_1        2
#define FIELD_MOTOR_2        3
#define FIELD_MOTOR_3        4
#define FIELD_CAMERA_DOWN    5
#define FIELD_CAMERA_FRONT   6
#define FIELD_BODY           7
#define FIELD_TARGET         8

// Our custom data is stored in the same format as the V-REP plug-in
// tutorial:
//
// 1000,{field_id,field_len,int x field_len}*

//////////////////////////////////////////////////////////////////////
// Utilities
//
// These could potentially move into a separate module.

// Shorthand for a vector of bytes.
typedef std::vector<uint8_t> byte_vector;

// Mapping of field IDs to their data.
typedef std::map<uint32_t, byte_vector> CustomData;

// Exception thrown when a Lua argument error occurs.
struct LuaArgException : public std::exception
{
  LuaArgException(const std::string& msg)
    : m_msg(msg) {}

  virtual const char *what() const noexcept(true)
  {
    return m_msg.c_str();
  }

private:
  std::string m_msg;
};

// Retrieve the "n"th integer argument to a Lua function.  Throws an
// exception if the argument is invalid.
int getInputIntArg(SLuaCallBack *p, int n)
{
  if (p->inputArgCount <= n)
    throw LuaArgException("not enough arguments");

  if (p->inputArgTypeAndSize[n * 2] != sim_lua_arg_int)
    throw LuaArgException("wrong argument type");

  return p->inputInt[n];
}

// Read a little endian integer from an iterator.  If the distance
// between "start" and "end" is too small, this throws an exception.
// The "start" iterator is modified to point at the next byte
// following the integer that was read.
static uint32_t parseUint32(byte_vector::const_iterator& p,
                            byte_vector::const_iterator end)
{
  if (end - p < 4)
    throw std::runtime_error("custom data format error");

  uint32_t result = (((uint32_t) p[0] << 0)  |
                     ((uint32_t) p[1] << 8)  |
                     ((uint32_t) p[2] << 16) |
                     ((uint32_t) p[3] << 24));

  p += 4;
  return result;
}

// Parse a custom data buffer into a list of fields.
static CustomData parseCustomData(const std::vector<uint8_t>& buf)
{
  CustomData result;
  auto p = buf.begin();
  auto end = buf.end();

  while (p != end) {
    uint32_t id  = parseUint32(p, end);
    uint32_t len = parseUint32(p, end);
    result[id]   = byte_vector(p, p + len);
    p += len;
  }

  return result;
}

// Return true if an object contains a custom data field.
static bool hasCustomDataField(int obj, uint32_t field)
{
  int size = simGetObjectCustomDataLength(obj, DATA_ID);
  if (size <= 0)
    return false;

  std::vector<uint8_t> buf(size);
  simGetObjectCustomData(obj, DATA_ID, (simChar *)&buf[0]);
  CustomData data(parseCustomData(buf));

  return data.find(field) != data.end();
}

// Search an object tree for an object that has the specified custom
// data field.  Returns the first matching object ID or -1 if no child
// object with that field is found.  The search is performed in
// breadth-first order.
static int searchCustomDataField(int root, uint32_t field)
{
  std::deque<int> q;
  q.push_back(root);

  while (!q.empty()) {
    int obj = q.front();
    q.pop_front();

    if (hasCustomDataField(obj, field))
      return obj;

    int i = 0;
    for (;;) {
      int child = simGetObjectChild(obj, i++);
      if (child == -1)
        break;

      q.push_back(child);
    }
  }

  return -1;
}

// Print an object with a label for debugging.
static void printObjWithLabel(const std::string& name, int obj)
{
  fprintf(stderr, "%-12s id %d", name.c_str(), obj);

  if (obj != -1) {
    simChar *objName = simGetObjectName(obj);
    fprintf(stderr, " name '%s'", objName);
    simReleaseBuffer(objName);
  }

  fprintf(stderr, "\n");
}

#if 0
// Write a camera's depth buffer as a PPM image to a file.
static bool writeCameraDepthPPM(const std::string& filename, int obj)
{
  simInt size[2];
  simFloat *image;

  fprintf(stderr, "saving image to file '%s'...\n", filename.c_str());

  if (simGetVisionSensorResolution(obj, size) == -1) {
    fprintf(stderr, "getting camera resolution failed\n");
    return false;
  }

  if ((image = simGetVisionSensorDepthBuffer(obj)) == NULL) {
    fprintf(stderr, "getting camera image failed\n");
    return false;
  }

  int pixel_size = size[0] * size[1] * 3;
  std::vector<uint8_t> data(pixel_size);

  for (int i = 0, j = 0; i < pixel_size; i += 3, j++) {
    data[i+0] = (uint8_t)(image[j] * 255.0f);
    data[i+1] = (uint8_t)(image[j] * 255.0f);
    data[i+2] = (uint8_t)(image[j] * 255.0f);
  }

  FILE *f;
  if ((f = fopen(filename.c_str(), "wb")) == NULL) {
    fprintf(stderr, "saving image failed\n");
    return false;
  }

  fprintf(f, "P6 %d %d 255\n", size[0], size[1]);
  fwrite(&data[0], 1, pixel_size, f);
  fclose(f);

  return true;
}
#endif

//////////////////////////////////////////////////////////////////////
// Lua Functions

// Read sensor state for a quadcopter.
void simExtQuadcopterReadSensors(SLuaCallBack *p)
{
  simLockInterface(1);

  try {
    int id = getInputIntArg(p, 0);
    std::shared_ptr<Quadcopter> qc = Quadcopter::all.get(id);

    if (qc) {
      qc->readSensors();
    } else {
      simSetLastError("simExtQuadcopterReadSensors",
                      "quadcopter object not found");
    }
  } catch (LuaArgException& e) {
    simSetLastError("simExtQuadcopterReadSensors", e.what());
  }

  p->outputArgCount = 1;
  p->outputArgTypeAndSize = (simInt*)simCreateBuffer(1 * sizeof(simInt));
  p->outputArgTypeAndSize[0] = sim_lua_arg_int;

  p->outputInt = (simInt*)simCreateBuffer(1 * sizeof(simInt));
  p->outputInt[0] = 1;

  simLockInterface(0);
}

// Return the four motor velocities for a quadcopter.
void simExtQuadcopterGetMotorVelocities(SLuaCallBack *p)
{
  float motors[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

  simLockInterface(1);

  try {
    int id   = getInputIntArg(p, 0);
    std::shared_ptr<Quadcopter> qc = Quadcopter::all.get(id);

    if (qc) {
      qc->pidControl(motors);
    } else {
      simSetLastError("simExtQuadcopterGetMotorVelocities",
                      "quadcopter object not found");
    }
  } catch (LuaArgException& e) {
    simSetLastError("simExtQuadcopterGetMotorVelocities", e.what());
  }

  p->outputArgCount          = 1;
  p->outputArgTypeAndSize    = (simInt*)simCreateBuffer(2 * sizeof(simInt));
  p->outputArgTypeAndSize[0] = sim_lua_arg_float|sim_lua_arg_table;
  p->outputArgTypeAndSize[1] = 4;

  p->outputFloat = (simFloat*)simCreateBuffer(4 * sizeof(simFloat));
  for (int i = 0; i < 4; ++i)
    p->outputFloat[i] = motors[i];

  simLockInterface(0);
}

// Set the tube to read accelerometer data from.
void simExtQuadcopterSetAccelTube(SLuaCallBack *p)
{
  int result = -1;

  simLockInterface(1);

  try {
    int id   = getInputIntArg(p, 0);
    int tube = getInputIntArg(p, 1);
    std::shared_ptr<Quadcopter> qc = Quadcopter::all.get(id);

    if (qc) {
      fprintf(stderr, "setting accel tube for copter %d to %d\n", id, tube);
      qc->setAccelTube(tube);
      result = 1;
    } else {
      simSetLastError("simExtQuadcopterSetAccelTube",
                      "quadcopter object not found");
    }
  } catch (LuaArgException& e) {
    simSetLastError("simExtQuadcopterSetAccelTube", e.what());
  }

  p->outputArgCount          = 1;
  p->outputArgTypeAndSize    = (simInt*)simCreateBuffer(2 * sizeof(simInt));
  p->outputArgTypeAndSize[0] = sim_lua_arg_int;
  p->outputArgTypeAndSize[1] = 1;

  p->outputInt    = (simInt*)simCreateBuffer(sizeof(result));
  p->outputInt[0] = result;

  simLockInterface(0);
}

// Set the tube to read gyro data from.
void simExtQuadcopterSetGyroTube(SLuaCallBack *p)
{
  int result = -1;

  simLockInterface(1);

  try {
    int id   = getInputIntArg(p, 0);
    int tube = getInputIntArg(p, 1);
    std::shared_ptr<Quadcopter> qc = Quadcopter::all.get(id);

    if (qc) {
      fprintf(stderr, "setting gyro tube for copter %d to %d\n", id, tube);
      qc->setGyroTube(tube);
      result = 1;
    } else {
      simSetLastError("simExtQuadcopterSetGyroTube",
                      "quadcopter object not found");
    }
  } catch (LuaArgException& e) {
    simSetLastError("simExtQuadcopterSetGyroTube", e.what());
  }

  p->outputArgCount          = 1;
  p->outputArgTypeAndSize    = (simInt*)simCreateBuffer(2 * sizeof(simInt));
  p->outputArgTypeAndSize[0] = sim_lua_arg_int;
  p->outputArgTypeAndSize[1] = 1;

  p->outputInt    = (simInt*)simCreateBuffer(sizeof(result));
  p->outputInt[0] = result;

  simLockInterface(0);
}

//////////////////////////////////////////////////////////////////////
// Quadcopter Methods

GenericContainer<Quadcopter> Quadcopter::all;

bool Quadcopter::query(int obj)
{
  return hasCustomDataField(obj, FIELD_QUADCOPTER);
}

bool Quadcopter::init()
{
  int args1[] = { 2, sim_lua_arg_int, sim_lua_arg_int };
  simRegisterCustomLuaFunction(
    "simExtQuadcopterSetAccelTube",
    "number result=simExtQuadcopterSetAccelTube("
    "number quadcopterID, number accelTubeID)",
    args1, simExtQuadcopterSetAccelTube);

  int args2[] = { 2, sim_lua_arg_int, sim_lua_arg_int };
  simRegisterCustomLuaFunction(
    "simExtQuadcopterSetGyroTube",
    "number result=simExtQuadcopterSetGyroTube("
    "number quadcopterID, number gyroTubeID)",
    args2, simExtQuadcopterSetGyroTube);

  int args3[] = { 1, sim_lua_arg_int };
  simRegisterCustomLuaFunction(
    "simExtQuadcopterGetMotorVelocities",
    "table_4 result=simExtQuadcopterGetMotorVelocities("
    "number quadcopterID)",
    args3, simExtQuadcopterGetMotorVelocities);

  int args4[] = { 1, sim_lua_arg_int };
  simRegisterCustomLuaFunction(
    "simExtQuadcopterReadSensors",
    "number result=simExtQuadcopterReadSensors("
    "number quadcopterID)",
    args4, simExtQuadcopterReadSensors);

  return true;
}

// GPS simulator configuration.  The UTM origins correspond to the
// following coordinates:
//
//   45d31'15"N 122d40'39"W
//
// We generate Gaussian noise with a mean of 0 and a standard
// deviation of 100cm.
static const GPSSimConfig g_gps_sim_config = {
  10,                           // utmZone
  true,                         // isNorth
  525187,                       // originX
  5040862,                      // originY
  10,                           // originZ
  0.0,                          // noiseMean
  0.1,                          // noiseStddev
};

Quadcopter::Quadcopter(int obj)
  : m_obj(obj), m_accelTube(-1), m_gyroTube(-1),
    m_gps(g_gps_sim_config),
    m_vertPID     ( 2.0f,   0.0f,  0.0f, -1.0f,   1.0f),
    m_alphaStabPID( 0.25f,  0.0f,  2.1f, -10.0f, 10.0f),
    m_alphaMovePID( 0.005f, 0.0f,  1.0f, -10.0f, 10.0f),
    m_betaStabPID (-0.25f,  0.0f, -2.1f, -10.0f, 10.0f),
    m_betaMovePID (-0.005f, 0.0f, -1.0f, -10.0f, 10.0f),
    m_rotPID      ( 0.1f,   0.0f,  2.0f, -1.0f,   1.0f)
{
  simGetObjectUniqueIdentifier(obj, &m_uniqueID);

  m_body        = searchCustomDataField(obj, FIELD_BODY);
  m_target      = searchCustomDataField(obj, FIELD_TARGET);
  m_cameraDown  = searchCustomDataField(obj, FIELD_CAMERA_DOWN);
  m_cameraFront = searchCustomDataField(obj, FIELD_CAMERA_FRONT);

  m_motors[0]   = searchCustomDataField(obj, FIELD_MOTOR_0);
  m_motors[1]   = searchCustomDataField(obj, FIELD_MOTOR_1);
  m_motors[2]   = searchCustomDataField(obj, FIELD_MOTOR_2);
  m_motors[3]   = searchCustomDataField(obj, FIELD_MOTOR_3);

  fprintf(stderr, "--- Found Quadcopter %d:\n", m_uniqueID);
  printObjWithLabel("Quadcopter:", m_obj);
  printObjWithLabel("Body:",       m_body);
  printObjWithLabel("Target:",     m_target);
  printObjWithLabel("Motor #1:",   m_motors[0]);
  printObjWithLabel("Motor #2:",   m_motors[1]);
  printObjWithLabel("Motor #3:",   m_motors[2]);
  printObjWithLabel("Motor #4:",   m_motors[3]);
  printObjWithLabel("Floor Cam:",  m_cameraDown);
  printObjWithLabel("Front Cam:",  m_cameraFront);
}

void Quadcopter::simulationStarted()
{
  m_lastSaveTime = 0;
  m_vertPID.reset();
  m_alphaStabPID.reset();
  m_alphaMovePID.reset();
  m_betaStabPID.reset();
  m_betaMovePID.reset();
  m_rotPID.reset();

  m_accel[0] = 0.0f;
  m_accel[1] = 0.0f;
  m_accel[2] = 0.0f;

  m_gyro[0] = 0.0f;
  m_gyro[1] = 0.0f;
  m_gyro[2] = 0.0f;

  char filename[128];
  snprintf(filename, sizeof(filename),
           "quadrotor_%d_log.csv", m_obj);

  m_csvFile = fopen(filename, "w");

  if (m_csvFile) {
    fprintf(stderr, "Logging data to '%s'\n", filename);
    fprintf(m_csvFile,
            "quadrotorID,time,latitude,longitude,altitude,"
            "accelX,accelY,accelZ,gyroX,gyroY,gyroZ\n");
  }
}

void Quadcopter::simulationStopped()
{
  if (m_csvFile != nullptr) {
    fclose(m_csvFile);
    m_csvFile = nullptr;
  }

  m_accelTube = -1;
  m_gyroTube  = -1;
}

bool Quadcopter::readAccelData(float *data_out)
{
  if (m_accelTube == -1)
    return false;

  if (simTubeStatus(m_accelTube, NULL, NULL) <= 0) {
    fprintf(stderr, "accel tube not connected\n");
    return false;
  }

  simInt len;
  simChar *result = simTubeRead(m_accelTube, &len);
  if (result == NULL) {
    fprintf(stderr, "reading from tube %d failed\n", m_accelTube);
    return false;
  }

  if (len != 12) {
    fprintf(stderr, "bad accel data, length %d\n", len);
    return false;
  }

  float *data = (float *)result;
  data_out[0] = data[0];
  data_out[1] = data[1];
  data_out[2] = data[2];
  simReleaseBuffer(result);

  return true;
}

bool Quadcopter::readGyroData(float *data_out)
{
  if (m_gyroTube == -1)
    return false;

  if (simTubeStatus(m_gyroTube, NULL, NULL) <= 0) {
    fprintf(stderr, "gyro tube not connected\n");
    return false;
  }

  simInt len;
  simChar *result = simTubeRead(m_gyroTube, &len);
  if (result == NULL) {
    fprintf(stderr, "reading from tube %d failed\n", m_gyroTube);
    return false;
  }

  if (len != 12) {
    fprintf(stderr, "bad gyro data, length %d\n", len);
    return false;
  }

  float *data = (float *)result;
  data_out[0] = data[0];
  data_out[1] = data[1];
  data_out[2] = data[2];
  simReleaseBuffer(result);

  return true;
}

// Read sensor data into our internal state.
void Quadcopter::readSensors()
{
  m_gpsPosition = m_gps.getGPSPosition(m_body);
  readAccelData(m_accel);
  readGyroData(m_gyro);

  float now = simGetSimulationTime();

  if (m_csvFile) {
    fprintf(m_csvFile,
            "%d,%.3f,%.10f,%.10f,%.10f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
            m_obj, now,
            m_gpsPosition.lat, m_gpsPosition.lon,
            m_gpsPosition.altitude,
            m_accel[0], m_accel[1], m_accel[2],
            m_gyro[0],  m_gyro[1],  m_gyro[2]);
  }
}

// Error checking macro for "pidControl".  This wraps calls to the
// V-REP API functions and prints an error message and returns from
// the current function if an error occurs.
#define CHECK(expr)                               \
  do {                                            \
    if ((expr) == -1) {                           \
      fprintf(stderr, "%s:%d: %s failed\n",       \
              __FILE__, __LINE__, # expr);        \
      return;                                     \
    }                                             \
  } while (0)

// Ported PID control from the Lua script.  Follows the quadcopter
// target object.
void Quadcopter::pidControl(float *motors_out)
{
  int d = m_body;               // to match lua script

  // Vertical control:
  float targetPos[3], pos[3], vel[3];
  float thrust;

  CHECK(simGetObjectPosition(m_target, -1, targetPos));
  CHECK(simGetObjectPosition(d, -1, pos));
  CHECK(simGetObjectVelocity(m_obj, vel, NULL));

  // NOTE: The magic number 5.335f is our estimated hover velocity?
  thrust = (5.335f - vel[2]) + m_vertPID.run(targetPos[2], pos[2]);

  // Horizontal control:
  float m[12];
  float vx[3] = { 1.0f, 0.0f, 0.0f };
  float vy[3] = { 0.0f, 1.0f, 0.0f };

  // stabilization:
  CHECK(simGetObjectMatrix(d, -1, m));
  CHECK(simTransformVector(m, vx));
  CHECK(simTransformVector(m, vy));
  float alphaCorr = m_alphaStabPID.run(vy[2], m[11]);
  float betaCorr  = m_betaStabPID.run(vx[2], m[11]);

  // move towards target:
  float sp[3];
  CHECK(simGetObjectPosition(m_target, d, sp));
  alphaCorr += m_alphaMovePID.run(sp[1], 0.0f);
  betaCorr  += m_betaMovePID.run(sp[0], 0.0f);

  // Rotational control:
  float rotCorr, euler[3];
  CHECK(simGetObjectOrientation(d, m_target, euler));
  rotCorr = m_rotPID.run(euler[2], 0.0f);

  motors_out[0] = thrust * (1.0f - alphaCorr + betaCorr + rotCorr);
  motors_out[1] = thrust * (1.0f - alphaCorr - betaCorr - rotCorr);
  motors_out[2] = thrust * (1.0f + alphaCorr - betaCorr + rotCorr);
  motors_out[3] = thrust * (1.0f + alphaCorr + betaCorr - rotCorr);
}

#undef CHECK

void Quadcopter::simulationStepped()
{
}
