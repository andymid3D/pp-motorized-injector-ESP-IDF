#ifndef MOULD_H
#define MOULD_H

#include <cstdint>

enum InjectProfileMode : uint8_t { INJECT_MODE_2D = 0, INJECT_MODE_3D = 1 };

struct actualMouldParams {
  char mouldName[20];

  float fillVolume;
  float fillSpeed;
  float fillPressure;

  float packVolume;
  float packSpeed;
  float packPressure;
  float packTime;

  float fillTrapAccel;
  float fillTrapDecel;
  float packTrapAccel;
  float packTrapDecel;

  uint8_t injectMode; // INJECT_MODE_2D or INJECT_MODE_3D
  float injectTorque; // used when injectMode == INJECT_MODE_3D
};

using actualMouldParams_t = actualMouldParams;

#endif // MOULD_H
