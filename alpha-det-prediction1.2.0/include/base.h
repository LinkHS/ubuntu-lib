//
//  Basic Data Type Definitions, Macros and inline functions
//  Copyright (c) 2016 by Horizon Robotics Inc.
//  Author: Chang Huang (chang.huang@hobot.cc)
//

#ifndef __BASE_H__
#define __BASE_H__

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#ifndef WIN32
#include <stdint.h>
#endif

namespace hobot {

//
//  type definitions
//

#ifdef WIN32
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned __int16 uint16;
typedef unsigned __int32 uint32;
typedef unsigned __int64 uint64;
typedef __int16 int16;
typedef __int32 int32;
typedef __int64 int64;
#else
typedef unsigned char uchar;
typedef unsigned int uint;
typedef uint64_t uint64;
typedef uint32_t uint32;
typedef int64_t int64;
typedef int32_t int32;
typedef int16_t int16;
typedef uint16_t uint16;
#endif

//
//  macros
//
#define ERROR_INFO(format, ...) \
{   \
  fprintf(stderr, format, ##__VA_ARGS__); \
  fprintf(stderr, "Error happens at line %d of %s\n", __LINE__, __FILE__); \
  exit(-1);   \
}

#define ERROR_IF(condition, format, ...) \
{ \
  if (condition) {  \
      ERROR_INFO(format, ##__VA_ARGS__); \
  } \
}

#define REPORT_ERROR_POSITION ERROR_INFO("")

#ifndef MAX
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#endif

#ifndef MIN
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#endif

//
//  memory alignment
//

const int kMemAlignStep = 16;

inline int AlignedStepRoundUp(int x) {
  return (x + kMemAlignStep - 1) / kMemAlignStep * kMemAlignStep;
}

int MallocAlignedMemory(void *&memory, size_t memory_size);

void FreeAlignedMemory(void *&memory);

//
//  division & shift round up
//

inline int DivUp(int numer, int denom) {
  return (numer + denom - 1) / denom;
}

inline int RightShiftRoundUp(int val, int bits) {
  return (val + (1 << (bits - 1))) >> bits;
}

inline unsigned int RightShiftRoundUp(unsigned int val, unsigned int bits) {
  return (val + (1 << (bits - 1))) >> bits;
}

//
// templates for basic data structures, e.g., size and rect
//

template<class T>
struct TSize {
  T w, h;

  T CalArea() const { return w * h; }
  TSize() {};

  TSize(T w, T h) {
    this->w = w;
    this->h = h;
  }

  TSize<T> &operator=(const TSize<T> &size) {
    this->w = size.w;
    this->h = size.h;
    return *this;
  }
};

template<class T>
struct TSRect {
  T l, t, r, b; //left, top, right and bottom
  T CalArea() const {
    return (r - l) * (b - t);
  }
  TSRect() {};
  TSRect(T l, T t, T r, T b) {
    this->l = l;
    this->t = t;
    this->r = r;
    this->b = b;
  }

  TSRect<T> &operator=(const TSRect<T> &rect) {
    this->l = rect.l;
    this->t = rect.t;
    this->r = rect.r;
    this->b = rect.b;
    return *this;
  }

  T GetCX() {
    return (l + r) * 0.5f;
  }

  T GetCY() {
    return (t + b) * 0.5f;
  }

  T GetRadius() {
    return ((r - l) + (b - t)) * 0.5f * 0.5f;
  }
};

template<class T>
bool GetIntersectionRect(const TSRect<T> &r1, const TSRect<T> &r2,
                         TSRect<T> &rect) {
  rect = TSRect<T>(0, 0, 0, 0);
  T l = MAX(r1.l, r2.l);
  T r = MIN(r1.r, r2.r);

  if (l >= r) {
    return false;
  }

  T t = MAX(r1.t, r2.t);
  T b = MIN(r1.b, r2.b);

  if (t >= b) {
    return false;
  }

  rect.l = l;
  rect.r = r;
  rect.t = t;
  rect.b = b;
  return true;
}

template<class T>
float CalOverlapRatio(TSRect<T> &r1, TSRect<T> &r2, float &r1_ratio,
                      float &r2_ratio) {
  TSRect <T> r;

  if (!GetIntersectionRect(r1, r2, r)) {
    r1_ratio = r2_ratio = 0.0;
    return 0.0;
  } else {
    r1_ratio = r.CalArea() / r1.CalArea();
    r2_ratio = r.CalArea() / r2.CalArea();
    return float(r.CalArea()) / (r1.CalArea() + r2.CalArea() - r.CalArea());
  }
}

template<class T>
bool IsR1WithinR2Rect(TSRect<T> &r1, TSRect<T> &r2) {
  if (r1.l >= r2.l && r1.r <= r2.r && r1.t >= r2.t && r1.b <= r2.b) {
    return true;
  } else {
    return false;
  }
}


//
//  templates for stream in & out
//

template<class T>
bool ReadFromStream(std::istream &is, T &value) {
  is.read((char *) &value, sizeof(T));
  if (is)
    return true;
  else
    return false;
}

template<class T>
bool WriteToStream(std::ostream &os, const T &value) {
  os.write((char *) &value, sizeof(T));
  if (os)
    return true;
  else
    return false;
}

template<class T>
bool ReadFromStreamV(std::istream &is, std::vector<T> &vec) {
  uint32 sz;
  if (!ReadFromStream(is, sz)) {
    return false;
  }
  vec.resize(sz);
  if (sz != 0)
    is.read((char *) &vec[0], sizeof(T) * sz);
  if (is)
    return true;
  else
    return false;
}

template<class T>
bool WriteToStreamV(std::ostream &os, const std::vector<T> &vec) {
  uint32 sz = vec.size();
  WriteToStream(os, sz);
  if (sz != 0)
    os.write((char *) &vec[0], sizeof(T) * sz);
  if (os)
    return true;
  else
    return false;
}

template<class T>
bool ReadFromStreamVS(std::istream &is, std::vector<T *> &vec) {
  uint32 sz;
  if (!ReadFromStream(is, sz)) {
    return false;
  }
  vec.resize(sz);
  for (uint i = 0; i < sz; i++) {
    vec[i] = new T(is);
  }
  if (is)
    return true;
  else
    return false;
}

template<class T>
bool WriteToStreamVS(std::ostream &os, const std::vector<T *> &vec) {
  uint32 sz = vec.size();
  WriteToStream(os, sz);
  for (uint i = 0; i < sz; i++)
    vec[i]->ToStream(os);
  if (os)
    return true;
  else
    return false;
}

template<class T>
bool ReadFromStreamVS(std::istream &is, std::vector<T> &vec) {
  uint32 sz;
  if (!ReadFromStream(is, sz)) {
    return false;
  }
  vec.resize(sz);
  for (uint i = 0; i < sz; i++)
    vec[i].FromStream(is);
  if (is)
    return true;
  else
    return false;
}

template<class T>
bool WriteToStreamVS(std::ostream &os, const std::vector<T> &vec) {
  uint32 sz = vec.size();
  WriteToStream(os, sz);
  for (uint i = 0; i < sz; i++)
    vec[i].ToStream(os);
  if (os)
    return true;
  else
    return false;
}

} // namespace hobot

#endif
