#pragma once
#include <cstdlib>
inline int RandomInt(int min, int max){
      int d = max - min + 1;
      return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
  }