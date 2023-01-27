#ifndef DATA_STRUCTS_H_
#define DATA_STRUCTS_H_
#include "Dense.h"

using namespace Eigen;

typedef struct {
    float time;
    Matrix <float, 7, 1>  sens_RS;      // RS data, ..., timestamp
} DATA_Xchange;
#endif