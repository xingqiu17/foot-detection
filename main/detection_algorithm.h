#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "detection_types.h"   

#ifdef __cplusplus
extern "C" {
#endif

bool step_update(const detection_data_t* det, int* step_total);
bool sit_lift_update(const detection_data* det);
void step_reset(void);
#ifdef __cplusplus
}
#endif
