/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CORE_H
#define CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Include */
#include <string.h>
#include "main.h"
#include "cJSON.h"
#include "servo.h"
#include "arm.h"
#include "mobile.h"
#include "adc.h"

/* FUNCTION (Prototype) DEFINITIONS */
float map(float x, float in_min, float in_max, float out_min, float out_max);
double max(double a, double b);
bool json_action(char *JSON_STRING, uint16_t token_size);

#ifdef __cplusplus
}
#endif

#endif /* CORE_H */

/*****END OF FILE*****/
