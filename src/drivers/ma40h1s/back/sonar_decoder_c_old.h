/*
 * File: sonar_decoder_c.h
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 03-Jul-2018 15:57:57
 */

#ifndef SONAR_DECODER_C_H
#define SONAR_DECODER_C_H

/* Include Files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rtwtypes.h"
#include "sonar_decoder_c_types.h"

/* Function Declarations */
extern unsigned short sonar_decoder_c(short wave[10000], unsigned short N, float
  dt_1, short lim_1, float lim_2);
extern void sonar_decoder_c_initialize(void);
extern void sonar_decoder_c_terminate(void);

#endif

/*
 * File trailer for sonar_decoder_c.h
 *
 * [EOF]
 */
