/*
 * File: sonar_decoder_c.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 03-Jul-2018 15:57:57
 */

/* Include Files */
#include "sonar_decoder_c.h"

/* Function Definitions */

/*
 * dt_1 - 计算一阶导数的低通时间常数，默认0.02
 *  lim_1 - 丢弃头部数据的一阶导数阈值，默认50
 *  lim_2 - 丢弃头部数据的二阶导数阈值，默认1
 * Arguments    : short wave[10000]
 *                unsigned short N
 *                float dt_1
 *                short lim_1
 *                float lim_2
 * Return Type  : unsigned short
 */
unsigned short sonar_decoder_c(short wave[10000], unsigned short N, float dt_1,
  short lim_1, float lim_2)
{
  int sum;
  unsigned short i;
  short b_y1;
  unsigned short ix;
  boolean_T flag;
  float temp;
  int i0;
  int n;
  int itmp;

  /*  signal = detrend(signal); */
  sum = 0;
  for (i = 1; i <= N; i++) {
    sum += wave[i - 1];
  }

  for (i = 1; i <= N; i++) {
    wave[i - 1] = (short)roundf(fabsf((float)wave[i - 1] - (float)sum / (float)N));
  }

  /* 加入低通滤波，int1为滤波后的信号，int2为1阶导数 */
  b_y1 = 0;
  ix = 2000;
  flag = true;
  for (i = 2; i <= N; i++) {
    temp = (float)(short)(wave[i - 1] - b_y1) * dt_1;
    b_y1 += (short)roundf(temp);
    wave[i - 1] = b_y1;
    if ((i > 50) && (i < 3000) && flag && (wave[i - 1] < (unsigned short)lim_1) &&
        (fabsf(temp) < lim_2)) {
      ix = i;
      flag = false;
    }
  }

  /* 寻找第一最大值的位置 */
  /* 丢掉int2前ix个数据，并用int4检测发射波是否全部丢弃 */
  if (ix > (unsigned short)(N - 10U)) {
    i0 = 1;
    sum = 1;
  } else {
    i0 = ix;
    sum = (unsigned short)(N - 10U) + 1;
  }

  n = sum - i0;
  b_y1 = wave[i0 - 1];
  itmp = -1;
  if ((sum - i0 > 1) && (1 < sum - i0)) {
    for (sum = 0; sum + 2 <= n; sum++) {
      if (wave[i0 + sum] > b_y1) {
        b_y1 = wave[i0 + sum];
        itmp = sum;
      }
    }
  }

  return (unsigned short)((itmp + ix) + 2);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void sonar_decoder_c_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void sonar_decoder_c_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for sonar_decoder_c.c
 *
 * [EOF]
 */
