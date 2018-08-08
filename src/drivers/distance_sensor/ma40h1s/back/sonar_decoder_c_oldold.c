/*
 * File: sonar_decoder_c.c
 *
 * MATLAB Coder version            : 3.4
 * C/C++ source code generated on  : 05-Jul-2018 14:37:07
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
  unsigned short location;
  int sum;
  unsigned short i;
  float temp;
  float y2;
  unsigned short ix;
  boolean_T flag;
  float temp2;
  int i0;
  int n;
  short mtmp;
  int itmp;

  /*  signal = detrend(signal); */
  if (N > 8000) {
    N = 8000;
  }

  sum = 0;
  for (i = 1; i < 1001; i++) {
    sum += wave[i - 1];
  }

  temp = (float)sum / 1000.0F;
  for (i = 1; i <= N; i++) {
    wave[i - 1] = (short)roundf(fabsf((float)wave[i - 1] - temp));
  }

  /*  %加入低通滤波，int1为滤波后的信号，int2为1阶导数 */
  /*  y1 = int16(0); */
  /*  dt = dt_1; */
  /*  ix = uint16(2000); */
  /*  flag = true; */
  /*  for i = 2:N */
  /*      temp = single((wave(i) - y1))*dt; */
  /*      y1 = y1 + int16(temp); */
  /*      wave(i) = y1; */
  /*      if (i>uint16(50)) && (i<uint16(3000)) && flag */
  /*          if (wave(i) < uint16(lim_1)) && (abs(temp) < single(lim_2)) */
  /*              ix = i; */
  /*              flag = false; */
  /*          end */
  /*      end */
  /*  end */
  /*   */
  /*  %寻找第一最大值的位置 */
  /*  %丢掉int2前ix个数据，并用int4检测发射波是否全部丢弃 */
  /*  [~,location] = max(wave(ix:(N-uint16(10))));  */
  /*  location = location + ix; */
  /*  end */
  /* 加入低通滤波，int1为滤波后的信号，int2为1阶导数 */
  temp = 0.0F;
  y2 = 0.0F;
  ix = 1100;
  flag = true;
  temp2 = 0.0F;
  for (i = 1; i <= N; i++) {
    temp += ((float)wave[i - 1] - temp) * dt_1;
    wave[i - 1] = (short)roundf(temp);
    if ((i > 10) && (i < 1100) && flag) {
      temp2 = ((float)wave[i - 1] - y2) * dt_1;
      y2 += temp2;
    }

    if ((i > 500) && (i < 1100) && flag && (wave[i - 1] < (unsigned short)lim_1)
        && (fabsf(temp2) < lim_2)) {
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
  mtmp = wave[i0 - 1];
  itmp = -1;
  if ((sum - i0 > 1) && (1 < sum - i0)) {
    for (sum = 0; sum + 2 <= n; sum++) {
      if (wave[i0 + sum] > mtmp) {
        mtmp = wave[i0 + sum];
        itmp = sum;
      }
    }
  }

  if (mtmp > 45) {
    location = (unsigned short)((itmp + ix) + 2);
  } else {
    location = 1;
  }

  return location;
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
