#include "buildover_fullstatemachine.h"
#include "rtwtypes.h"
#include "buildover_fullstatemachine_types.h"
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "buildover_fullstatemachine_private.h"
#include <uORB/topics/actuator_outputs_commanded.h>
#include "rt_defines.h"
#include "zero_crossing_types.h"

const uint8_T buildover_IN_Rotor_DEACCEL_prep = 7U;
const uint8_T buildover_ful_IN_FF_Preparation = 1U;
const uint8_T buildover_ful_IN_Forward_Flight = 2U;
const uint8_T buildover_full_IN_Rotor_DEACCEL = 6U;
const uint8_T buildover_full_IN_Rotor_Spin_Up = 8U;
const uint8_T buildover_fulls_IN_off_disarmed = 11U;
const uint8_T buildover_fullst_IN_Rotor_ACCEL = 5U;
const uint8_T buildover_fullsta_IN_Quad_Hover = 4U;
const uint8_T buildover_fullstat_IN_off_armed = 10U;
const uint8_T buildover_fullstatemach_IN_Kill = 3U;
const uint8_T buildover_fullstatemach_IN_VTOL = 9U;
const real_T buildover_fullstatemachi_period = 0.001;
B_buildover_fullstatemachine_T buildover_fullstatemachine_B;
DW_buildover_fullstatemachine_T buildover_fullstatemachine_DW;
PrevZCX_buildover_fullstatema_T buildover_fullstatemach_PrevZCX;
RT_MODEL_buildover_fullstatem_T buildover_fullstatemachine_M_ =
  RT_MODEL_buildover_fullstatem_T();
RT_MODEL_buildover_fullstatem_T *const buildover_fullstatemachine_M =
  &buildover_fullstatemachine_M_;
static void buildover_fullsta_Rotor_DEACCEL(boolean_T *arm_output, real_T
  *Tune_ID, real_T *slider_position, const uint16_T *uDLookupTable6, const
  real_T *Diff, real_T *w_ff_gain);
static void buildover_fullstatema_off_armed(boolean_T *arm_output, real_T
  *Tune_ID, real_T *slider_position, const uint16_T *uDLookupTable6, real_T
  *w_ff_gain, const real_T *CastToDouble);
static real_T buildover_fullstatemachine_norm(const real_T x[4]);
static void buildover_fullstat_expand_atan2(const real_T a_data[], const int32_T
  *a_size, const real_T b_data[], const int32_T *b_size, real_T c_data[],
  int32_T *c_size);
static void buildover_full_binary_expand_op(real_T in1[3], const int32_T
  in2_data[], const real_T in3_data[], const int32_T *in3_size, const real_T
  in4_data[], const int32_T *in4_size);
static void buildover_full_SystemCore_setup(px4_internal_block_PWM_buildo_T *obj,
  boolean_T varargin_1, boolean_T varargin_2);
real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T iLeft;
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

uint32_T plook_u32u16_bincka(uint16_T u, const uint16_T bp[], uint32_T maxIndex)
{
  uint32_T bpIndex;
  if (u <= bp[0U]) {
    bpIndex = 0U;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32u16(u, bp, maxIndex >> 1U, maxIndex);
  } else {
    bpIndex = maxIndex;
  }

  return bpIndex;
}

uint32_T binsearch_u32u16(uint16_T u, const uint16_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIdx;
  uint32_T bpIndex;
  uint32_T iRght;
  bpIdx = startIndex;
  bpIndex = 0U;
  iRght = maxIndex;
  while (iRght - bpIndex > 1U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      bpIndex = bpIdx;
    }

    bpIdx = (iRght + bpIndex) >> 1U;
  }

  return bpIndex;
}

static void buildover_fullsta_Rotor_DEACCEL(boolean_T *arm_output, real_T
  *Tune_ID, real_T *slider_position, const uint16_T *uDLookupTable6, const
  real_T *Diff, real_T *w_ff_gain)
{
  if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
    buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
      buildover_fullstatemach_IN_Kill;
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant_Value_fc;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant_Value_fc;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant_Value_fc;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant_Value_fc;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant_Value_fc;
    *slider_position = buildover_fullstatemachine_P.Constant_Value_fc;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant_Value_fc;
    *w_ff_gain = buildover_fullstatemachine_P.Constant_Value_fc;
    *Tune_ID = buildover_fullstatemachine_P.Constant13_Value;
    *arm_output = buildover_fullstatemachine_P.Constant1_Value_a;
  } else if ((*uDLookupTable6 == 7) && (*Diff == 0.0)) {
    buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
      buildover_fulls_IN_off_disarmed;
    *w_ff_gain = buildover_fullstatemachine_P.Constant12_Value_d;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant1_Value_o;
    *Tune_ID = buildover_fullstatemachine_P.Constant11_Value_c;
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant2_Value_o4;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant2_Value_o4;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant3_Value_l;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant4_Value_i;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant4_Value_i;
    *slider_position = buildover_fullstatemachine_P.Constant6_Value_b;
    *arm_output = buildover_fullstatemachine_P.Constant13_Value_f;
  } else if ((*uDLookupTable6 == 2) && (*Diff == 0.0)) {
    buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
      buildover_fullsta_IN_Quad_Hover;
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant3_Value_j;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant4_Value_g;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant5_Value_p;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant1_Value_dz;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant14_Value;
    *slider_position = buildover_fullstatemachine_P.Constant7_Value_e;
    *w_ff_gain = buildover_fullstatemachine_P.Constant2_Value_b;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant6_Value_p;
    *Tune_ID = buildover_fullstatemachine_P.Constant8_Value_c;
    *arm_output = buildover_fullstatemachine_P.Constant13_Value_p;
  } else if ((*uDLookupTable6 == 6) && (*Diff == 0.0)) {
    buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
      buildover_ful_IN_FF_Preparation;
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant_Value_e;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant4_Value;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant3_Value;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant5_Value;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant1_Value;
    *slider_position = buildover_fullstatemachine_P.Constant9_Value;
    *w_ff_gain = buildover_fullstatemachine_P.Constant6_Value;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant7_Value;
    *Tune_ID = buildover_fullstatemachine_P.Constant8_Value;
    *arm_output = buildover_fullstatemachine_P.Constant13_Value_j;
  } else {
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant6_Value_c;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant5_Value_l;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant4_Value_o;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant3_Value_e;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant14_Value_i;
    *slider_position = buildover_fullstatemachine_P.Constant_Value_ml;
    *w_ff_gain = buildover_fullstatemachine_P.Constant1_Value_n;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant2_Value_o;
    *Tune_ID = buildover_fullstatemachine_P.Constant7_Value_p;
    *arm_output = buildover_fullstatemachine_P.Constant13_Value_o;
  }
}

static void buildover_fullstatema_off_armed(boolean_T *arm_output, real_T
  *Tune_ID, real_T *slider_position, const uint16_T *uDLookupTable6, real_T
  *w_ff_gain, const real_T *CastToDouble)
{
  switch (*uDLookupTable6) {
   case 2:
    buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
      buildover_fullsta_IN_Quad_Hover;
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant3_Value_j;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant4_Value_g;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant5_Value_p;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant1_Value_dz;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant14_Value;
    *slider_position = buildover_fullstatemachine_P.Constant7_Value_e;
    *w_ff_gain = buildover_fullstatemachine_P.Constant2_Value_b;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant6_Value_p;
    *Tune_ID = buildover_fullstatemachine_P.Constant8_Value_c;
    *arm_output = buildover_fullstatemachine_P.Constant13_Value_p;
    break;

   case 4:
    buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
      buildover_full_IN_Rotor_Spin_Up;
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant6_Value_cy;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant_Value_as;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant5_Value_n;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant1_Value_g;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant3_Value_j5;
    *slider_position = buildover_fullstatemachine_P.Constant2_Value_oh;
    *w_ff_gain = buildover_fullstatemachine_P.Constant4_Value_d;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant11_Value;
    *Tune_ID = buildover_fullstatemachine_P.Constant7_Value_j;
    *arm_output = buildover_fullstatemachine_P.Constant13_Value_kn;
    break;

   default:
    if (*CastToDouble == 0.0) {
      buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
        buildover_fulls_IN_off_disarmed;
      *w_ff_gain = buildover_fullstatemachine_P.Constant12_Value_d;
      buildover_fullstatemachine_B.tau_ff_gain =
        buildover_fullstatemachine_P.Constant1_Value_o;
      *Tune_ID = buildover_fullstatemachine_P.Constant11_Value_c;
      buildover_fullstatemachine_B.lcb_speed =
        buildover_fullstatemachine_P.Constant2_Value_o4;
      buildover_fullstatemachine_B.rcb_speed =
        buildover_fullstatemachine_P.Constant2_Value_o4;
      buildover_fullstatemachine_B.mainrotor_control =
        buildover_fullstatemachine_P.Constant3_Value_l;
      buildover_fullstatemachine_B.leftcb_angle =
        buildover_fullstatemachine_P.Constant4_Value_i;
      buildover_fullstatemachine_B.rightcb_angle =
        buildover_fullstatemachine_P.Constant4_Value_i;
      *slider_position = buildover_fullstatemachine_P.Constant6_Value_b;
      *arm_output = buildover_fullstatemachine_P.Constant13_Value_f;
    } else if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
      buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
        buildover_fullstatemach_IN_Kill;
      buildover_fullstatemachine_B.lcb_speed =
        buildover_fullstatemachine_P.Constant_Value_fc;
      buildover_fullstatemachine_B.leftcb_angle =
        buildover_fullstatemachine_P.Constant_Value_fc;
      buildover_fullstatemachine_B.mainrotor_control =
        buildover_fullstatemachine_P.Constant_Value_fc;
      buildover_fullstatemachine_B.rcb_speed =
        buildover_fullstatemachine_P.Constant_Value_fc;
      buildover_fullstatemachine_B.rightcb_angle =
        buildover_fullstatemachine_P.Constant_Value_fc;
      *slider_position = buildover_fullstatemachine_P.Constant_Value_fc;
      buildover_fullstatemachine_B.tau_ff_gain =
        buildover_fullstatemachine_P.Constant_Value_fc;
      *w_ff_gain = buildover_fullstatemachine_P.Constant_Value_fc;
      *Tune_ID = buildover_fullstatemachine_P.Constant13_Value;
      *arm_output = buildover_fullstatemachine_P.Constant1_Value_a;
    } else {
      buildover_fullstatemachine_B.lcb_speed =
        buildover_fullstatemachine_P.Constant2_Value_h;
      buildover_fullstatemachine_B.rcb_speed =
        buildover_fullstatemachine_P.Constant2_Value_h;
      buildover_fullstatemachine_B.mainrotor_control =
        buildover_fullstatemachine_P.Constant3_Value_jc;
      buildover_fullstatemachine_B.rightcb_angle =
        buildover_fullstatemachine_P.Constant4_Value_e;
      buildover_fullstatemachine_B.leftcb_angle =
        buildover_fullstatemachine_P.Constant14_Value_f;
      *slider_position = buildover_fullstatemachine_P.Constant6_Value_f5;
      *w_ff_gain = buildover_fullstatemachine_P.Constant12_Value_j;
      buildover_fullstatemachine_B.tau_ff_gain =
        buildover_fullstatemachine_P.Constant1_Value_ly;
      *Tune_ID = buildover_fullstatemachine_P.Constant11_Value_a;
      *arm_output = buildover_fullstatemachine_P.Constant13_Value_d;
    }
    break;
  }
}

static real_T buildover_fullstatemachine_norm(const real_T x[4])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

static void buildover_fullstat_expand_atan2(const real_T a_data[], const int32_T
  *a_size, const real_T b_data[], const int32_T *b_size, real_T c_data[],
  int32_T *c_size)
{
  int32_T csz_idx_0;
  if (*b_size == 1) {
    csz_idx_0 = *a_size;
  } else {
    csz_idx_0 = 0;
  }

  *c_size = csz_idx_0;
  if (csz_idx_0 != 0) {
    c_data[0] = rt_atan2d_snf(a_data[0], b_data[0]);
  }
}

static void buildover_full_binary_expand_op(real_T in1[3], const int32_T
  in2_data[], const real_T in3_data[], const int32_T *in3_size, const real_T
  in4_data[], const int32_T *in4_size)
{
  int32_T loop_ub;
  loop_ub = *in4_size == 1 ? *in3_size : *in4_size;
  for (int32_T i = 0; i < loop_ub; i++) {
    in1[in2_data[0]] = -in3_data[0] * 2.0 * in4_data[0];
  }
}

static void buildover_full_SystemCore_setup(px4_internal_block_PWM_buildo_T *obj,
  boolean_T varargin_1, boolean_T varargin_2)
{
  uint16_T status;
  obj->isSetupComplete = false;
  obj->isInitialized = 1;
  obj->isMain = true;
  obj->pwmDevObj = MW_PWM_OUTPUT_MAIN_DEVICE_PATH;
  status = pwm_open(&obj->pwmDevHandler, obj->pwmDevObj,
                    &obj->actuatorAdvertiseObj, &obj->armAdvertiseObj);
  obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  obj->servoCount = 0;
  status = pwm_getServoCount(&obj->pwmDevHandler, &obj->servoCount);
  obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  if (varargin_1) {
    status = pwm_arm(&obj->pwmDevHandler, &obj->armAdvertiseObj);
    obj->isArmed = true;
  } else {
    status = pwm_disarm(&obj->pwmDevHandler, &obj->armAdvertiseObj);
    obj->isArmed = false;
  }

  obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  status = pwm_setPWMRate(&obj->pwmDevHandler, obj->isMain);
  obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  obj->channelMask = 63;
  status = pwm_setChannelMask(&obj->pwmDevHandler, obj->channelMask);
  obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  status = pwm_setFailsafePWM(&obj->pwmDevHandler, obj->servoCount,
    obj->channelMask, obj->isMain);
  obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  status = pwm_setDisarmedPWM(&obj->pwmDevHandler, obj->servoCount,
    obj->channelMask, obj->isMain, &obj->actuatorAdvertiseObj);
  obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  if (obj->isMain) {
    status = pwm_forceFailsafe(&obj->pwmDevHandler, static_cast<int32_T>
      (varargin_2));
    obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
    status = pwm_forceTerminateFailsafe(&obj->pwmDevHandler, 0);
    obj->errorStatus = static_cast<uint16_T>(obj->errorStatus | status);
  }

  obj->isSetupComplete = true;
}

void buildover_fullstatemachine_step(void)
{
  real_T b_idx_0;
  real_T out_tmp;
  real_T out_tmp_0;
  real_T out_tmp_1;
  real_T u0_tmp;
  real_T y_idx_2;
  int32_T f_size_idx_0;
  int32_T g_size_idx_0;
  int32_T h_size_idx_0;
  int32_T k;
  int32_T tmp_data;
  int32_T trueCount;
  uint16_T uDLookupTable6;
  boolean_T x[4];
  boolean_T NOT1;
  boolean_T exitg1;
  boolean_T mask1;
  ZCEventType zcEvent;
  mask1 = uORB_read_step(buildover_fullstatemachine_DW.obj_l.orbMetadataObj,
    &buildover_fullstatemachine_DW.obj_l.eventStructObj,
    &buildover_fullstatemachine_B.b_varargout_2_m, false, 5000.0);
  if (mask1) {
    buildover_fullstatemachine_B.In1_m =
      buildover_fullstatemachine_B.b_varargout_2_m;
  }

  if (buildover_fullstatemachine_B.In1_m.values[6] >
      buildover_fullstatemachine_P.WrapToZero1_Threshold) {
    buildover_fullstatemachine_B.CastToDouble =
      (buildover_fullstatemachine_P.Constant_Value_j == 0);
  } else {
    buildover_fullstatemachine_B.CastToDouble =
      (buildover_fullstatemachine_B.In1_m.values[6] == 0);
  }

  if (buildover_fullstatemachine_B.In1_m.values[7] >
      buildover_fullstatemachine_P.WrapToZero_Threshold) {
    buildover_fullstatemachine_B.CastToDouble16 =
      (buildover_fullstatemachine_P.Constant_Value_k == 0);
  } else {
    buildover_fullstatemachine_B.CastToDouble16 =
      (buildover_fullstatemachine_B.In1_m.values[7] == 0);
  }

  uDLookupTable6 =
    buildover_fullstatemachine_P.uDLookupTable6_tableData[plook_u32u16_bincka
    (buildover_fullstatemachine_B.In1_m.values[5],
     buildover_fullstatemachine_P.uDLookupTable6_bp01Data, 6U)];
  if (buildover_fullstatemachine_DW.Memory_PreviousInput >
      buildover_fullstatemachine_P.Saturation7_UpperSat) {
    buildover_fullstatemachine_B.Saturation =
      buildover_fullstatemachine_P.Saturation7_UpperSat;
  } else if (buildover_fullstatemachine_DW.Memory_PreviousInput <
             buildover_fullstatemachine_P.Saturation7_LowerSat) {
    buildover_fullstatemachine_B.Saturation =
      buildover_fullstatemachine_P.Saturation7_LowerSat;
  } else {
    buildover_fullstatemachine_B.Saturation =
      buildover_fullstatemachine_DW.Memory_PreviousInput;
  }

  buildover_fullstatemachine_B.RateLimiter1_f =
    buildover_fullstatemachine_B.Saturation -
    buildover_fullstatemachine_DW.PrevY;
  if (buildover_fullstatemachine_B.RateLimiter1_f >
      buildover_fullstatemachine_P.RateLimiter_RisingLim *
      buildover_fullstatemachi_period) {
    buildover_fullstatemachine_B.Saturation =
      buildover_fullstatemachine_P.RateLimiter_RisingLim *
      buildover_fullstatemachi_period + buildover_fullstatemachine_DW.PrevY;
  } else if (buildover_fullstatemachine_B.RateLimiter1_f <
             buildover_fullstatemachine_P.RateLimiter_FallingLim *
             buildover_fullstatemachi_period) {
    buildover_fullstatemachine_B.Saturation =
      buildover_fullstatemachine_P.RateLimiter_FallingLim *
      buildover_fullstatemachi_period + buildover_fullstatemachine_DW.PrevY;
  }

  buildover_fullstatemachine_DW.PrevY = buildover_fullstatemachine_B.Saturation;
  buildover_fullstatemachine_B.TSamp = buildover_fullstatemachine_B.Saturation *
    buildover_fullstatemachine_P.TSamp_WtEt;
  buildover_fullstatemachine_B.Diff = buildover_fullstatemachine_B.TSamp -
    buildover_fullstatemachine_DW.UD_DSTATE;
  buildover_fullstatemachine_B.TSamp_o =
    buildover_fullstatemachine_DW.Memory_PreviousInput_d *
    buildover_fullstatemachine_P.TSamp_WtEt_m;
  buildover_fullstatemachine_B.Diff_l = buildover_fullstatemachine_B.TSamp_o -
    buildover_fullstatemachine_DW.UD_DSTATE_d;
  buildover_fullstatemachine_B.TSamp_g =
    buildover_fullstatemachine_DW.Memory1_PreviousInput *
    buildover_fullstatemachine_P.TSamp_WtEt_e;
  buildover_fullstatemachine_B.Diff_g = buildover_fullstatemachine_B.TSamp_g -
    buildover_fullstatemachine_DW.UD_DSTATE_f;
  mask1 = uORB_read_step(buildover_fullstatemachine_DW.obj_b.orbMetadataObj,
    &buildover_fullstatemachine_DW.obj_b.eventStructObj,
    &buildover_fullstatemachine_B.b_varargout_2_m, false, 1.0);
  if (mask1) {
    buildover_fullstatemachine_B.In1_n =
      buildover_fullstatemachine_B.b_varargout_2_m;
  }

  if (buildover_fullstatemachine_B.In1_n.values[8] >
      buildover_fullstatemachine_P.WrapToZero1_Threshold_j) {
    NOT1 = (buildover_fullstatemachine_P.Constant_Value_p1 == 0);
  } else {
    NOT1 = (buildover_fullstatemachine_B.In1_n.values[8] == 0);
  }

  buildover_fullstatemachine_B.Switch = look1_binlxpw(static_cast<real_T>
    (buildover_fullstatemachine_B.In1_m.values[2]),
    buildover_fullstatemachine_P.uDLookupTable1_bp01Data_g,
    buildover_fullstatemachine_P.uDLookupTable1_tableData_b, 1U);
  if (buildover_fullstatemachine_B.Switch >
      buildover_fullstatemachine_P.Saturation_UpperSat) {
    buildover_fullstatemachine_B.Switch =
      buildover_fullstatemachine_P.Saturation_UpperSat;
  } else if (buildover_fullstatemachine_B.Switch <
             buildover_fullstatemachine_P.Saturation_LowerSat) {
    buildover_fullstatemachine_B.Switch =
      buildover_fullstatemachine_P.Saturation_LowerSat;
  }

  if (buildover_fullstatemachine_B.Switch >
      buildover_fullstatemachine_P.DeadZone_End) {
    buildover_fullstatemachine_B.Switch -=
      buildover_fullstatemachine_P.DeadZone_End;
  } else if (buildover_fullstatemachine_B.Switch >=
             buildover_fullstatemachine_P.DeadZone_Start) {
    buildover_fullstatemachine_B.Switch = 0.0;
  } else {
    buildover_fullstatemachine_B.Switch -=
      buildover_fullstatemachine_P.DeadZone_Start;
  }

  if (buildover_fullstatemachine_DW.is_active_c3_buildover_fullstat == 0U) {
    buildover_fullstatemachine_DW.is_active_c3_buildover_fullstat = 1U;
    buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
      buildover_fulls_IN_off_disarmed;
    buildover_fullstatemachine_B.w_ff_gain =
      buildover_fullstatemachine_P.Constant12_Value_d;
    buildover_fullstatemachine_B.tau_ff_gain =
      buildover_fullstatemachine_P.Constant1_Value_o;
    buildover_fullstatemachine_B.Diff_l =
      buildover_fullstatemachine_P.Constant11_Value_c;
    buildover_fullstatemachine_B.lcb_speed =
      buildover_fullstatemachine_P.Constant2_Value_o4;
    buildover_fullstatemachine_B.rcb_speed =
      buildover_fullstatemachine_P.Constant2_Value_o4;
    buildover_fullstatemachine_B.mainrotor_control =
      buildover_fullstatemachine_P.Constant3_Value_l;
    buildover_fullstatemachine_B.leftcb_angle =
      buildover_fullstatemachine_P.Constant4_Value_i;
    buildover_fullstatemachine_B.rightcb_angle =
      buildover_fullstatemachine_P.Constant4_Value_i;
    buildover_fullstatemachine_B.Diff_g =
      buildover_fullstatemachine_P.Constant6_Value_b;
    NOT1 = buildover_fullstatemachine_P.Constant13_Value_f;
  } else {
    switch (buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin) {
     case buildover_ful_IN_FF_Preparation:
      if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_Kill;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      } else if (uDLookupTable6 == 2) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullsta_IN_Quad_Hover;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant3_Value_j;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant4_Value_g;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_p;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant1_Value_dz;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant7_Value_e;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value_b;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant6_Value_p;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant8_Value_c;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_p;
      } else if (NOT1 && (fabs(buildover_fullstatemachine_B.Diff_g) < 100.0) &&
                 (fabs(buildover_fullstatemachine_B.Diff_l) < 100.0)) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_ful_IN_Forward_Flight;
        buildover_fullstatemachine_B.Switch = look1_binlxpw
          (buildover_fullstatemachine_B.Switch,
           buildover_fullstatemachine_P.uDLookupTable1_bp01Data,
           buildover_fullstatemachine_P.uDLookupTable1_tableData, 1U);
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_B.Switch;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_B.Switch;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_a;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant9_Value_e;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant10_Value;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant4_Value_m;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant1_Value_d;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_k;
      } else {
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_e;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant4_Value;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant3_Value;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant5_Value;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant1_Value;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant9_Value;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant6_Value;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant7_Value;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant8_Value;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_j;
      }
      break;

     case buildover_ful_IN_Forward_Flight:
      if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_Kill;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      } else if (!NOT1) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_ful_IN_FF_Preparation;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_e;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant4_Value;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant3_Value;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant5_Value;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant1_Value;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant9_Value;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant6_Value;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant7_Value;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant8_Value;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_j;
      } else {
        buildover_fullstatemachine_B.Switch = look1_binlxpw
          (buildover_fullstatemachine_B.Switch,
           buildover_fullstatemachine_P.uDLookupTable1_bp01Data,
           buildover_fullstatemachine_P.uDLookupTable1_tableData, 1U);
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_B.Switch;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_B.Switch;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_a;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant9_Value_e;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant10_Value;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant4_Value_m;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant1_Value_d;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_k;
      }
      break;

     case buildover_fullstatemach_IN_Kill:
      if ((buildover_fullstatemachine_B.CastToDouble16 == 0.0) &&
          (buildover_fullstatemachine_B.CastToDouble == 0.0)) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fulls_IN_off_disarmed;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value_d;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_o;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant11_Value_c;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_o4;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_o4;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_l;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_i;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_i;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant6_Value_b;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_f;
      } else {
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      }
      break;

     case buildover_fullsta_IN_Quad_Hover:
      switch (uDLookupTable6) {
       case 4:
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullst_IN_Rotor_ACCEL;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant6_Value_pc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant4_Value_mr;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_j;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant3_Value_g;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value_c;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fp;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_l;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value_j;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant7_Value_l;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_a;
        break;

       case 7:
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fulls_IN_off_disarmed;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value_d;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_o;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant11_Value_c;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_o4;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_o4;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_l;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_i;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_i;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant6_Value_b;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_f;
        break;

       default:
        if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
          buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
            buildover_fullstatemach_IN_Kill;
          buildover_fullstatemachine_B.lcb_speed =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.leftcb_angle =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.rcb_speed =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.rightcb_angle =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.Diff_g =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.tau_ff_gain =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.w_ff_gain =
            buildover_fullstatemachine_P.Constant_Value_fc;
          buildover_fullstatemachine_B.Diff_l =
            buildover_fullstatemachine_P.Constant13_Value;
          NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
        } else if (uDLookupTable6 == 6) {
          buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
            buildover_ful_IN_FF_Preparation;
          buildover_fullstatemachine_B.lcb_speed =
            buildover_fullstatemachine_P.Constant_Value_e;
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Constant4_Value;
          buildover_fullstatemachine_B.rcb_speed =
            buildover_fullstatemachine_P.Constant3_Value;
          buildover_fullstatemachine_B.rightcb_angle =
            buildover_fullstatemachine_P.Constant5_Value;
          buildover_fullstatemachine_B.leftcb_angle =
            buildover_fullstatemachine_P.Constant1_Value;
          buildover_fullstatemachine_B.Diff_g =
            buildover_fullstatemachine_P.Constant9_Value;
          buildover_fullstatemachine_B.w_ff_gain =
            buildover_fullstatemachine_P.Constant6_Value;
          buildover_fullstatemachine_B.tau_ff_gain =
            buildover_fullstatemachine_P.Constant7_Value;
          buildover_fullstatemachine_B.Diff_l =
            buildover_fullstatemachine_P.Constant8_Value;
          NOT1 = buildover_fullstatemachine_P.Constant13_Value_j;
        } else {
          buildover_fullstatemachine_B.lcb_speed =
            buildover_fullstatemachine_P.Constant3_Value_j;
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Constant4_Value_g;
          buildover_fullstatemachine_B.rcb_speed =
            buildover_fullstatemachine_P.Constant5_Value_p;
          buildover_fullstatemachine_B.rightcb_angle =
            buildover_fullstatemachine_P.Constant1_Value_dz;
          buildover_fullstatemachine_B.leftcb_angle =
            buildover_fullstatemachine_P.Constant14_Value;
          buildover_fullstatemachine_B.Diff_g =
            buildover_fullstatemachine_P.Constant7_Value_e;
          buildover_fullstatemachine_B.w_ff_gain =
            buildover_fullstatemachine_P.Constant2_Value_b;
          buildover_fullstatemachine_B.tau_ff_gain =
            buildover_fullstatemachine_P.Constant6_Value_p;
          buildover_fullstatemachine_B.Diff_l =
            buildover_fullstatemachine_P.Constant8_Value_c;
          NOT1 = buildover_fullstatemachine_P.Constant13_Value_p;
        }
        break;
      }
      break;

     case buildover_fullst_IN_Rotor_ACCEL:
      if (buildover_fullstatemachine_B.Diff == 0.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_VTOL;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant6_Value_pg;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_af;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_a;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_gr;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant12_Value_e;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_h;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_p;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value_bd;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant7_Value_e1;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_l;
      } else if ((uDLookupTable6 == 2) || (uDLookupTable6 == 6)) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_IN_Rotor_DEACCEL_prep;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant4_Value_l;
        buildover_fullstatemachine_B.Switch =
          buildover_fullstatemachine_P.Constant2_Value_i -
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE;
        if (buildover_fullstatemachine_B.Switch >
            buildover_fullstatemachine_P.Saturation6_UpperSat) {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Saturation6_UpperSat;
        } else if (buildover_fullstatemachine_B.Switch <
                   buildover_fullstatemachine_P.Saturation6_LowerSat) {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Saturation6_LowerSat;
        } else {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_B.Switch;
        }

        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_f;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant3_Value_p;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value_ch;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_a;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value_p;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_k;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant6_Value_f;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_n;
        buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE +=
          buildover_fullstatemachine_B.Switch *
          buildover_fullstatemachine_B.Switch *
          buildover_fullstatemachine_P.Constant7_Value_o *
          buildover_fullstatemachine_P.DiscreteTimeIntegrator_gainval;
        if (buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE >=
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_UpperSat) {
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE =
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_UpperSat;
        } else if (buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE <=
                   buildover_fullstatemachine_P.DiscreteTimeIntegrator_LowerSat)
        {
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE =
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_LowerSat;
        }
      } else if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_Kill;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      } else {
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant6_Value_pc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant4_Value_mr;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_j;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant3_Value_g;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value_c;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fp;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_l;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value_j;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant7_Value_l;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_a;
      }
      break;

     case buildover_full_IN_Rotor_DEACCEL:
      buildover_fullsta_Rotor_DEACCEL(&NOT1,
        &buildover_fullstatemachine_B.Diff_l,
        &buildover_fullstatemachine_B.Diff_g, &uDLookupTable6,
        &buildover_fullstatemachine_B.Diff,
        &buildover_fullstatemachine_B.w_ff_gain);
      break;

     case buildover_IN_Rotor_DEACCEL_prep:
      if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_Kill;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      } else if ((fabs(buildover_fullstatemachine_B.Diff_g) < 100.0) && (fabs
                  (buildover_fullstatemachine_B.Diff_l) < 100.0)) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_full_IN_Rotor_DEACCEL;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant6_Value_c;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant5_Value_l;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant4_Value_o;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant3_Value_e;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value_i;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_ml;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_n;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value_o;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant7_Value_p;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_o;
      } else {
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant4_Value_l;
        buildover_fullstatemachine_B.Switch =
          buildover_fullstatemachine_P.Constant2_Value_i -
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE;
        if (buildover_fullstatemachine_B.Switch >
            buildover_fullstatemachine_P.Saturation6_UpperSat) {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Saturation6_UpperSat;
        } else if (buildover_fullstatemachine_B.Switch <
                   buildover_fullstatemachine_P.Saturation6_LowerSat) {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Saturation6_LowerSat;
        } else {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_B.Switch;
        }

        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_f;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant3_Value_p;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value_ch;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_a;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value_p;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_k;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant6_Value_f;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_n;
        buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE +=
          buildover_fullstatemachine_B.Switch *
          buildover_fullstatemachine_B.Switch *
          buildover_fullstatemachine_P.Constant7_Value_o *
          buildover_fullstatemachine_P.DiscreteTimeIntegrator_gainval;
        if (buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE >=
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_UpperSat) {
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE =
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_UpperSat;
        } else if (buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE <=
                   buildover_fullstatemachine_P.DiscreteTimeIntegrator_LowerSat)
        {
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE =
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_LowerSat;
        }
      }
      break;

     case buildover_full_IN_Rotor_Spin_Up:
      if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_Kill;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      } else if (buildover_fullstatemachine_B.Diff == 0.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_VTOL;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant6_Value_pg;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_af;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_a;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_gr;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant12_Value_e;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_h;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_p;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value_bd;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant7_Value_e1;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_l;
      } else {
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant6_Value_cy;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_as;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_n;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant1_Value_g;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant3_Value_j5;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant2_Value_oh;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant4_Value_d;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant11_Value;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant7_Value_j;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_kn;
      }
      break;

     case buildover_fullstatemach_IN_VTOL:
      if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_Kill;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      } else if ((uDLookupTable6 == 2) || (uDLookupTable6 == 6) ||
                 (uDLookupTable6 == 7)) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_IN_Rotor_DEACCEL_prep;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant4_Value_l;
        buildover_fullstatemachine_B.Switch =
          buildover_fullstatemachine_P.Constant2_Value_i -
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE;
        if (buildover_fullstatemachine_B.Switch >
            buildover_fullstatemachine_P.Saturation6_UpperSat) {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Saturation6_UpperSat;
        } else if (buildover_fullstatemachine_B.Switch <
                   buildover_fullstatemachine_P.Saturation6_LowerSat) {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_P.Saturation6_LowerSat;
        } else {
          buildover_fullstatemachine_B.mainrotor_control =
            buildover_fullstatemachine_B.Switch;
        }

        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_f;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant3_Value_p;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value_ch;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_a;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value_p;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_k;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant6_Value_f;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_n;
        buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE +=
          buildover_fullstatemachine_B.Switch *
          buildover_fullstatemachine_B.Switch *
          buildover_fullstatemachine_P.Constant7_Value_o *
          buildover_fullstatemachine_P.DiscreteTimeIntegrator_gainval;
        if (buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE >=
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_UpperSat) {
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE =
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_UpperSat;
        } else if (buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE <=
                   buildover_fullstatemachine_P.DiscreteTimeIntegrator_LowerSat)
        {
          buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE =
            buildover_fullstatemachine_P.DiscreteTimeIntegrator_LowerSat;
        }
      } else {
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant6_Value_pg;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_af;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant5_Value_a;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_gr;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant12_Value_e;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_h;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_p;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant2_Value_bd;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant7_Value_e1;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_l;
      }
      break;

     case buildover_fullstat_IN_off_armed:
      buildover_fullstatema_off_armed(&NOT1,
        &buildover_fullstatemachine_B.Diff_l,
        &buildover_fullstatemachine_B.Diff_g, &uDLookupTable6,
        &buildover_fullstatemachine_B.w_ff_gain,
        &buildover_fullstatemachine_B.CastToDouble);
      break;

     default:
      if ((buildover_fullstatemachine_B.CastToDouble == 1.0) && (uDLookupTable6 ==
           1)) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstat_IN_off_armed;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_h;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_h;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_jc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_e;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant14_Value_f;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant6_Value_f5;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value_j;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_ly;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant11_Value_a;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_d;
      } else if (buildover_fullstatemachine_B.CastToDouble16 == 1.0) {
        buildover_fullstatemachine_DW.is_c3_buildover_fullstatemachin =
          buildover_fullstatemach_IN_Kill;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant_Value_fc;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant13_Value;
        NOT1 = buildover_fullstatemachine_P.Constant1_Value_a;
      } else {
        buildover_fullstatemachine_B.w_ff_gain =
          buildover_fullstatemachine_P.Constant12_Value_d;
        buildover_fullstatemachine_B.tau_ff_gain =
          buildover_fullstatemachine_P.Constant1_Value_o;
        buildover_fullstatemachine_B.Diff_l =
          buildover_fullstatemachine_P.Constant11_Value_c;
        buildover_fullstatemachine_B.lcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_o4;
        buildover_fullstatemachine_B.rcb_speed =
          buildover_fullstatemachine_P.Constant2_Value_o4;
        buildover_fullstatemachine_B.mainrotor_control =
          buildover_fullstatemachine_P.Constant3_Value_l;
        buildover_fullstatemachine_B.leftcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_i;
        buildover_fullstatemachine_B.rightcb_angle =
          buildover_fullstatemachine_P.Constant4_Value_i;
        buildover_fullstatemachine_B.Diff_g =
          buildover_fullstatemachine_P.Constant6_Value_b;
        NOT1 = buildover_fullstatemachine_P.Constant13_Value_f;
      }
      break;
    }
  }

  if (buildover_fullstatemachine_B.mainrotor_control >
      buildover_fullstatemachine_P.Switch_Threshold) {
    buildover_fullstatemachine_B.Switch =
      buildover_fullstatemachine_B.mainrotor_control;
  } else {
    buildover_fullstatemachine_B.Switch = look1_binlxpw
      (-buildover_fullstatemachine_B.Saturation,
       buildover_fullstatemachine_P.uDLookupTable5_bp01Data,
       buildover_fullstatemachine_P.uDLookupTable5_tableData, 1U);
    if (buildover_fullstatemachine_B.Switch >
        buildover_fullstatemachine_P.Saturation5_UpperSat) {
      buildover_fullstatemachine_B.Switch =
        buildover_fullstatemachine_P.Saturation5_UpperSat;
    } else if (buildover_fullstatemachine_B.Switch <
               buildover_fullstatemachine_P.Saturation5_LowerSat) {
      buildover_fullstatemachine_B.Switch =
        buildover_fullstatemachine_P.Saturation5_LowerSat;
    }
  }

  buildover_fullstatemachine_B.RateLimiter1_f =
    buildover_fullstatemachine_B.leftcb_angle -
    buildover_fullstatemachine_DW.PrevY_l;
  if (buildover_fullstatemachine_B.RateLimiter1_f >
      buildover_fullstatemachine_P.RateLimiter1_RisingLim *
      buildover_fullstatemachi_period) {
    buildover_fullstatemachine_B.RateLimiter1 =
      buildover_fullstatemachine_P.RateLimiter1_RisingLim *
      buildover_fullstatemachi_period + buildover_fullstatemachine_DW.PrevY_l;
  } else if (buildover_fullstatemachine_B.RateLimiter1_f <
             buildover_fullstatemachine_P.RateLimiter1_FallingLim *
             buildover_fullstatemachi_period) {
    buildover_fullstatemachine_B.RateLimiter1 =
      buildover_fullstatemachine_P.RateLimiter1_FallingLim *
      buildover_fullstatemachi_period + buildover_fullstatemachine_DW.PrevY_l;
  } else {
    buildover_fullstatemachine_B.RateLimiter1 =
      buildover_fullstatemachine_B.leftcb_angle;
  }

  buildover_fullstatemachine_DW.PrevY_l =
    buildover_fullstatemachine_B.RateLimiter1;
  buildover_fullstatemachine_B.RateLimiter1_f =
    buildover_fullstatemachine_B.rightcb_angle -
    buildover_fullstatemachine_DW.PrevY_f;
  if (buildover_fullstatemachine_B.RateLimiter1_f >
      buildover_fullstatemachine_P.RateLimiter1_RisingLim_p *
      buildover_fullstatemachi_period) {
    buildover_fullstatemachine_B.RateLimiter1_f =
      buildover_fullstatemachine_P.RateLimiter1_RisingLim_p *
      buildover_fullstatemachi_period + buildover_fullstatemachine_DW.PrevY_f;
  } else if (buildover_fullstatemachine_B.RateLimiter1_f <
             buildover_fullstatemachine_P.RateLimiter1_FallingLim_k *
             buildover_fullstatemachi_period) {
    buildover_fullstatemachine_B.RateLimiter1_f =
      buildover_fullstatemachine_P.RateLimiter1_FallingLim_k *
      buildover_fullstatemachi_period + buildover_fullstatemachine_DW.PrevY_f;
  } else {
    buildover_fullstatemachine_B.RateLimiter1_f =
      buildover_fullstatemachine_B.rightcb_angle;
  }

  buildover_fullstatemachine_DW.PrevY_f =
    buildover_fullstatemachine_B.RateLimiter1_f;
  u0_tmp = buildover_fullstatemachine_P.Gain_Gain * fabs
    (buildover_fullstatemachine_B.Diff) *
    buildover_fullstatemachine_B.tau_ff_gain;
  buildover_fullstatemachine_B.Saturation =
    -buildover_fullstatemachine_B.Saturation *
    -buildover_fullstatemachine_B.Saturation;
  buildover_fullstatemachine_B.Diff = look1_binlxpw
    (((buildover_fullstatemachine_P.Gain6_Gain *
       buildover_fullstatemachine_B.rcb_speed +
       buildover_fullstatemachine_P.Constant2_Value_p) + u0_tmp) +
     buildover_fullstatemachine_B.Saturation *
     buildover_fullstatemachine_P.Gain11_Gain *
     buildover_fullstatemachine_B.w_ff_gain,
     buildover_fullstatemachine_P.uDLookupTable4_bp01Data,
     buildover_fullstatemachine_P.uDLookupTable4_tableData, 1U);
  if (buildover_fullstatemachine_B.Diff >
      buildover_fullstatemachine_P.Saturation1_UpperSat) {
    buildover_fullstatemachine_B.Diff =
      buildover_fullstatemachine_P.Saturation1_UpperSat;
  } else if (buildover_fullstatemachine_B.Diff <
             buildover_fullstatemachine_P.Saturation1_LowerSat) {
    buildover_fullstatemachine_B.Diff =
      buildover_fullstatemachine_P.Saturation1_LowerSat;
  }

  buildover_fullstatemachine_B.Saturation = look1_binlxpw
    (((buildover_fullstatemachine_P.Gain7_Gain *
       buildover_fullstatemachine_B.lcb_speed +
       buildover_fullstatemachine_P.Constant2_Value_l) + u0_tmp) +
     buildover_fullstatemachine_B.Saturation *
     buildover_fullstatemachine_P.Gain11_Gain_j *
     buildover_fullstatemachine_B.w_ff_gain,
     buildover_fullstatemachine_P.uDLookupTable4_bp01Data_k,
     buildover_fullstatemachine_P.uDLookupTable4_tableData_b, 1U);
  if (buildover_fullstatemachine_B.Saturation >
      buildover_fullstatemachine_P.Saturation_UpperSat_a) {
    buildover_fullstatemachine_B.Saturation =
      buildover_fullstatemachine_P.Saturation_UpperSat_a;
  } else if (buildover_fullstatemachine_B.Saturation <
             buildover_fullstatemachine_P.Saturation_LowerSat_j) {
    buildover_fullstatemachine_B.Saturation =
      buildover_fullstatemachine_P.Saturation_LowerSat_j;
  }

  mask1 = uORB_read_step(buildover_fullstatemachine_DW.obj_m.orbMetadataObj,
    &buildover_fullstatemachine_DW.obj_m.eventStructObj,
    &buildover_fullstatemachine_B.b_varargout_2, false, 1.0);
  if (mask1) {
    buildover_fullstatemachine_B.In1 =
      buildover_fullstatemachine_B.b_varargout_2;
  }

  zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,
                     &buildover_fullstatemach_PrevZCX.SampleandHold_Trig_ZCE,
                     (buildover_fullstatemachine_DW.Memory_PreviousInput_g));
  if (zcEvent != NO_ZCEVENT) {
    buildover_fullstatemachine_B.In = buildover_fullstatemachine_B.In1.z;
  }

  mask1 = uORB_read_step(buildover_fullstatemachine_DW.obj_pt.orbMetadataObj,
    &buildover_fullstatemachine_DW.obj_pt.eventStructObj,
    &buildover_fullstatemachine_B.b_varargout_2_c, false, 1.0);
  if (mask1) {
    buildover_fullstatemachine_B.In1_b =
      buildover_fullstatemachine_B.b_varargout_2_c;
  }

  buildover_fullstatemachine_B.DataTypeConversion6[0] =
    buildover_fullstatemachine_B.In1_b.q[0];
  x[0] = rtIsNaN(static_cast<real_T>(buildover_fullstatemachine_B.In1_b.q[0]));
  buildover_fullstatemachine_B.DataTypeConversion6[1] =
    buildover_fullstatemachine_B.In1_b.q[1];
  x[1] = rtIsNaN(static_cast<real_T>(buildover_fullstatemachine_B.In1_b.q[1]));
  buildover_fullstatemachine_B.DataTypeConversion6[2] =
    buildover_fullstatemachine_B.In1_b.q[2];
  x[2] = rtIsNaN(static_cast<real_T>(buildover_fullstatemachine_B.In1_b.q[2]));
  buildover_fullstatemachine_B.DataTypeConversion6[3] =
    buildover_fullstatemachine_B.In1_b.q[3];
  x[3] = rtIsNaN(static_cast<real_T>(buildover_fullstatemachine_B.In1_b.q[3]));
  mask1 = false;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
    if (x[k]) {
      mask1 = true;
      exitg1 = true;
    } else {
      k++;
    }
  }

  if (mask1) {
    buildover_fullstatemachine_B.DataTypeConversion6[0] = 1.0;
    buildover_fullstatemachine_B.DataTypeConversion6[1] = 0.0;
    buildover_fullstatemachine_B.DataTypeConversion6[2] = 0.0;
    buildover_fullstatemachine_B.DataTypeConversion6[3] = 0.0;
  } else {
    buildover_fullstatemachine_B.aSinInput = buildover_fullstatemachine_norm
      (buildover_fullstatemachine_B.DataTypeConversion6);
    if (buildover_fullstatemachine_B.aSinInput < 0.1) {
      buildover_fullstatemachine_B.DataTypeConversion6[0] = 1.0;
      buildover_fullstatemachine_B.DataTypeConversion6[1] = 0.0;
      buildover_fullstatemachine_B.DataTypeConversion6[2] = 0.0;
      buildover_fullstatemachine_B.DataTypeConversion6[3] = 0.0;
    } else {
      buildover_fullstatemachine_B.DataTypeConversion6[0] =
        buildover_fullstatemachine_B.In1_b.q[0] /
        buildover_fullstatemachine_B.aSinInput;
      buildover_fullstatemachine_B.DataTypeConversion6[1] =
        buildover_fullstatemachine_B.In1_b.q[1] /
        buildover_fullstatemachine_B.aSinInput;
      buildover_fullstatemachine_B.DataTypeConversion6[2] =
        buildover_fullstatemachine_B.In1_b.q[2] /
        buildover_fullstatemachine_B.aSinInput;
      buildover_fullstatemachine_B.DataTypeConversion6[3] =
        buildover_fullstatemachine_B.In1_b.q[3] /
        buildover_fullstatemachine_B.aSinInput;
    }
  }

  buildover_fullstatemachine_B.aSinInput = 1.0 / sqrt
    (((buildover_fullstatemachine_B.DataTypeConversion6[0] *
       buildover_fullstatemachine_B.DataTypeConversion6[0] +
       buildover_fullstatemachine_B.DataTypeConversion6[1] *
       buildover_fullstatemachine_B.DataTypeConversion6[1]) +
      buildover_fullstatemachine_B.DataTypeConversion6[2] *
      buildover_fullstatemachine_B.DataTypeConversion6[2]) +
     buildover_fullstatemachine_B.DataTypeConversion6[3] *
     buildover_fullstatemachine_B.DataTypeConversion6[3]);
  buildover_fullstatemachine_B.w_ff_gain =
    buildover_fullstatemachine_B.DataTypeConversion6[0] *
    buildover_fullstatemachine_B.aSinInput;
  u0_tmp = buildover_fullstatemachine_B.DataTypeConversion6[1] *
    buildover_fullstatemachine_B.aSinInput;
  y_idx_2 = buildover_fullstatemachine_B.DataTypeConversion6[2] *
    buildover_fullstatemachine_B.aSinInput;
  buildover_fullstatemachine_B.y_idx_3 =
    buildover_fullstatemachine_B.DataTypeConversion6[3] *
    buildover_fullstatemachine_B.aSinInput;
  buildover_fullstatemachine_B.aSinInput = (u0_tmp *
    buildover_fullstatemachine_B.y_idx_3 -
    buildover_fullstatemachine_B.w_ff_gain * y_idx_2) * -2.0;
  mask1 = (buildover_fullstatemachine_B.aSinInput >= 0.99999999999999778);
  b_idx_0 = buildover_fullstatemachine_B.aSinInput;
  if (mask1) {
    b_idx_0 = 1.0;
  }

  if (buildover_fullstatemachine_B.aSinInput <= -0.99999999999999778) {
    b_idx_0 = -1.0;
  }

  mask1 = (mask1 || (buildover_fullstatemachine_B.aSinInput <=
                     -0.99999999999999778));
  buildover_fullstatemachine_B.aSinInput =
    buildover_fullstatemachine_B.w_ff_gain *
    buildover_fullstatemachine_B.w_ff_gain;
  out_tmp = u0_tmp * u0_tmp;
  out_tmp_0 = y_idx_2 * y_idx_2;
  out_tmp_1 = buildover_fullstatemachine_B.y_idx_3 *
    buildover_fullstatemachine_B.y_idx_3;
  buildover_fullstatemachine_B.out[0] = rt_atan2d_snf((u0_tmp * y_idx_2 +
    buildover_fullstatemachine_B.w_ff_gain *
    buildover_fullstatemachine_B.y_idx_3) * 2.0,
    ((buildover_fullstatemachine_B.aSinInput + out_tmp) - out_tmp_0) - out_tmp_1);
  buildover_fullstatemachine_B.out[1] = asin(b_idx_0);
  buildover_fullstatemachine_B.out[2] = rt_atan2d_snf((y_idx_2 *
    buildover_fullstatemachine_B.y_idx_3 +
    buildover_fullstatemachine_B.w_ff_gain * u0_tmp) * 2.0,
    ((buildover_fullstatemachine_B.aSinInput - out_tmp) - out_tmp_0) + out_tmp_1);
  trueCount = 0;
  if (mask1) {
    for (k = 0; k < 1; k++) {
      trueCount++;
    }
  }

  f_size_idx_0 = trueCount;
  trueCount = 0;
  if (mask1) {
    for (k = 0; k < 1; k++) {
      trueCount++;
    }
  }

  g_size_idx_0 = trueCount;
  trueCount = 0;
  if (mask1) {
    for (k = 0; k < 1; k++) {
      trueCount++;
    }
  }

  h_size_idx_0 = trueCount;
  if (f_size_idx_0 - 1 >= 0) {
    buildover_fullstatemachine_B.b_x_data = b_idx_0;
  }

  trueCount = f_size_idx_0 - 1;
  for (k = 0; k <= trueCount; k++) {
    if (rtIsNaN(buildover_fullstatemachine_B.b_x_data)) {
      buildover_fullstatemachine_B.b_x_data = (rtNaN);
    } else if (buildover_fullstatemachine_B.b_x_data < 0.0) {
      buildover_fullstatemachine_B.b_x_data = -1.0;
    } else {
      buildover_fullstatemachine_B.b_x_data =
        (buildover_fullstatemachine_B.b_x_data > 0.0);
    }
  }

  if (g_size_idx_0 == h_size_idx_0) {
    trueCount = g_size_idx_0;
    if (g_size_idx_0 - 1 >= 0) {
      buildover_fullstatemachine_B.j_data = rt_atan2d_snf(u0_tmp,
        buildover_fullstatemachine_B.w_ff_gain);
    }
  } else {
    if (g_size_idx_0 - 1 >= 0) {
      buildover_fullstatemachine_B.y_data = u0_tmp;
    }

    if (h_size_idx_0 - 1 >= 0) {
      buildover_fullstatemachine_B.y_data_b =
        buildover_fullstatemachine_B.w_ff_gain;
    }

    buildover_fullstat_expand_atan2(&buildover_fullstatemachine_B.y_data,
      &g_size_idx_0, &buildover_fullstatemachine_B.y_data_b, &h_size_idx_0,
      &buildover_fullstatemachine_B.j_data, &trueCount);
  }

  if (mask1) {
    for (k = 0; k < 1; k++) {
      tmp_data = 0;
    }
  }

  if (f_size_idx_0 == trueCount) {
    if (f_size_idx_0 - 1 >= 0) {
      buildover_fullstatemachine_B.out[0] =
        -buildover_fullstatemachine_B.b_x_data * 2.0 *
        buildover_fullstatemachine_B.j_data;
    }
  } else {
    buildover_full_binary_expand_op(buildover_fullstatemachine_B.out, &tmp_data,
      &buildover_fullstatemachine_B.b_x_data, &f_size_idx_0,
      &buildover_fullstatemachine_B.j_data, &trueCount);
  }

  trueCount = 0;
  if (mask1) {
    for (k = 0; k < 1; k++) {
      trueCount++;
    }
  }

  if (trueCount - 1 >= 0) {
    buildover_fullstatemachine_B.out[2] = 0.0;
  }

  mask1 = uORB_read_step(buildover_fullstatemachine_DW.obj_n.orbMetadataObj,
    &buildover_fullstatemachine_DW.obj_n.eventStructObj,
    &buildover_fullstatemachine_B.b_varargout_2_k, false, 1.0);
  if (mask1) {
    buildover_fullstatemachine_B.In1_mg =
      buildover_fullstatemachine_B.b_varargout_2_k;
  }

  buildover_fullstatemachine_B.BusAssignment.timestamp =
    buildover_fullstatemachine_B.In1_mg.timestamp;
  buildover_fullstatemachine_B.BusAssignment.output[0] = static_cast<real32_T>
    (buildover_fullstatemachine_B.RateLimiter1);
  buildover_fullstatemachine_B.BusAssignment.output[1] = static_cast<real32_T>
    (buildover_fullstatemachine_B.RateLimiter1_f);
  buildover_fullstatemachine_B.BusAssignment.output[2] = static_cast<real32_T>
    (buildover_fullstatemachine_B.Switch);
  buildover_fullstatemachine_B.BusAssignment.output[3] = static_cast<real32_T>
    (buildover_fullstatemachine_B.Saturation);
  buildover_fullstatemachine_B.BusAssignment.output[4] = static_cast<real32_T>
    (buildover_fullstatemachine_B.Diff);
  buildover_fullstatemachine_B.BusAssignment.output[5] = static_cast<real32_T>
    (buildover_fullstatemachine_B.Diff_g);
  buildover_fullstatemachine_B.BusAssignment.output[6] = static_cast<real32_T>
    (buildover_fullstatemachine_B.Diff_l);
  buildover_fullstatemachine_B.BusAssignment.output[7] =
    buildover_fullstatemachine_B.In1.x;
  buildover_fullstatemachine_B.BusAssignment.output[8] =
    buildover_fullstatemachine_B.In1.y;
  buildover_fullstatemachine_B.BusAssignment.output[9] = static_cast<real32_T>
    (buildover_fullstatemachine_B.In1.z - buildover_fullstatemachine_B.In);
  buildover_fullstatemachine_B.BusAssignment.output[10] =
    buildover_fullstatemachine_B.In1.vx;
  buildover_fullstatemachine_B.BusAssignment.output[11] =
    buildover_fullstatemachine_B.In1.vy;
  buildover_fullstatemachine_B.BusAssignment.output[12] =
    buildover_fullstatemachine_B.In1.vz;
  buildover_fullstatemachine_B.BusAssignment.output[13] = static_cast<real32_T>
    (buildover_fullstatemachine_B.out[2]);
  buildover_fullstatemachine_B.BusAssignment.output[14] = static_cast<real32_T>
    (buildover_fullstatemachine_B.out[1]);
  buildover_fullstatemachine_B.BusAssignment.output[15] = static_cast<real32_T>
    (buildover_fullstatemachine_B.out[0]);
  uORB_write_step(buildover_fullstatemachine_DW.obj_g.orbMetadataObj,
                  &buildover_fullstatemachine_DW.obj_g.orbAdvertiseObj,
                  &buildover_fullstatemachine_B.BusAssignment);
  for (k = 0; k < 8; k++) {
    buildover_fullstatemachine_B.pwmValue[k] = 0U;
  }

  buildover_fullstatemachine_B.Diff_l = floor
    (buildover_fullstatemachine_B.Switch);
  if (rtIsNaN(buildover_fullstatemachine_B.Diff_l) || rtIsInf
      (buildover_fullstatemachine_B.Diff_l)) {
    buildover_fullstatemachine_B.Diff_l = 0.0;
  } else {
    buildover_fullstatemachine_B.Diff_l = fmod
      (buildover_fullstatemachine_B.Diff_l, 65536.0);
  }

  buildover_fullstatemachine_B.pwmValue[0] = static_cast<uint16_T>
    (buildover_fullstatemachine_B.Diff_l < 0.0 ? static_cast<int32_T>(
      static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-buildover_fullstatemachine_B.Diff_l)))) : static_cast<int32_T>(
      static_cast<uint16_T>(buildover_fullstatemachine_B.Diff_l)));
  buildover_fullstatemachine_B.Diff_l = floor
    (buildover_fullstatemachine_B.RateLimiter1);
  if (rtIsNaN(buildover_fullstatemachine_B.Diff_l) || rtIsInf
      (buildover_fullstatemachine_B.Diff_l)) {
    buildover_fullstatemachine_B.Diff_l = 0.0;
  } else {
    buildover_fullstatemachine_B.Diff_l = fmod
      (buildover_fullstatemachine_B.Diff_l, 65536.0);
  }

  buildover_fullstatemachine_B.pwmValue[1] = static_cast<uint16_T>
    (buildover_fullstatemachine_B.Diff_l < 0.0 ? static_cast<int32_T>(
      static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-buildover_fullstatemachine_B.Diff_l)))) : static_cast<int32_T>(
      static_cast<uint16_T>(buildover_fullstatemachine_B.Diff_l)));
  buildover_fullstatemachine_B.Diff_l = floor
    (buildover_fullstatemachine_B.RateLimiter1_f);
  if (rtIsNaN(buildover_fullstatemachine_B.Diff_l) || rtIsInf
      (buildover_fullstatemachine_B.Diff_l)) {
    buildover_fullstatemachine_B.Diff_l = 0.0;
  } else {
    buildover_fullstatemachine_B.Diff_l = fmod
      (buildover_fullstatemachine_B.Diff_l, 65536.0);
  }

  buildover_fullstatemachine_B.pwmValue[2] = static_cast<uint16_T>
    (buildover_fullstatemachine_B.Diff_l < 0.0 ? static_cast<int32_T>(
      static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-buildover_fullstatemachine_B.Diff_l)))) : static_cast<int32_T>(
      static_cast<uint16_T>(buildover_fullstatemachine_B.Diff_l)));
  buildover_fullstatemachine_B.Diff_l = floor(buildover_fullstatemachine_B.Diff);
  if (rtIsNaN(buildover_fullstatemachine_B.Diff_l) || rtIsInf
      (buildover_fullstatemachine_B.Diff_l)) {
    buildover_fullstatemachine_B.Diff_l = 0.0;
  } else {
    buildover_fullstatemachine_B.Diff_l = fmod
      (buildover_fullstatemachine_B.Diff_l, 65536.0);
  }

  buildover_fullstatemachine_B.pwmValue[3] = static_cast<uint16_T>
    (buildover_fullstatemachine_B.Diff_l < 0.0 ? static_cast<int32_T>(
      static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-buildover_fullstatemachine_B.Diff_l)))) : static_cast<int32_T>(
      static_cast<uint16_T>(buildover_fullstatemachine_B.Diff_l)));
  buildover_fullstatemachine_B.Diff_l = floor
    (buildover_fullstatemachine_B.Saturation);
  if (rtIsNaN(buildover_fullstatemachine_B.Diff_l) || rtIsInf
      (buildover_fullstatemachine_B.Diff_l)) {
    buildover_fullstatemachine_B.Diff_l = 0.0;
  } else {
    buildover_fullstatemachine_B.Diff_l = fmod
      (buildover_fullstatemachine_B.Diff_l, 65536.0);
  }

  buildover_fullstatemachine_B.pwmValue[4] = static_cast<uint16_T>
    (buildover_fullstatemachine_B.Diff_l < 0.0 ? static_cast<int32_T>(
      static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-buildover_fullstatemachine_B.Diff_l)))) : static_cast<int32_T>(
      static_cast<uint16_T>(buildover_fullstatemachine_B.Diff_l)));
  buildover_fullstatemachine_B.Diff_l = floor
    (buildover_fullstatemachine_B.Diff_g);
  if (rtIsNaN(buildover_fullstatemachine_B.Diff_l) || rtIsInf
      (buildover_fullstatemachine_B.Diff_l)) {
    buildover_fullstatemachine_B.Diff_l = 0.0;
  } else {
    buildover_fullstatemachine_B.Diff_l = fmod
      (buildover_fullstatemachine_B.Diff_l, 65536.0);
  }

  buildover_fullstatemachine_B.pwmValue[5] = static_cast<uint16_T>
    (buildover_fullstatemachine_B.Diff_l < 0.0 ? static_cast<int32_T>(
      static_cast<uint16_T>(-static_cast<int16_T>(static_cast<uint16_T>
        (-buildover_fullstatemachine_B.Diff_l)))) : static_cast<int32_T>(
      static_cast<uint16_T>(buildover_fullstatemachine_B.Diff_l)));
  if (NOT1) {
    if (!buildover_fullstatemachine_DW.obj.isArmed) {
      buildover_fullstatemachine_DW.obj.isArmed = true;
      uDLookupTable6 = pwm_arm(&buildover_fullstatemachine_DW.obj.pwmDevHandler,
        &buildover_fullstatemachine_DW.obj.armAdvertiseObj);
      buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
        (buildover_fullstatemachine_DW.obj.errorStatus | uDLookupTable6);
    }

    uDLookupTable6 = pwm_setServo
      (&buildover_fullstatemachine_DW.obj.pwmDevHandler,
       buildover_fullstatemachine_DW.obj.servoCount,
       buildover_fullstatemachine_DW.obj.channelMask,
       &buildover_fullstatemachine_B.pwmValue[0],
       buildover_fullstatemachine_DW.obj.isMain,
       &buildover_fullstatemachine_DW.obj.actuatorAdvertiseObj);
    buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
      (buildover_fullstatemachine_DW.obj.errorStatus | uDLookupTable6);
  } else {
    uDLookupTable6 = pwm_disarm(&buildover_fullstatemachine_DW.obj.pwmDevHandler,
      &buildover_fullstatemachine_DW.obj.armAdvertiseObj);
    buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
      (buildover_fullstatemachine_DW.obj.errorStatus | uDLookupTable6);
    buildover_fullstatemachine_DW.obj.isArmed = false;
    uDLookupTable6 = pwm_resetServo
      (&buildover_fullstatemachine_DW.obj.pwmDevHandler,
       buildover_fullstatemachine_DW.obj.servoCount,
       buildover_fullstatemachine_DW.obj.channelMask,
       buildover_fullstatemachine_DW.obj.isMain,
       &buildover_fullstatemachine_DW.obj.actuatorAdvertiseObj);
    buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
      (buildover_fullstatemachine_DW.obj.errorStatus | uDLookupTable6);
  }

  if (buildover_fullstatemachine_DW.obj.isMain) {
    uDLookupTable6 = pwm_forceFailsafe
      (&buildover_fullstatemachine_DW.obj.pwmDevHandler, static_cast<int32_T>
       (buildover_fullstatemachine_P.Constant4_Value_n));
    buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
      (buildover_fullstatemachine_DW.obj.errorStatus | uDLookupTable6);
  }

  rt_ZCFcn(RISING_ZERO_CROSSING,
           &buildover_fullstatemach_PrevZCX.SampleandHold_Trig_ZCE_b,
           (buildover_fullstatemachine_DW.Memory_PreviousInput_h));
  uORB_read_step(buildover_fullstatemachine_DW.obj_p.orbMetadataObj,
                 &buildover_fullstatemachine_DW.obj_p.eventStructObj,
                 &buildover_fullstatemachine_B.b_varargout_2_cx, false, 1.0);
  if (buildover_fullstatemachine_B.mainrotor_control >
      buildover_fullstatemachine_P.Switch1_Threshold) {
    buildover_fullstatemachine_DW.Memory_PreviousInput =
      buildover_fullstatemachine_P.Constant_Value_g;
  } else {
    buildover_fullstatemachine_DW.Memory_PreviousInput =
      buildover_fullstatemachine_B.mainrotor_control;
  }

  buildover_fullstatemachine_DW.UD_DSTATE = buildover_fullstatemachine_B.TSamp;
  buildover_fullstatemachine_DW.Memory_PreviousInput_d =
    buildover_fullstatemachine_B.RateLimiter1;
  buildover_fullstatemachine_DW.UD_DSTATE_d =
    buildover_fullstatemachine_B.TSamp_o;
  buildover_fullstatemachine_DW.Memory1_PreviousInput =
    buildover_fullstatemachine_B.RateLimiter1_f;
  buildover_fullstatemachine_DW.UD_DSTATE_f =
    buildover_fullstatemachine_B.TSamp_g;
  buildover_fullstatemachine_DW.Memory_PreviousInput_g =
    buildover_fullstatemachine_B.CastToDouble;
  buildover_fullstatemachine_DW.Memory_PreviousInput_h =
    buildover_fullstatemachine_B.CastToDouble;
}

void buildover_fullstatemachine_initialize(void)
{
  rt_InitInfAndNaN(sizeof(real_T));

  {
    px4_Bus_actuator_outputs_commanded rtb_BusAssignment;
    buildover_fullstatemach_PrevZCX.SampleandHold_Trig_ZCE_b =
      UNINITIALIZED_ZCSIG;
    buildover_fullstatemach_PrevZCX.SampleandHold_Trig_ZCE = UNINITIALIZED_ZCSIG;
    buildover_fullstatemachine_DW.Memory_PreviousInput =
      buildover_fullstatemachine_P.Memory_InitialCondition;
    buildover_fullstatemachine_DW.PrevY =
      buildover_fullstatemachine_P.RateLimiter_IC;
    buildover_fullstatemachine_DW.UD_DSTATE =
      buildover_fullstatemachine_P.DiscreteDerivative_ICPrevScaled;
    buildover_fullstatemachine_DW.Memory_PreviousInput_d =
      buildover_fullstatemachine_P.Memory_InitialCondition_h;
    buildover_fullstatemachine_DW.UD_DSTATE_d =
      buildover_fullstatemachine_P.DiscreteDerivative_ICPrevScal_b;
    buildover_fullstatemachine_DW.Memory1_PreviousInput =
      buildover_fullstatemachine_P.Memory1_InitialCondition;
    buildover_fullstatemachine_DW.UD_DSTATE_f =
      buildover_fullstatemachine_P.DiscreteDerivative_ICPrevSca_bq;
    buildover_fullstatemachine_DW.PrevY_l =
      buildover_fullstatemachine_P.RateLimiter1_IC;
    buildover_fullstatemachine_DW.PrevY_f =
      buildover_fullstatemachine_P.RateLimiter1_IC_g;
    buildover_fullstatemachine_DW.Memory_PreviousInput_g =
      buildover_fullstatemachine_P.Memory_InitialCondition_hl;
    buildover_fullstatemachine_DW.Memory_PreviousInput_h =
      buildover_fullstatemachine_P.Memory_InitialCondition_p;
    buildover_fullstatemachine_B.In1_m = buildover_fullstatemachine_P.Out1_Y0_a;
    buildover_fullstatemachine_B.In1_n = buildover_fullstatemachine_P.Out1_Y0_d;
    buildover_fullstatemachine_DW.DiscreteTimeIntegrator_DSTATE =
      buildover_fullstatemachine_P.DiscreteTimeIntegrator_IC;
    buildover_fullstatemachine_B.In1 = buildover_fullstatemachine_P.Out1_Y0;
    buildover_fullstatemachine_B.In = buildover_fullstatemachine_P._Y0_g;
    buildover_fullstatemachine_B.In1_b = buildover_fullstatemachine_P.Out1_Y0_l;
    buildover_fullstatemachine_B.In1_mg = buildover_fullstatemachine_P.Out1_Y0_g;
    buildover_fullstatemachine_DW.obj_l.matlabCodegenIsDeleted = false;
    buildover_fullstatemachine_DW.obj_l.isInitialized = 1;
    buildover_fullstatemachine_DW.obj_l.orbMetadataObj = ORB_ID(input_rc);
    uORB_read_initialize(buildover_fullstatemachine_DW.obj_l.orbMetadataObj,
                         &buildover_fullstatemachine_DW.obj_l.eventStructObj);
    buildover_fullstatemachine_DW.obj_l.isSetupComplete = true;
    buildover_fullstatemachine_DW.obj_b.matlabCodegenIsDeleted = false;
    buildover_fullstatemachine_DW.obj_b.isInitialized = 1;
    buildover_fullstatemachine_DW.obj_b.orbMetadataObj = ORB_ID(input_rc);
    uORB_read_initialize(buildover_fullstatemachine_DW.obj_b.orbMetadataObj,
                         &buildover_fullstatemachine_DW.obj_b.eventStructObj);
    buildover_fullstatemachine_DW.obj_b.isSetupComplete = true;
    buildover_fullstatemachine_DW.obj_m.matlabCodegenIsDeleted = false;
    buildover_fullstatemachine_DW.obj_m.isInitialized = 1;
    buildover_fullstatemachine_DW.obj_m.orbMetadataObj = ORB_ID
      (vehicle_local_position);
    uORB_read_initialize(buildover_fullstatemachine_DW.obj_m.orbMetadataObj,
                         &buildover_fullstatemachine_DW.obj_m.eventStructObj);
    buildover_fullstatemachine_DW.obj_m.isSetupComplete = true;
    buildover_fullstatemachine_DW.obj_pt.matlabCodegenIsDeleted = false;
    buildover_fullstatemachine_DW.obj_pt.isInitialized = 1;
    buildover_fullstatemachine_DW.obj_pt.orbMetadataObj = ORB_ID
      (vehicle_attitude);
    uORB_read_initialize(buildover_fullstatemachine_DW.obj_pt.orbMetadataObj,
                         &buildover_fullstatemachine_DW.obj_pt.eventStructObj);
    buildover_fullstatemachine_DW.obj_pt.isSetupComplete = true;
    buildover_fullstatemachine_DW.obj_n.matlabCodegenIsDeleted = false;
    buildover_fullstatemachine_DW.obj_n.isInitialized = 1;
    buildover_fullstatemachine_DW.obj_n.orbMetadataObj = ORB_ID
      (vehicle_angular_velocity);
    uORB_read_initialize(buildover_fullstatemachine_DW.obj_n.orbMetadataObj,
                         &buildover_fullstatemachine_DW.obj_n.eventStructObj);
    buildover_fullstatemachine_DW.obj_n.isSetupComplete = true;
    buildover_fullstatemachine_DW.obj_g.matlabCodegenIsDeleted = false;
    buildover_fullstatemachine_DW.obj_g.isInitialized = 1;
    buildover_fullstatemachine_DW.obj_g.orbMetadataObj = ORB_ID
      (actuator_outputs_commanded);
    uORB_write_initialize(buildover_fullstatemachine_DW.obj_g.orbMetadataObj,
                          &buildover_fullstatemachine_DW.obj_g.orbAdvertiseObj,
                          &rtb_BusAssignment, 1);
    buildover_fullstatemachine_DW.obj_g.isSetupComplete = true;
    buildover_fullstatemachine_DW.obj.errorStatus = 0U;
    buildover_fullstatemachine_DW.obj.isInitialized = 0;
    buildover_fullstatemachine_DW.obj.matlabCodegenIsDeleted = false;
    buildover_full_SystemCore_setup(&buildover_fullstatemachine_DW.obj, false,
      buildover_fullstatemachine_P.Constant4_Value_n);
    buildover_fullstatemachine_DW.obj_p.matlabCodegenIsDeleted = false;
    buildover_fullstatemachine_DW.obj_p.isInitialized = 1;
    buildover_fullstatemachine_DW.obj_p.orbMetadataObj = ORB_ID(airspeed);
    uORB_read_initialize(buildover_fullstatemachine_DW.obj_p.orbMetadataObj,
                         &buildover_fullstatemachine_DW.obj_p.eventStructObj);
    buildover_fullstatemachine_DW.obj_p.isSetupComplete = true;
  }
}

void buildover_fullstatemachine_terminate(void)
{
  if (!buildover_fullstatemachine_DW.obj_l.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj_l.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj_l.isSetupComplete) {
      uORB_read_terminate(&buildover_fullstatemachine_DW.obj_l.eventStructObj);
    }
  }

  if (!buildover_fullstatemachine_DW.obj_b.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj_b.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj_b.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj_b.isSetupComplete) {
      uORB_read_terminate(&buildover_fullstatemachine_DW.obj_b.eventStructObj);
    }
  }

  if (!buildover_fullstatemachine_DW.obj_m.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj_m.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj_m.isSetupComplete) {
      uORB_read_terminate(&buildover_fullstatemachine_DW.obj_m.eventStructObj);
    }
  }

  if (!buildover_fullstatemachine_DW.obj_pt.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj_pt.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj_pt.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj_pt.isSetupComplete) {
      uORB_read_terminate(&buildover_fullstatemachine_DW.obj_pt.eventStructObj);
    }
  }

  if (!buildover_fullstatemachine_DW.obj_n.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj_n.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj_n.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj_n.isSetupComplete) {
      uORB_read_terminate(&buildover_fullstatemachine_DW.obj_n.eventStructObj);
    }
  }

  if (!buildover_fullstatemachine_DW.obj_g.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj_g.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj_g.isSetupComplete) {
      uORB_write_terminate(&buildover_fullstatemachine_DW.obj_g.orbAdvertiseObj);
    }
  }

  if (!buildover_fullstatemachine_DW.obj.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj.isSetupComplete) {
      uint16_T status;
      status = pwm_disarm(&buildover_fullstatemachine_DW.obj.pwmDevHandler,
                          &buildover_fullstatemachine_DW.obj.armAdvertiseObj);
      buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
        (buildover_fullstatemachine_DW.obj.errorStatus | status);
      status = pwm_resetServo(&buildover_fullstatemachine_DW.obj.pwmDevHandler,
        buildover_fullstatemachine_DW.obj.servoCount,
        buildover_fullstatemachine_DW.obj.channelMask,
        buildover_fullstatemachine_DW.obj.isMain,
        &buildover_fullstatemachine_DW.obj.actuatorAdvertiseObj);
      buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
        (buildover_fullstatemachine_DW.obj.errorStatus | status);
      status = pwm_close(&buildover_fullstatemachine_DW.obj.pwmDevHandler,
                         &buildover_fullstatemachine_DW.obj.actuatorAdvertiseObj,
                         &buildover_fullstatemachine_DW.obj.armAdvertiseObj);
      buildover_fullstatemachine_DW.obj.errorStatus = static_cast<uint16_T>
        (buildover_fullstatemachine_DW.obj.errorStatus | status);
    }
  }

  if (!buildover_fullstatemachine_DW.obj_p.matlabCodegenIsDeleted) {
    buildover_fullstatemachine_DW.obj_p.matlabCodegenIsDeleted = true;
    if ((buildover_fullstatemachine_DW.obj_p.isInitialized == 1) &&
        buildover_fullstatemachine_DW.obj_p.isSetupComplete) {
      uORB_read_terminate(&buildover_fullstatemachine_DW.obj_p.eventStructObj);
    }
  }
}
