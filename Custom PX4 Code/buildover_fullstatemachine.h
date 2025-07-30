#ifndef RTW_HEADER_buildover_fullstatemachine_h_
#define RTW_HEADER_buildover_fullstatemachine_h_
#include <poll.h>
#include <uORB/uORB.h>
#include "rtwtypes.h"
#include "MW_PX4_PWM.h"
#include "MW_uORB_Write.h"
#include "MW_uORB_Read.h"
#include "buildover_fullstatemachine_types.h"
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/actuator_outputs_commanded.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/airspeed.h>
#include "rt_zcfcn.h"

extern "C"
{

#include "rt_nonfinite.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <stddef.h>
#include "zero_crossing_types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

struct B_buildover_fullstatemachine_T {
  px4_Bus_vehicle_local_position In1;
  px4_Bus_vehicle_local_position b_varargout_2;
  px4_Bus_actuator_outputs_commanded BusAssignment;
  px4_Bus_input_rc In1_n;
  px4_Bus_input_rc In1_m;
  px4_Bus_input_rc b_varargout_2_m;
  px4_Bus_vehicle_attitude In1_b;
  px4_Bus_vehicle_attitude b_varargout_2_c;
  px4_Bus_vehicle_angular_velocity In1_mg;
  px4_Bus_vehicle_angular_velocity b_varargout_2_k;
  real_T DataTypeConversion6[4];
  px4_Bus_airspeed b_varargout_2_cx;
  real_T out[3];
  uint16_T pwmValue[8];
  real_T CastToDouble16;
  real_T In;
  real_T tau_ff_gain;
  real_T lcb_speed;
  real_T mainrotor_control;
  real_T rcb_speed;
  real_T rightcb_angle;
  real_T leftcb_angle;
  real_T aSinInput;
  real_T j_data;
  real_T b_x_data;
  real_T Saturation;
  real_T TSamp;
  real_T Switch;
  real_T TSamp_o;
  real_T TSamp_g;
  real_T RateLimiter1;
  real_T RateLimiter1_f;
  real_T Diff_g;
  real_T Diff_l;
  real_T y_data;
  real_T y_data_b;
  real_T Diff;
  real_T w_ff_gain;
  real_T CastToDouble;
  real_T y_idx_3;
};

struct DW_buildover_fullstatemachine_T {
  px4_internal_block_PWM_buildo_T obj;
  px4_internal_block_Subscriber_T obj_b;
  px4_internal_block_Subscriber_T obj_p;
  px4_internal_block_Subscriber_T obj_n;
  px4_internal_block_Subscriber_T obj_m;
  px4_internal_block_Subscriber_T obj_pt;
  px4_internal_block_Subscriber_T obj_l;
  px4_internal_block_Publisher__T obj_g;
  real_T UD_DSTATE;
  real_T UD_DSTATE_d;
  real_T UD_DSTATE_f;
  real_T DiscreteTimeIntegrator_DSTATE;
  real_T Memory_PreviousInput;
  real_T PrevY;
  real_T Memory_PreviousInput_d;
  real_T Memory1_PreviousInput;
  real_T PrevY_l;
  real_T PrevY_f;
  real_T Memory_PreviousInput_g;
  real_T Memory_PreviousInput_h;
  uint8_T is_active_c3_buildover_fullstat;
  uint8_T is_c3_buildover_fullstatemachin;
};

struct PrevZCX_buildover_fullstatema_T {
  ZCSigState SampleandHold_Trig_ZCE;
  ZCSigState SampleandHold_Trig_ZCE_b;
};

struct P_buildover_fullstatemachine_T_ {
  real_T DiscreteDerivative_ICPrevScaled;
  real_T DiscreteDerivative_ICPrevScal_b;
  real_T DiscreteDerivative_ICPrevSca_bq;
  uint16_T WrapToZero1_Threshold;
  uint16_T WrapToZero_Threshold;
  uint16_T WrapToZero1_Threshold_j;
  px4_Bus_vehicle_local_position Out1_Y0;
  px4_Bus_vehicle_local_position Constant_Value;
  px4_Bus_actuator_outputs_commanded Constant_Value_m;
  px4_Bus_input_rc Out1_Y0_a;
  px4_Bus_input_rc Out1_Y0_d;
  px4_Bus_input_rc Constant_Value_p;
  px4_Bus_input_rc Constant_Value_pf;
  px4_Bus_vehicle_attitude Out1_Y0_l;
  px4_Bus_vehicle_attitude Constant_Value_mf;
  px4_Bus_vehicle_angular_velocity Out1_Y0_g;
  px4_Bus_vehicle_angular_velocity Constant_Value_mi;
  px4_Bus_airspeed Out1_Y0_c;
  px4_Bus_airspeed Constant_Value_f;
  real_T Constant_Value_e;
  real_T Constant4_Value;
  real_T Constant3_Value;
  real_T Constant5_Value;
  real_T Constant1_Value;
  real_T Constant9_Value;
  real_T Constant6_Value;
  real_T Constant7_Value;
  real_T Constant8_Value;
  real_T uDLookupTable1_tableData[2];
  real_T uDLookupTable1_bp01Data[2];
  real_T Constant3_Value_a;
  real_T Constant9_Value_e;
  real_T Constant10_Value;
  real_T Constant4_Value_m;
  real_T Constant12_Value;
  real_T Constant2_Value;
  real_T Constant1_Value_d;
  real_T Constant_Value_fc;
  real_T Constant13_Value;
  real_T Constant3_Value_j;
  real_T Constant4_Value_g;
  real_T Constant5_Value_p;
  real_T Constant1_Value_dz;
  real_T Constant14_Value;
  real_T Constant7_Value_e;
  real_T Constant2_Value_b;
  real_T Constant6_Value_p;
  real_T Constant8_Value_c;
  real_T Constant6_Value_pc;
  real_T Constant4_Value_mr;
  real_T Constant5_Value_j;
  real_T Constant3_Value_g;
  real_T Constant14_Value_c;
  real_T Constant_Value_fp;
  real_T Constant1_Value_l;
  real_T Constant2_Value_j;
  real_T Constant7_Value_l;
  real_T Constant6_Value_c;
  real_T Constant5_Value_l;
  real_T Constant4_Value_o;
  real_T Constant3_Value_e;
  real_T Constant14_Value_i;
  real_T Constant_Value_ml;
  real_T Constant1_Value_n;
  real_T Constant2_Value_o;
  real_T Constant7_Value_p;
  real_T Constant4_Value_l;
  real_T Constant2_Value_i;
  real_T DiscreteTimeIntegrator_gainval;
  real_T DiscreteTimeIntegrator_IC;
  real_T DiscreteTimeIntegrator_UpperSat;
  real_T DiscreteTimeIntegrator_LowerSat;
  real_T Constant7_Value_o;
  real_T Saturation6_UpperSat;
  real_T Saturation6_LowerSat;
  real_T Constant5_Value_f;
  real_T Constant3_Value_p;
  real_T Constant14_Value_ch;
  real_T Constant_Value_a;
  real_T Constant12_Value_p;
  real_T Constant1_Value_k;
  real_T Constant6_Value_f;
  real_T Constant6_Value_cy;
  real_T Constant_Value_as;
  real_T Constant5_Value_n;
  real_T Constant1_Value_g;
  real_T Constant3_Value_j5;
  real_T Constant2_Value_oh;
  real_T Constant4_Value_d;
  real_T Constant11_Value;
  real_T Constant7_Value_j;
  real_T Constant6_Value_pg;
  real_T Constant3_Value_af;
  real_T Constant5_Value_a;
  real_T Constant4_Value_gr;
  real_T Constant12_Value_e;
  real_T Constant_Value_h;
  real_T Constant1_Value_p;
  real_T Constant2_Value_bd;
  real_T Constant7_Value_e1;
  real_T Constant2_Value_h;
  real_T Constant3_Value_jc;
  real_T Constant4_Value_e;
  real_T Constant14_Value_f;
  real_T Constant6_Value_f5;
  real_T Constant12_Value_j;
  real_T Constant1_Value_ly;
  real_T Constant11_Value_a;
  real_T Constant12_Value_d;
  real_T Constant1_Value_o;
  real_T Constant11_Value_c;
  real_T Constant2_Value_o4;
  real_T Constant3_Value_l;
  real_T Constant4_Value_i;
  real_T Constant6_Value_b;
  real_T Constant_Value_g;
  real_T uDLookupTable5_tableData[2];
  real_T uDLookupTable5_bp01Data[2];
  real_T Saturation5_UpperSat;
  real_T Saturation5_LowerSat;
  real_T Constant_Value_c;
  real_T Constant_Value_d;
  real_T _Y0;
  real_T _Y0_g;
  real_T Memory_InitialCondition;
  real_T Saturation7_UpperSat;
  real_T Saturation7_LowerSat;
  real_T RateLimiter_RisingLim;
  real_T RateLimiter_FallingLim;
  real_T RateLimiter_IC;
  real_T TSamp_WtEt;
  real_T Memory_InitialCondition_h;
  real_T TSamp_WtEt_m;
  real_T Memory1_InitialCondition;
  real_T TSamp_WtEt_e;
  real_T uDLookupTable1_tableData_b[2];
  real_T uDLookupTable1_bp01Data_g[2];
  real_T Saturation_UpperSat;
  real_T Saturation_LowerSat;
  real_T DeadZone_Start;
  real_T DeadZone_End;
  real_T Switch_Threshold;
  real_T RateLimiter1_RisingLim;
  real_T RateLimiter1_FallingLim;
  real_T RateLimiter1_IC;
  real_T RateLimiter1_RisingLim_p;
  real_T RateLimiter1_FallingLim_k;
  real_T RateLimiter1_IC_g;
  real_T Constant2_Value_p;
  real_T Gain6_Gain;
  real_T Gain_Gain;
  real_T Gain11_Gain;
  real_T uDLookupTable4_tableData[2];
  real_T uDLookupTable4_bp01Data[2];
  real_T Saturation1_UpperSat;
  real_T Saturation1_LowerSat;
  real_T Constant2_Value_l;
  real_T Gain7_Gain;
  real_T Gain11_Gain_j;
  real_T uDLookupTable4_tableData_b[2];
  real_T uDLookupTable4_bp01Data_k[2];
  real_T Saturation_UpperSat_a;
  real_T Saturation_LowerSat_j;
  real_T Memory_InitialCondition_hl;
  real_T Switch1_Threshold;
  real_T Memory_InitialCondition_p;
  uint16_T Constant_Value_p1;
  uint16_T Constant_Value_k;
  uint16_T Constant_Value_j;
  uint16_T uDLookupTable6_tableData[7];
  uint16_T uDLookupTable6_bp01Data[7];
  boolean_T Constant13_Value_j;
  boolean_T Constant13_Value_k;
  boolean_T Constant1_Value_a;
  boolean_T Constant13_Value_p;
  boolean_T Constant13_Value_a;
  boolean_T Constant13_Value_o;
  boolean_T Constant13_Value_n;
  boolean_T Constant13_Value_kn;
  boolean_T Constant13_Value_l;
  boolean_T Constant13_Value_d;
  boolean_T Constant13_Value_f;
  boolean_T Constant4_Value_n;
};

struct tag_RTM_buildover_fullstatema_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C"
{

#endif

  extern P_buildover_fullstatemachine_T buildover_fullstatemachine_P;

#ifdef __cplusplus

}

#endif

#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_buildover_fullstatemachine_T buildover_fullstatemachine_B;

#ifdef __cplusplus

}

#endif

extern struct DW_buildover_fullstatemachine_T buildover_fullstatemachine_DW;
extern PrevZCX_buildover_fullstatema_T buildover_fullstatemach_PrevZCX;

#ifdef __cplusplus

extern "C"
{

#endif

  extern void buildover_fullstatemachine_initialize(void);
  extern void buildover_fullstatemachine_step(void);
  extern void buildover_fullstatemachine_terminate(void);

#ifdef __cplusplus

}

#endif

#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_buildover_fullstatem_T *const buildover_fullstatemachine_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

#endif

