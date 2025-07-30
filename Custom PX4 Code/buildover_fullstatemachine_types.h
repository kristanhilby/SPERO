#ifndef RTW_HEADER_buildover_fullstatemachine_types_h_
#define RTW_HEADER_buildover_fullstatemachine_types_h_
#include "rtwtypes.h"
#include <uORB/topics/actuator_outputs_commanded.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/airspeed.h>
#ifndef struct_px4_internal_block_PWM_buildo_T
#define struct_px4_internal_block_PWM_buildo_T

struct px4_internal_block_PWM_buildo_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  int pwmDevHandler;
  const char * pwmDevObj;
  unsigned int servoCount;
  int channelMask;
  boolean_T isMain;
  uint16_T errorStatus;
  orb_advert_t armAdvertiseObj;
  orb_advert_t actuatorAdvertiseObj;
  boolean_T isArmed;
};

#endif

#ifndef struct_px4_internal_block_Publisher__T
#define struct_px4_internal_block_Publisher__T

struct px4_internal_block_Publisher__T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  orb_advert_t orbAdvertiseObj;
  orb_metadata_t * orbMetadataObj;
};

#endif

#ifndef struct_e_px4_internal_block_SampleTi_T
#define struct_e_px4_internal_block_SampleTi_T

struct e_px4_internal_block_SampleTi_T
{
  int32_T __dummy;
};

#endif

#ifndef struct_px4_internal_block_Subscriber_T
#define struct_px4_internal_block_Subscriber_T

struct px4_internal_block_Subscriber_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  e_px4_internal_block_SampleTi_T SampleTimeHandler;
  pollfd_t eventStructObj;
  orb_metadata_t * orbMetadataObj;
};

#endif

#ifndef struct_robotics_slcore_internal_bloc_T
#define struct_robotics_slcore_internal_bloc_T

struct robotics_slcore_internal_bloc_T
{
  int32_T isInitialized;
};

#endif

typedef struct P_buildover_fullstatemachine_T_ P_buildover_fullstatemachine_T;
typedef struct tag_RTM_buildover_fullstatema_T RT_MODEL_buildover_fullstatem_T;

#endif

