#ifndef _MW_UORB_BUSSTRUCT_CONVERSION_H_
#define _MW_UORB_BUSSTRUCT_CONVERSION_H_

#include <uORB/topics/actuator_outputs_commanded.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

typedef struct actuator_outputs_commanded_s  px4_Bus_actuator_outputs_commanded ;
typedef struct airspeed_s  px4_Bus_airspeed ;
typedef struct input_rc_s  px4_Bus_input_rc ;
typedef struct vehicle_angular_velocity_s  px4_Bus_vehicle_angular_velocity ;
typedef struct vehicle_attitude_s  px4_Bus_vehicle_attitude ;
typedef struct vehicle_local_position_s  px4_Bus_vehicle_local_position ;

#endif
