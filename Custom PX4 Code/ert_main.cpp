#include <stdio.h>
#include <stdlib.h>
#include "buildover_fullstatemachine.h"
#include "buildover_fullstatemachine_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "MW_PX4_TaskControl.h"
#include "nuttxinitialize.h"
#define UNUSED(x)                      x = x
#define NAMELEN                        16

void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
px4_sem_t stopSem;
px4_sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
void *threadJoinStatus;
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(buildover_fullstatemachine_M) == (NULL));
  while (runModel) {
    px4_sem_wait(&baserateTaskSem);
    buildover_fullstatemachine_step();
    stopRequested = !((rtmGetErrorStatus(buildover_fullstatemachine_M) == (NULL)));
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(buildover_fullstatemachine_M, "stopping the model");
  runModel = 0;
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  MW_PX4_Terminate();
  buildover_fullstatemachine_terminate();
  px4_sem_post(&stopSem);
  return NULL;
}

int px4_simulink_app_task_main (int argc, char *argv[])
{
  px4_simulink_app_control_MAVLink();
  rtmSetErrorStatus(buildover_fullstatemachine_M, 0);
  buildover_fullstatemachine_initialize();
  nuttxRTOSInit(0.001, 0);
  px4_sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(px4_sem_destroy(&timerTaskSem[i]), 0, "px4_sem_destroy");
    }
  }

#endif

  return 0;
}
