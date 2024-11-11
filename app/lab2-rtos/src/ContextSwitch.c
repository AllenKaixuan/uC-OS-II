// File: TwoTasks.c

#include <stdio.h>
#include "includes.h"
#include <string.h>

#include "system.h"
#include "unistd.h"
#include "altera_avalon_performance_counter.h"
#include "altera_avalon_pio_regs.h"

#define DEBUG 0
#define THRESHOLD_MAX 1.5
#define THRESHOLD_MIN 0.5
#define TARGET_TIMES  10

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define TASK_STACKSIZE 2048
OS_STK task1_stk[TASK_STACKSIZE];
OS_STK task2_stk[TASK_STACKSIZE];
OS_STK stat_stk[TASK_STACKSIZE];

/* Definition of Task Priorities */
#define TASK1_PRIORITY 6 // highest priority
#define TASK2_PRIORITY 7
#define TASK_STAT_PRIORITY 12 // lowest priority
static int TASK1_STATE = 0;
static int TASK2_STATE = 0;

OS_EVENT *Task1_Sem;
OS_EVENT *Task2_Sem;





void printStackSize(char *name, INT8U prio)
{
  INT8U err;
  OS_STK_DATA stk_data;

  err = OSTaskStkChk(prio, &stk_data);
  if (err == OS_NO_ERR)
  {
    if (DEBUG == 1)
      printf("%s (priority %d) - Used: %d; Free: %d\n",
             name, prio, stk_data.OSUsed, stk_data.OSFree);
  }
  else
  {
    if (DEBUG == 1)
      printf("Stack Check Error!\n");
  }
}

/* Prints a message and sleeps for given time interval */
void task1(void *pdata)
{
  int times = 0;
  INT8U err;
  static alt_u64 time2 = 0;
  static alt_u64 time2_avg = 0;
  
  while (times < TARGET_TIMES)
  {
    if(times == TARGET_TIMES)
    //printf("Task 0 - State %d\n", TASK1_STATE);
    PERF_RESET(PERFORMANCE_COUNTER_BASE);
    PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
    PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 1);

    OSSemPend(Task1_Sem, 0, &err);
    PERF_END(PERFORMANCE_COUNTER_BASE, 2);
    PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
    time2 = perf_get_section_time(PERFORMANCE_COUNTER_BASE,2);
    
    // fist time must update, and in the range update
    if((time2_avg == 0)||((time2 < THRESHOLD_MAX * time2_avg) && (time2 > THRESHOLD_MIN * time2_avg))) 
      time2_avg = (time2_avg * times + time2) / (times + 1);
    times++;  
    printf("NO.%d 2 to 1: %d\n", times,time2);  
    
  
    //TASK1_STATE = (TASK1_STATE + 1) % 2;
    //printf("Task 0 - State %d\n", TASK1_STATE);

    //TASK1_STATE = (TASK1_STATE + 1) % 2;

    if(times == TARGET_TIMES)
    printf("Average 2 to 1: %d\n", time2_avg);
    
    OSSemPost(Task2_Sem);
    
    
  }
  
  OSTaskDel(TASK2_PRIORITY);
}

/* Prints a message and sleeps for given time interval */
void task2(void *pdata)
{

  
  // total_clocks = perf_get_total_time (PERFORMANCE_COUNTER_BASE);
  // total_sec    = (double)total_clocks ;
  // printf("total_sec:, %e",total_sec);
  int times = 0;
  static alt_u64 time1 = 0;
  static alt_u64 time1_avg = 0;
  INT8U err;
  
  while (times < TARGET_TIMES)
  {

    PERF_END(PERFORMANCE_COUNTER_BASE, 1);
    PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
    time1 = perf_get_section_time(PERFORMANCE_COUNTER_BASE,1);
    if((time1_avg == 0)||((time1 < THRESHOLD_MAX * time1_avg) && (time1 > THRESHOLD_MIN * time1_avg)))
      time1_avg = (time1_avg * times + time1) / (times + 1);
    times++;  
    printf("NO.%d 1 to 2: %d\n", times,time1);
    

    OSSemPend(Task2_Sem, 0, &err);
    //printf("Task 1 - State %d\n", TASK2_STATE);
    
    
    // fist time must update, and in the range update
     

    //TASK2_STATE = (TASK2_STATE + 1) % 2;
    //printf("Task 1 - State %d\n", TASK2_STATE);

    //TASK2_STATE = (TASK2_STATE + 1) % 2;

    if(times == TARGET_TIMES)
     printf("Average 1 to 2: %d\n", time1_avg);

    PERF_RESET(PERFORMANCE_COUNTER_BASE);
    PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);
    PERF_BEGIN(PERFORMANCE_COUNTER_BASE, 2);
    OSSemPost(Task1_Sem);
    //OSTimeDlyHMSM(0, 0, 0, 4);
    
  }
  
  OSTaskDel(TASK2_PRIORITY);
}

/* Printing Statistics */
void statisticTask(void *pdata)
{
  while (1)
  {
    printStackSize("Task1", TASK1_PRIORITY);
    printStackSize("Task2", TASK2_PRIORITY);
    printStackSize("StatisticTask", TASK_STAT_PRIORITY);
  }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
  printf("Lab 3 - Two Tasks\n");
  Task1_Sem = OSSemCreate(0);
  Task2_Sem = OSSemCreate(1);
  OSTaskCreateExt(task1,                          // Pointer to task code
                  NULL,                           // Pointer to argument passed to task
                  &task1_stk[TASK_STACKSIZE - 1], // Pointer to top of task stack
                  TASK1_PRIORITY,                 // Desired Task priority
                  TASK1_PRIORITY,                 // Task ID
                  &task1_stk[0],                  // Pointer to bottom of task stack
                  TASK_STACKSIZE,                 // Stacksize
                  NULL,                           // Pointer to user supplied memory (not needed)
                  OS_TASK_OPT_STK_CHK |           // Stack Checking enabled
                      OS_TASK_OPT_STK_CLR         // Stack Cleared
  );

  OSTaskCreateExt(task2,                          // Pointer to task code
                  NULL,                           // Pointer to argument passed to task
                  &task2_stk[TASK_STACKSIZE - 1], // Pointer to top of task stack
                  TASK2_PRIORITY,                 // Desired Task priority
                  TASK2_PRIORITY,                 // Task ID
                  &task2_stk[0],                  // Pointer to bottom of task stack
                  TASK_STACKSIZE,                 // Stacksize
                  NULL,                           // Pointer to user supplied memory (not needed)
                  OS_TASK_OPT_STK_CHK |           // Stack Checking enabled
                      OS_TASK_OPT_STK_CLR         // Stack Cleared
  );

  if (DEBUG == 1)
  {
    OSTaskCreateExt(statisticTask,                 // Pointer to task code
                    NULL,                          // Pointer to argument passed to task
                    &stat_stk[TASK_STACKSIZE - 1], // Pointer to top of task stack
                    TASK_STAT_PRIORITY,            // Desired Task priority
                    TASK_STAT_PRIORITY,            // Task ID
                    &stat_stk[0],                  // Pointer to bottom of task stack
                    TASK_STACKSIZE,                // Stacksize
                    NULL,                          // Pointer to user supplied memory (not needed)
                    OS_TASK_OPT_STK_CHK |          // Stack Checking enabled
                        OS_TASK_OPT_STK_CLR        // Stack Cleared
    );
  }

  OSStart();
  return 0;
}
