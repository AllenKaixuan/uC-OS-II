/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 1

#define HW_TIMER_PERIOD 100 /* 100ms */
#define DEBOUNCE_DELAY 200
#define TARGET_SPEED 25
#define SPEED_TOLERANCE 4
#define MIN_SPEED (TARGET_SPEED - SPEED_TOLERANCE)
#define MAX_SPEED (TARGET_SPEED + SPEED_TOLERANCE)

/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear
#define LED_RED_17 0x00020000
#define LED_RED_16 0x00010000
#define LED_RED_15 0x00008000
#define LED_RED_14 0x00004000
#define LED_RED_13 0x00002000
#define LED_RED_12 0x00001000


#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0004 // Cruise Control Button
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];
OS_STK SwitchIOTask_Stack[TASK_STACKSIZE];
OS_STK WatchdogTask_Stack[TASK_STACKSIZE];
OS_STK OverloadDetectionTask_Stack[TASK_STACKSIZE];
OS_STK ExtraLoadTask_Stack[TASK_STACKSIZE];

// Task Priorities

#define STARTTASK_PRIO     5
#define SWITCH_PRIO 6   // higher priority becuase of engine
#define BUTTON_PRIO 7

#define WATCHDOG_TASK_PRIO 4


#define VEHICLETASK_PRIO  10
#define CONTROLTASK_PRIO  12
#define OVERLOAD_DETECTION_TASK_PRIO 15
#define EXTRA_LOAD_TASK_PRIO 14


// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define BUTTONIO_PERIOD 100
#define SWITCHIO_PERIOD 100
#define OVERLOADDETEC_PERIOD 300
#define EXTRALOAD_PERIOD 300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;
OS_EVENT *Mbox_Velocity;
OS_EVENT *Mbox_Brake;
OS_EVENT *Mbox_Engine;
OS_EVENT *Mbox_Gas;
OS_EVENT *Mbox_Cruise;
OS_EVENT *Mbox_Gear;

// Semaphores
OS_EVENT *VehicleSem;
OS_EVENT *ControlSem;
OS_EVENT *ButtonIOSem;
OS_EVENT *SwitchIOSem;
OS_EVENT *WatchdogSem;
OS_EVENT *FeedDog;
OS_EVENT *OverloadDetectionSem;
OS_EVENT *ExtraloadSem;


// SW-Timer
OS_TMR *VehicleTimer;
OS_TMR *ControlTimer;
OS_TMR *ButtonIOTimer;
OS_TMR *SwitchIOTimer;

OS_TMR *OverloadDetectionTimer;
OS_TMR *ExtraloadTimer;

/*
 * Types
 */
enum active {on = 2, off = 1};



/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs
volatile int edge_capture;
enum active ENGINE = off;
enum active TOP_GEAR = off;
enum active CRUISE_CONTROL = off;
enum active BRAKE_PEDAL = off;
enum active GAS_PEDAL = off;
int ENABLE_ENGINEOFF = 0;
INT16S acceleration; 
volatile int system_utilization = 0;


/*
 * Helper functions
 */

int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

/*
 * Callback for HW Timer
 */
void VehicleTimerCallback (void *ptmr, void *callback_arg){
  OSSemPost(VehicleSem);
  // printf("OSSemPost(VehicleSem);\n");
}

void ControlTimerCallback (void *ptmr, void *callback_arg){
  OSSemPost(ControlSem);
  // printf("OSSemPost(ControlSem);\n");
}

void ButtonIOTimerCallback (void *ptmr, void *callback_arg){
  OSSemPost(ButtonIOSem);
}

void SwitchIOTimerCallback (void *ptmr, void *callback_arg){
  OSSemPost(SwitchIOSem);
}

void OverloadDetectionTimerCallback (void *ptmr, void *callback_arg){
  OSSemPost(OverloadDetectionSem);
}

void ExtraloadTimerCallback (void *ptmr, void *callback_arg){
  OSSemPost(ExtraloadSem);
}


static int b2sLUT[] = {0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8U target_vel)
{
  int tmp = target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);
  out = int2seven(0) << 21 |
    int2seven(0) << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);
}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
  led_red &= ~(LED_RED_17 | LED_RED_16 | LED_RED_15 | LED_RED_14 | LED_RED_13 | LED_RED_12);
  if (0 <= position && position < 400)
    led_red |= LED_RED_17;
        
  else if(400 <= position && position < 800)
    led_red |= LED_RED_16;
  else if (800 <= position && position < 1200)
    led_red |= LED_RED_15;
  else if (1200 <= position && position < 1600)
    led_red |= LED_RED_14;
  else if (1600 <= position && position < 2000)
    led_red |= LED_RED_13;
  else if (2000 <= position)
    led_red |= LED_RED_12;
IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
}


/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */
void VehicleTask(void* pdata)
{ 
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;  
  void* msg;
  INT8U* throttle; 
   
  INT16U position = 0; 
  INT16S velocity = 0; 
  enum active brake_pedal = off;
  enum active engine = off;

  printf("Vehicle task created!\n");

  while(1)
  {
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

    // OSTimeDlyHMSM(0,0,0,VEHICLE_PERIOD); 
    OSSemPend(VehicleSem, 0, &err);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_NO_ERR) 
      throttle = (INT8U*) msg;
    /* Same for the brake signal that bypass the control law */
    msg = OSMboxPend(Mbox_Brake, 1, &err); 
    if (err == OS_NO_ERR) 
      brake_pedal = *(enum active*) msg;
    /* Same for the engine signal that bypass the control law */
    msg = OSMboxPend(Mbox_Engine, 1, &err); 
    if (err == OS_NO_ERR) 
      engine = *(enum active*) msg;
      

    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);

    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;

    show_velocity_on_sevenseg((INT8S) velocity);
    show_position(position);
  }
} 

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
 
  void* msg;
  INT16S* current_velocity;
  INT16S  pre_previous_vel = 0;

  enum active gas_pedal = off;
  enum active top_gear = off;
  enum active cruise_control = off; 

  printf("Control Task created!\n");

  while(1)
  {
    msg = OSMboxPend(Mbox_Velocity, 0, &err);
    current_velocity = (INT16S*) msg;
    msg = OSMboxPend(Mbox_Cruise, 0, &err);
    if (err == OS_NO_ERR) 
      cruise_control = *(enum active*) msg;

    msg = OSMboxPend(Mbox_Gas, 0, &err);
    if (err == OS_NO_ERR) 
      gas_pedal = *(enum active*) msg;
    show_target_velocity(0);
    if(cruise_control == on)
    {
      msg = OSMboxPend(Mbox_Gear, 0, &err);
      if (err == OS_NO_ERR) 
        top_gear = *(enum active*) msg;

      if( *current_velocity > 20)  //gas and brake must be closed in IO, need not care
      { 
        // cruise
        led_green |= LED_GREEN_0;
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
        show_target_velocity(TARGET_SPEED);
        int speed_error = TARGET_SPEED - *current_velocity;
        throttle += (int)(0.2 * speed_error);
        if (throttle < 0) {
            throttle = 0;
        } else if (throttle > 80) {
            throttle = 80;
        }
        if (current_velocity >= MIN_SPEED && current_velocity <= MAX_SPEED) {
          throttle = throttle; 
    }
        
        
      }
      else
      {
        cruise_control = off;
        led_green &= ~LED_GREEN_2;
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
        
      }
      
    }
    else    // mannul
    {
      
      if(*current_velocity == 0)
        ENABLE_ENGINEOFF = 1;
      else
        ENABLE_ENGINEOFF = 0;

      if(gas_pedal == on)
        throttle += 30;
      


    }

    // Here you can use whatever technique or algorithm that you prefer to control
    // the velocity via the throttle. There are no right and wrong answer to this controller, so
    // be free to use anything that is able to maintain the cruise working properly. You are also
    // allowed to store more than one sample of the velocity. For instance, you could define
    //
    // INT16S previous_vel;
    // INT16S pre_previous_vel;
    // ...
    //
    // If your control algorithm/technique needs them in order to function. 

    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);

    // OSTimeDlyHMSM(0,0,0, CONTROL_PERIOD);
    OSSemPend(ControlSem, 0, &err);
  }
}

void ButtonIOTask(void* pdata)
{
    printf("ButtonIOTask generated!\n");

    
    INT8U err;

    while (1)
    {     
        int btn_reg;   
        btn_reg = buttons_pressed();
        btn_reg = btn_reg & 0xf;
        
        if (btn_reg == CRUISE_CONTROL_FLAG)
        {
          
            OSTimeDlyHMSM(0, 0, 0, DEBOUNCE_DELAY);   
            // btn_reg = buttons_pressed();   
            // btn_reg = btn_reg & 0xf;
            if((TOP_GEAR == on) && (GAS_PEDAL == off) && (BRAKE_PEDAL == off))
            {
              if (CRUISE_CONTROL == off)
              {

                  led_green |= LED_GREEN_2;
                  CRUISE_CONTROL = on;
              }
              else
              {
                  led_green &= ~LED_GREEN_2;
                  led_green &= ~LED_GREEN_0;
                  CRUISE_CONTROL = off;
              }
            }

            
        }

        
        if (btn_reg == BRAKE_PEDAL_FLAG)
        {
          OSTimeDlyHMSM(0, 0, 0, DEBOUNCE_DELAY);  
          // btn_reg = buttons_pressed();
          // btn_reg = btn_reg & 0xf; 
          if (BRAKE_PEDAL == off)
          {
            led_green &= ~LED_GREEN_6;  // deactive brake
            GAS_PEDAL = off;

            led_green &= ~LED_GREEN_0;  // deactive cruise
            CRUISE_CONTROL = off;

            led_green |= LED_GREEN_4; 
            BRAKE_PEDAL = on;
            
              

          }
          else
          {
              led_green &= ~LED_GREEN_4;
              BRAKE_PEDAL = off;
          }
        }

        
        if (btn_reg == GAS_PEDAL_FLAG)
        {
          OSTimeDlyHMSM(0, 0, 0, DEBOUNCE_DELAY);
          // btn_reg = buttons_pressed();
          // btn_reg = btn_reg & 0xf;
          if(BRAKE_PEDAL == off){
            if (GAS_PEDAL == off)
            {
                led_green |= LED_GREEN_6;
                GAS_PEDAL = on;

                led_green &= ~LED_GREEN_0;  // deactive cruise
                CRUISE_CONTROL = off;
            }
            else
            {
                led_green &= ~LED_GREEN_6;
                GAS_PEDAL = off;
            }
          }
        }
        OSMboxPost(Mbox_Brake, (void *) &BRAKE_PEDAL);
        OSMboxPost(Mbox_Gas, (void *) &GAS_PEDAL);
        OSMboxPost(Mbox_Cruise, (void *) &CRUISE_CONTROL);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
      
        
        OSSemPend(ButtonIOSem, 0, &err);
    }
}

void SwitchIOTask(void* pdata)
{
  INT8U err;
  printf("SwitchIOTask generated!\n");
  // /* Write to the edge capture register to reset it. */
  // IOWR_ALTERA_AVALON_PIO_EDGE_CAP(DE2_PIO_TOGGLES18_BASE, 0);
  // /* reset interrupt capability for the Button PIO. */
  // IOWR_ALTERA_AVALON_PIO_IRQ_MASK(DE2_PIO_TOGGLES18_BASE, 0x3);
  while (1)
    {
        int switch_reg;
        
        int last_switch = -1;
        switch_reg = switches_pressed();
        switch_reg = switch_reg & 0x3;
       if(switch_reg != last_switch){
         last_switch = switch_reg;
        switch (switch_reg) // the mechanism is different with buttons
        {
          case (ENGINE_FLAG | TOP_GEAR_FLAG):
            led_red |= LED_RED_1;
            led_red |= LED_RED_0;
            ENGINE = on;
            TOP_GEAR = on;
            break;
          case (ENGINE_FLAG & TOP_GEAR_FLAG):
            if(ENABLE_ENGINEOFF)    // engine off when v = 0
            {
              led_red &= ~LED_RED_0;
              ENGINE = off;
            }
            led_red &= ~LED_RED_1;
            TOP_GEAR = off;
            CRUISE_CONTROL = off;
            led_green &= ~LED_GREEN_0;
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);

            break;
          case (TOP_GEAR_FLAG):
            led_red |= LED_RED_1;
            led_red &= ~LED_RED_0;
            ENGINE = off;
            TOP_GEAR = on;
            break;
          case (ENGINE_FLAG):
            led_red |= LED_RED_0;
            led_red &= ~LED_RED_1; 
            ENGINE = on;
            TOP_GEAR = off;
            CRUISE_CONTROL = off;
            led_green &= ~LED_GREEN_0;
            IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, led_green);
            break;

          default:
            break;
        } 
        OSMboxPost(Mbox_Engine, (void *) &ENGINE);
        IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, led_red);
       }
    OSSemPend(SwitchIOSem, 0, &err);
  }
}

// Watchdog Task
void WatchdogTask(void* pdata) {
    INT8U err;
    while (1) {
        OSSemPend(WatchdogSem, 400, &err); 
        if (err == OS_TIMEOUT) {
            printf("System Overload Detected!\n");
        } else {
            printf("System OK\n");
        }
    }
}

// Overload Detection Task
void OverloadDetectionTask(void* pdata) {
  INT8U err;
    while (1) {
      OSSemPend(OverloadDetectionSem, 0, &err); 
      OSSemPost(WatchdogSem); 
    }
}


void simulate_load(int load) {
    int i, j;
    for (i = 0; i < load * 1000; i++) {
        j = i * i; //  CPU LOAD
    }
}

// Extra Load Task
void ExtraLoadTask(void* pdata) {
  INT8U err;
    while (1) {
       
        int load = ( switches_pressed()& 0x3F0)>>4; // read SW4~SW9
        if (load > 50) load = 50; 
        system_utilization = load * 2; // +2%
        //int load_time = system_utilization*10;
        printf("ExtraLoadTask system_utilization %d %% \n", system_utilization);
        simulate_load(system_utilization);
       
        OSSemPend(ExtraloadSem, 0, &err);
    }
}

/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000;    // to calculate how many system clocks per HW clock
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,    // callback funtion, is called after delay, after delay, alarm_handler give a new delay
        context) < 0)
  {
    printf("No system clock available!n");
  }
  
  

  /* 
   * Create and start Software Timer 
   */
  VehicleTimer = OSTmrCreate(0, //delay
                        VEHICLE_PERIOD/HW_TIMER_PERIOD, //period
                        OS_TMR_OPT_PERIODIC,  //  automatically reload itself
                        VehicleTimerCallback, //OS_TMR_CALLBACK
                        (void *)0,
                        "VehicleTimer",
                        &err);
                            
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("VehicleTimer created\n");
    }
   }


  ControlTimer = OSTmrCreate(0, //delay
                      CONTROL_PERIOD/HW_TIMER_PERIOD, //period
                      OS_TMR_OPT_PERIODIC,
                      ControlTimerCallback, //OS_TMR_CALLBACK
                      (void *)0,
                      "ControlTimer",
                      &err);
                          
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("ControlTimer created\n");
    }
  }

  ButtonIOTimer = OSTmrCreate(0, //delay
                      BUTTONIO_PERIOD/HW_TIMER_PERIOD, //period
                      OS_TMR_OPT_PERIODIC,
                      ButtonIOTimerCallback, //OS_TMR_CALLBACK
                      (void *)0,
                      "ButtonIOTimer",
                      &err);
                          
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("ButtonIOTimer created\n");
    }
  }

  SwitchIOTimer = OSTmrCreate(0, //delay
                      SWITCHIO_PERIOD/HW_TIMER_PERIOD, //period
                      OS_TMR_OPT_PERIODIC,
                      SwitchIOTimerCallback, //OS_TMR_CALLBACK
                      (void *)0,
                      "SwitchIOTimer",
                      &err);
                          
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("SwitchIOTimer created\n");
    }
  }

  OverloadDetectionTimer = OSTmrCreate(0, //delay
                      OVERLOADDETEC_PERIOD/HW_TIMER_PERIOD, //period
                      OS_TMR_OPT_PERIODIC,
                      OverloadDetectionTimerCallback, //OS_TMR_CALLBACK
                      (void *)0,
                      "OverloadDetectionTimer",
                      &err);
                          
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("OverloadDetectionTimer created\n");
    }
  }

  ExtraloadTimer = OSTmrCreate(0, //delay
                     EXTRALOAD_PERIOD/HW_TIMER_PERIOD, //period
                      OS_TMR_OPT_PERIODIC,
                      ExtraloadTimerCallback, //OS_TMR_CALLBACK
                      (void *)0,
                      "ExtraloadTimer",
                      &err);
                          
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //if creation successful
      printf("ExtraloadTimer created\n");
    }
  }

  OSTmrStart(VehicleTimer, &err);
   
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if start successful
      printf("VehicleTimer started\n");
    }
   }

  OSTmrStart(ControlTimer, &err);
   
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if start successful
      printf("ControlTimer started\n");
    }
   }

  OSTmrStart(ButtonIOTimer, &err);
   
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if start successful
      printf("ButtonIOTimer started\n");
    }
   }

  OSTmrStart(SwitchIOTimer, &err);
   
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if start successful
      printf("SwitchIOTimer started\n");
    }
   }  

  OSTmrStart(OverloadDetectionTimer, &err);
   
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if start successful
      printf("OverloadDetectionTimer started\n");
    }
   }
   
  OSTmrStart(ExtraloadTimer, &err);
   
   if (DEBUG) {
    if (err == OS_ERR_NONE) { //if start successful
      printf("ExtraloadTimer started\n");
    }
   }
   
  /* 
   * Create and start Semaphore 
   */
  VehicleSem = OSSemCreate(0);
  ControlSem = OSSemCreate(0);
  ButtonIOSem = OSSemCreate(0);
  SwitchIOSem = OSSemCreate(0);
  WatchdogSem = OSSemCreate(0);
  OverloadDetectionSem = OSSemCreate(0);
  ExtraloadSem = OSSemCreate(0);

  
  /*
   * Creation of Kernel Objects
   */

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 0); 
  Mbox_Engine = OSMboxCreate((void*) 0); 
  Mbox_Gas = OSMboxCreate((void*) 0);
  Mbox_Cruise = OSMboxCreate((void*) 0);

  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  

  err = OSTaskCreateExt(
      ButtonIOTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ButtonIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      BUTTON_PRIO,
      BUTTON_PRIO,
      (void *)&ButtonIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      SwitchIOTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &SwitchIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      SWITCH_PRIO,
      SWITCH_PRIO,
      (void *)&SwitchIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
    WatchdogTask, 
    NULL, 
    &WatchdogTask_Stack[TASK_STACKSIZE-1], 
    WATCHDOG_TASK_PRIO, 
    WATCHDOG_TASK_PRIO, 
    (void *)&WatchdogTask_Stack[0], 
    TASK_STACKSIZE, 
    (void *) 0, 
    OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    err = OSTaskCreateExt(
      OverloadDetectionTask, 
      NULL, 
      &OverloadDetectionTask_Stack[TASK_STACKSIZE-1],
       OVERLOAD_DETECTION_TASK_PRIO, 
       OVERLOAD_DETECTION_TASK_PRIO, 
       (void *)&OverloadDetectionTask_Stack[0], 
       TASK_STACKSIZE, 
       (void *) 0, 
       OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

    err = OSTaskCreateExt(
      ExtraLoadTask, 
      NULL, 
      &ExtraLoadTask_Stack[TASK_STACKSIZE-1], 
      EXTRA_LOAD_TASK_PRIO, EXTRA_LOAD_TASK_PRIO, 
      (void *)&ExtraLoadTask_Stack[0], 
      TASK_STACKSIZE, 
      (void *) 0, 
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  printf("All Tasks and Kernel Objects generated!\n");

  

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");
 

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSStart();
  

  return 0;
}
