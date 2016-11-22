#include <allegro.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>


#ifndef constant
#define constant

/* constant for threads */

#define Nm 10

/* constant for controller */

#define Ts 0.05
#define PI 3.14159265

/* define colors */

#define COL_MOT 14             /* yellow motors */
#define COL_MOT_REF 12         /* red line */
#define COL_REF 10             /* green triangle */
#define COL_BUT 14             /* yellow buttons */
#define COL_BG 9               /* blue background */
#define BG -1                  /* background of writer */

/* page index */

#define PID_PARAM_PAGE 1
#define MOTOR_PARAM_PAGE 2
#define WORK_PAGE 0


/* column index*/

#define POS_COLUMN 0
#define VEL_COLUMN 1
#define TOR_COLUMN 2

/* threads parameters */

#define SYS_PERIOD 90
#define SYS_DEADLINE 88
#define SYS_PRIORITY 80

#define GRAPHIC_PERIOD 100
#define GRAPHIC_DEADLINE 98
#define GRAPHIC_PRIORITY 70

#define MOUSE_PERIOD 50
#define MOUSE_DEADLINE 28
#define MOUSE_PRIORITY 75

/* define screen dimension */

extern const int scrx;
extern const int scry;
extern const int sezx;     /* small screen dimension */
extern const int sezy;     /* small screen dimension */
extern const int sezxm;    /* length of motor screen */
extern const int sezym;    /* heigth of motor screen */


/* define buttons geometrics property */

extern const int rectx1;               /* x position for first watch button */
extern const int recty1;               /* y position for first watch button */

extern const int pidx1;               /* x position for first pid button */
extern const int pidy1;               /* y position for first pid button */

extern const int motparx1;            /* x position for first button of motor's parameter */
extern const int motpary1;            /* y position for first button of motor's parameter */

extern const int l;                    /* rectangle length */
extern const int h;                    /* rectangle heigth */
extern const int dh;                   /* text distance */
extern const int d;                    /* spacing */
extern const int dy;                  /* sezy-(ctrmpy+r+2*d+h) = 157 */
extern const int dx;
/* define motors geometrics property */

extern const int r;                    /* radius of motors */
extern const int ctrmpx;               /* x center of motor for position control */
extern const int ctrmpy;               /* y center of motor for position control */

extern const int ctrmvx;               /* x center of motor for velocity control */
extern const int ctrmvy;               /* y center of motor for velocity control */

extern const int ctrmtx;               /* x center of motor for torque control */
extern const int ctrmty;               /* y center of motor for torque control */

                                     /* the motor section have dimension of 300*233 */

char p[100];                                                /* stringa di appoggio */




/****************************************************************************************************************************************************************************/


struct controller
{
       float KP;            /* Proportional gain */
       float KI;            /* Integrative gain */
       float KD;            /* Derivative gain */
       float ref;           /* reference to follow */

};

struct select_motor
{
       float L;                      /* terminal inductance */
       float R;                      /* terminal resistence */
       float J;                      /* the motor and load inercia */
       float b;                      /* friction constant */
       float kb;                     /* the back-EMF constant of the motor */
       float kt;                     /* the torque constant */
       float V;                      /* voltage */
};


struct par_mgt
{
      float K1, K2, K3, D, E, F, G, H, I;  
      float a1, a2, a3, a4, a5, b1, b2, b3, b4, b5, b6, c1, c2, c3, c4, c5, c6, x, y;
      float d11, d12, d13, d14, d15, n11, n12, n13, n14, n15;
      float d21, d22, d23, d24, d25, d26, n21, n22, n23, n24, n25, n26; 
      float d31, d32, d33, d34, d35, d36, n31, n32, n33, n34, n35, n36; 
};

       


/****************************************************************************************************************************************************************************/


struct task_par {
       
       int arg;
       
       long wcet;            /* in microsecond */
       int period;           /* in millisecond */ 
       int deadline;         /* relative (ms) */
       int priority;         /* in [0, 99] */
       int dmiss;            /* no. dmiss */
       struct timespec at;   /* next activ. time */
       struct timespec dl;   /* abs deadline */
};



/****************************************************************************************************************************************************************************/
/******************************************************************* Define Global Variable *********************************************************************************/
/****************************************************************************************************************************************************************************/



int n_p, n_v, n_t;                                    /* the current motor in array */
int np, nv, nt;                                       /* the total number of motor */
int page, column, but;                                /* variable for page control */


struct select_motor motor_p[Nm], motor_v[Nm], motor_t[Nm];
struct controller PID_p[Nm], PID_v[Nm], PID_t[Nm];
float out_ctr_p[Nm], out_ctr_v[Nm], out_ctr_t[Nm];


//struct par_mgt par_mgt_p, par_mgt_v, par_mgt_t;


struct sched_param s_param_g, s_param_m;

struct sched_param s_param_p;
struct sched_param s_param_v;
struct sched_param s_param_t;

struct task_par t_param_g, t_param_m;

struct task_par t_param_p[Nm];
struct task_par t_param_v[Nm];
struct task_par t_param_t[Nm];
     
struct timespec start_p, start_v, start_t;
     
pthread_t thread_g, thread_m;

pthread_t thread_p[Nm];
pthread_t thread_v[Nm];
pthread_t thread_t[Nm];

pthread_attr_t my_attr_g, my_attr_m;

pthread_attr_t my_attr_p[Nm];
pthread_attr_t my_attr_v[Nm];
pthread_attr_t my_attr_t[Nm];

pthread_mutex_t mux_p[Nm], mux_v[Nm], mux_t[Nm], mux_us[3], mux_page;


/****************************************************************************************************************************************************************************/
/****************************************************************************************************************************************************************************/
/****************************************************************************************************************************************************************************/



#endif

