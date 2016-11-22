#include <allegro.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "global.h"


/* utility functions */

void get_keycodes(char *scan, char *ascii);
void get_string(char *str, int x, int y, int c, int b);
bool click_button(int x, int y);
void read_data(struct controller *PID, struct select_motor *motor, char q[100]);
void time_add_ms(struct timespec *t, int ms);
void time_copy(struct timespec *td, struct timespec ts);
int time_cmp(struct timespec t1, struct timespec t2);
void set_period(struct task_par *tp);
void wait_for_period(struct task_par *tp);
int deadline_miss(struct task_par *tp);
void set_par(float *Ki, float *Kp, float *Kd, float *J, float *L, float *b, float *R, float *Ke, float *Kt, float *ref, struct controller *PID, struct select_motor *motor);

/* graphical functions */

void button_set(int x, int y, char s[40]);
void geo_motor(int x, int y, long double theta);
void axis(int x0, int y0, char s[40]);
void layout_base(char s[100]);
void vertical_layout(int x, int y, int *n, int *n_n, char s[100]);

/* PID parameter */

void control_pid_par();
void display_gains(int x, int y, struct controller *PID);

/* motors parameters*/

void set_motor_par();
void display_mot_par(int x, int y, struct select_motor *motor);

/* layout design */

void layout();


/* main */

void initialize_graph_threads();
void initialize_system_threads();
void mouse_thread();

/* thread graphics task */

void *display_task(void *arg);

void trends_pos(double *dt_p, struct task_par *tp);
void trends_vel(double *dt_v, struct task_par *tp);
void trends_tor(double *dt_t, struct task_par *tp);

/* controller functions */

void tf_torque(struct par_mgt *par_mgt, float Ki, float Kp, float Kd, float J, float L, float b, float R, float Ke, float Kt);
void control_torque_compute(struct par_mgt par_mgt_t, float ref, float buff_in[], float aux_i[], float aux_t[], float out_tor[], int i);

void tf_velocity(struct par_mgt *par_mgt, float Ki, float Kp, float Kd, float J, float L, float b, float R, float Ke, float Kt);
void control_velocity_compute(struct par_mgt par_mgt, float ref, float buff_in[], float aux_i[], float aux_v[], float out_vel[], int i);

void tf_position(struct par_mgt *par_mgt, float Ki, float Kp, float Kd, float J, float L, float b, float R, float Ke, float Kt);
void control_position_compute(struct par_mgt par_mgt, float ref, float buff_in[], float aux_i[], float aux_p[], float out_pos[], int i);

/* thread control task */

void *control_vel(void *arg);
void *control_pos(void *arg);
void *control_tor(void *arg);

/* thread mouse task */

void *mouse_task(void *arg);

void mouse_task_set_motor_par(struct select_motor *motor, int sem);
void mouse_task_layout();
void mouse_task_set_PID_par(struct controller *PID, int sem);



















