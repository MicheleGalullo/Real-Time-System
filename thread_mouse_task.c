#include "global.h"
#include "functions.h"

void *mouse_task(void *arg)
{
     struct timespec t;
     struct task_par *tp;
     tp = (struct task_par *)arg;
     
     
     while(true)
     {
         show_mouse(screen);
         
         if(page == WORK_PAGE)
         {
            mouse_task_layout();
         }
     
         if(page == PID_PARAM_PAGE)
         {
            if(column == POS_COLUMN) 
               mouse_task_set_PID_par(&PID_p[n_p], POS_COLUMN);
            
            if(column == VEL_COLUMN) 
               mouse_task_set_PID_par(&PID_v[n_v], VEL_COLUMN);
            
            if(column == TOR_COLUMN) 
               mouse_task_set_PID_par(&PID_t[n_t], TOR_COLUMN);                     
             
         }
         
         if(page == MOTOR_PARAM_PAGE)
         {
            if(column == POS_COLUMN)
               mouse_task_set_motor_par(&motor_p[n_p], POS_COLUMN);
            
            if(column == VEL_COLUMN)
               mouse_task_set_motor_par(&motor_v[n_v], VEL_COLUMN);
            
            if(column == TOR_COLUMN)
               mouse_task_set_motor_par(&motor_t[n_t], TOR_COLUMN);
              
         }
         
         
         clock_gettime(CLOCK_MONOTONIC, &t);
         time_add_ms(&t, (*tp).period);
         clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
         
     }    
}
