#include "global.h"
#include "functions.h"

void *display_task(void *arg)
{
     struct task_par *tp;
     tp = (struct task_par *)arg;
     
     double dt_p = 0 , dt_v = 0, dt_t = 0;
     int p, b;   
     double t = 0;
    
     set_period(tp);
    
     while(true)
     {
        pthread_mutex_lock(&mux_page);
        p = page;
        b = but;
        pthread_mutex_unlock(&mux_page);
        
        if(b == 0)
        {
           dt_p = 0;
           dt_v = 0;
           dt_t = 0;
        }
        
        if(p == WORK_PAGE) 
        {
                             
           pthread_mutex_lock(&mux_p[n_p]);
           geo_motor(ctrmpx, ctrmpy, out_ctr_p[n_p]);
           pthread_mutex_unlock(&mux_p[n_p]);
               
           pthread_mutex_lock(&mux_v[n_v]);
           geo_motor(ctrmvx, ctrmvy, (long double)(out_ctr_v[n_v]*t)); //((*tp).at.tv_sec)));
           pthread_mutex_unlock(&mux_v[n_v]);
           
           pthread_mutex_lock(&mux_t[n_t]);
           geo_motor(ctrmtx, ctrmty, out_ctr_t[n_t]/motor_t[n_t].b); //(*tp).at.tv_sec));
           pthread_mutex_unlock(&mux_t[n_t]);
        }
        
        if(b == 1)
        {
           trends_pos(&dt_p, tp);              /* display trend for position control system */
            
           trends_vel(&dt_v, tp);              /* dispaly trend for velocity control system */    
                                
           trends_tor(&dt_t, tp);              /* dipslay trend for torque control system */
                                
        }
         
        t += GRAPHIC_PERIOD*0.001;
        
        if(deadline_miss(tp))
        {
          printf("Missed deadline of graphical thread!!\n");
        }
        wait_for_period(tp);
         
     }
}
