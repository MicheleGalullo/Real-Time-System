#include "global.h"
#include "functions.h"

/****************************************************************************************************************************************************************************/
/*********************************************************************** THREADS FUNCTIONS **********************************************************************************/
/****************************************************************************************************************************************************************************/


void *control_pos(void *arg)
{
      struct task_par *tp;
      tp = (struct task_par *)arg;
     
      float buff_in[5];
      float out_pos[5];
      float aux_i[5];
      float aux_p[5];

      float Ki, Kp, Kd, J, L, b, R, Ke, Kt, ref;
      
      struct par_mgt par_mgt_p;
     
      int i = 0;
      
      set_period(tp);
      
      while(true)
      {    
           pthread_mutex_lock(&mux_us[POS_COLUMN]);
           set_par(&Ki, &Kp, &Kd, &J, &L, &b, &R, &Ke, &Kt, &ref, &PID_p[(*tp).arg], &motor_p[(*tp).arg]);
           pthread_mutex_unlock(&mux_us[POS_COLUMN]);
           
           tf_position(&par_mgt_p, Ki, Kp, Kd, J, L, b, R, Ke, Kt);
           
           control_position_compute(par_mgt_p, ref, buff_in, aux_i, aux_p, out_pos, i);
           
           if((*tp).arg == n_p)
           {   
               pthread_mutex_lock(&mux_p[(*tp).arg]);
               out_ctr_p[(*tp).arg] = out_pos[0];
               pthread_mutex_unlock(&mux_p[(*tp).arg]);
           }
           else out_ctr_p[(*tp).arg] = out_pos[0];
           
           i++;   
           
           if(deadline_miss(tp))
           {
              printf("Missed deadline of System control position thread!! System n. %d\n", (*tp).arg);
           }
           wait_for_period(tp);
      
      }

}




void *control_vel (void *arg)
{
      struct task_par *tp;
      tp = (struct task_par *)arg;

      float buff_in[5];
      float out_vel[5];
      float aux_i[5];
      float aux_v[5];
      
      float Ki, Kp, Kd, J, L, b, R, Ke, Kt, ref;
      
     
      struct par_mgt par_mgt_v;
      
      int i = 0;
           
      set_period(tp);

      while(true)
      {    
           pthread_mutex_lock(&mux_us[VEL_COLUMN]);
           set_par(&Ki, &Kp, &Kd, &J, &L, &b, &R, &Ke, &Kt, &ref, &PID_v[(*tp).arg], &motor_v[(*tp).arg]);
           pthread_mutex_unlock(&mux_us[VEL_COLUMN]);
           
           tf_velocity(&par_mgt_v, Ki, Kp, Kd, J, L, b, R, Ke, Kt);
           
           control_velocity_compute(par_mgt_v, ref, buff_in, aux_i, aux_v, out_vel, i);
           
           
           if((*tp).arg == n_v)
           {
               pthread_mutex_lock(&mux_v[(*tp).arg]);
               out_ctr_v[(*tp).arg] = out_vel[0];
               pthread_mutex_unlock(&mux_v[(*tp).arg]);
           }
           else out_ctr_v[(*tp).arg] = out_vel[0];
           
           i++;
           
           
           if(deadline_miss(tp))
           {
              printf("Missed deadline of System control velocity thread!! System n. %d\n", (*tp).arg);
           }
           
           wait_for_period(tp);
      
      }

}




void *control_tor(void *arg)
{
      struct task_par *tp;
      tp = (struct task_par *)arg;
     
      float buff_in[5];
      float out_tor[5];
      float aux_i[5];
      float aux_t[5];
      
      float Ki, Kp, Kd, J, L, b, R, Ke, Kt, ref;
      
      struct par_mgt par_mgt_t;
     
      int i = 0;
     
      set_period(tp);
      
      while(true)
      {
           pthread_mutex_lock(&mux_us[TOR_COLUMN]);   
           set_par(&Ki, &Kp, &Kd, &J, &L, &b, &R, &Ke, &Kt, &ref, &PID_t[(*tp).arg], &motor_t[(*tp).arg]);
           pthread_mutex_unlock(&mux_us[TOR_COLUMN]);
           
           tf_torque(&par_mgt_t, Ki, Kp, Kd, J, L, b, R, Ke, Kt);
           
           control_torque_compute(par_mgt_t, ref, buff_in, aux_i, aux_t, out_tor, i);
           
           
           if((*tp).arg == n_t)
           {
              pthread_mutex_lock(&mux_t[(*tp).arg]);
              out_ctr_t[(*tp).arg] = out_tor[0];
              pthread_mutex_unlock(&mux_t[(*tp).arg]);
           }
           else out_ctr_t[(*tp).arg] = out_tor[0];
           
           i++;
          
           
           if(deadline_miss(tp))
           {
              printf("Missed deadline of System control torque thread!! System n. %d\n", (*tp).arg);
           } 
           wait_for_period(tp);
      }

}


/****************************************************************************************************************************************************************************/
/****************************************************************************************************************************************************************************/
