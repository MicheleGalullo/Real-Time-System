#include "global.h"
#include "functions.h"

     
void trends_pos(double *dt_p, struct task_par *tp)
{     
               //if( ( (*dt_p) += 0.001*GRAPHIC_PERIOD ) >= (dx-5) )
              
               if(((*dt_p) += 2 ) >= (dx-5))   //((*tp).at.tv_sec-start_p.tv_sec)
               {
                  rectfill(screen, d, sezy-1, d+dx, sezy-d-dy, COL_BG);
                  //clock_gettime(CLOCK_MONOTONIC, &start_p);
                  (*dt_p) = 0;//((*tp).at.tv_sec-start_p.tv_sec);
               }
               
               axis(d, sezy-d, "theta");
               
               //pthread_mutex_lock(&mux_us[POS_COLUMN]);
               line(screen, d, sezy-d -fmod(PID_p[n_p].ref, dy), d +dx-5, sezy-d -fmod(PID_p[n_p].ref, dy), COL_REF);
               //pthread_mutex_unlock(&mux_us[POS_COLUMN]);
               
               pthread_mutex_lock(&mux_p[n_p]);
               line(screen, (*dt_p) +d, sezy-d, (*dt_p) +d, (-out_ctr_p[n_p] +sezy-d), COL_MOT_REF);
               pthread_mutex_unlock(&mux_p[n_p]);
               
}
               
     /* dispaly trend for velocity control system */
               
void trends_vel(double *dt_v, struct task_par *tp)
{
               
               if(((*dt_v) += 2 ) >= (dx-5))  //((*tp).at.tv_sec-start_v.tv_sec)
               {
                  rectfill(screen, d+sezxm, sezy-1, d+sezxm+dx, sezy-d -dy, COL_BG);
                  //clock_gettime(CLOCK_MONOTONIC, &start_v);
                  (*dt_v) = 0;//((*tp).at.tv_sec-start_v.tv_sec);
               }
               
               axis(d+sezxm, sezy-d, "omega");
               
               //pthread_mutex_lock(&mux_us[VEL_COLUMN]);
               line(screen, d+sezxm, sezy-d -fmod(PID_v[n_v].ref, dy), d+sezxm+dx-5, sezy-d -fmod(PID_v[n_v].ref, dy), COL_REF);
               //pthread_mutex_unlock(&mux_us[VEL_COLUMN]);
               
               pthread_mutex_lock(&mux_v[n_v]);
               line(screen, (*dt_v)+ d+sezxm, sezy-d, (*dt_v) +d+sezxm, (-out_ctr_v[n_v] +sezy-d), COL_MOT_REF);
               pthread_mutex_unlock(&mux_v[n_v]);
}
               
     /* dipslay trend for torque control system */
               
void trends_tor(double *dt_t, struct task_par *tp)
{
               
               if(((*dt_t) += 2 ) >= (dx-5)) //(*tp).at.tv_sec-start_t.tv_sec
               {   
                  rectfill(screen, d+2*sezxm, sezy-1, d+2*sezxm+dx, sezy-d-dy, COL_BG);
                  //clock_gettime(CLOCK_MONOTONIC, &start_t);
                  (*dt_t) = 0; // (*tp).at.tv_sec-start_t.tv_sec;
               }
               
               axis(d+2*sezxm, sezy-d, "torque");
               
               //pthread_mutex_lock(&mux_us[TOR_COLUMN]);
               line(screen, d+2*sezxm, sezy-d -fmod(PID_t[n_t].ref, dy), d+2*sezxm+dx-5, sezy-d -fmod(PID_t[n_t].ref, dy), COL_REF);
               //pthread_mutex_unlock(&mux_us[TOR_COLUMN]);
               
               pthread_mutex_lock(&mux_t[n_t]);
               line(screen, (*dt_t) +d+2*sezxm, sezy-d, (*dt_t) +d+2*sezxm, (-out_ctr_t[n_t] +sezy-d), COL_MOT_REF);
               pthread_mutex_unlock(&mux_t[n_t]);
}



