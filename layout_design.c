#include "global.h"
#include "functions.h"


void layout()
{
     
     
     
     layout_base("Work_page");
     
     
     
     /* motor shaped */
     
    
     
     /* set buttons on the right */
     
     button_set(rectx1, recty1, "Clear");
     button_set(rectx1, recty1+d+h, "Trends");
     button_set(rectx1, recty1+2*(d+h), "Gains");
     button_set(rectx1, recty1+3*(d+h), "Mot_Par");
     button_set(rectx1, recty1+4*(d+h), "pos_system_del");
     button_set(rectx1, recty1+5*(d+h), "vel_system_del");
     button_set(rectx1, recty1+6*(d+h), "tor_system_del");
     
     button_set(rectx1, sezy+(d+h), "DEMO");
     button_set(rectx1, sezy+2*(d+h), "OFF");
    
     /********************************************************************************************************************************/
    
     vertical_layout(ctrmpx, ctrmpy, &np, &n_p, "SYSTEM WITH CONTROL IN POSITION");        /* set buttons for position control motor */
     vertical_layout(ctrmvx, ctrmvy, &nv, &n_v, "SYSTEM WITH CONTROL IN VELOCITY");        /* set buttons for velocity control motor */
     vertical_layout(ctrmtx, ctrmty, &nt, &n_t, "SYSTEM WITH CONTROL IN TORQUE");          /* set buttons for torque control motor */

}

void mouse_task_layout()
{
            if(click_button(rectx1, sezy+2*(d+h)))     /* click OFF */
            {
               int i;
               for(i = 0; i < np; i++)
                   pthread_cancel(thread_p[i]);
     
               for(i = 0; i < nv; i++)
                   pthread_cancel(thread_v[i]);

               for(i = 0; i < nt; i++)
                   pthread_cancel(thread_t[i]);
               
               pthread_cancel(thread_g);
               
               pthread_exit(NULL);
             }
 
 /******************************************************** click in position section **************************************************************/        
            
            if(click_button(ctrmpx-r, sezy+2*(d+h)))             /* click the Add button */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               np = np + 1;                                    /* add a new motor */
               n_p = np - 1;                                   /* go on last motor insert in array */
           
           
               if(pthread_create(&thread_p[n_p], &my_attr_p[n_p], control_pos , &t_param_p[n_p]))
                   allegro_message("System control position thread not create!\n");
              
              rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
              vertical_layout(ctrmpx, ctrmpy, &np, &n_p, "SYSTEM WITH CONTROL IN POSITION");        /* set buttons for position control motor */
            }
            
            if(click_button(ctrmpx+d/2, sezy+2*(d+h)))           /* click the Next button*/
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               n_p = (n_p + 1) % np;                           /* lo scorrimento è ciclico */
           
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
               vertical_layout(ctrmpx, ctrmpy, &np, &n_p, "SYSTEM WITH CONTROL IN POSITION");        /* set buttons for position control motor */
               //layout();
            }
      
            if(click_button(ctrmpx-r, sezy+d+h))                 /* click the PID set parameters */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               page = PID_PARAM_PAGE;
               column = POS_COLUMN;
               pthread_mutex_unlock(&mux_page);
               
               control_pid_par();
            }
      
            if(click_button(ctrmpx+d/2, sezy+d+h))               /* click the motor set parameters */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               page = MOTOR_PARAM_PAGE;
               column = POS_COLUMN;
               pthread_mutex_unlock(&mux_page);
               
               set_motor_par();
            }
      
/******************************************************** click in velocity section **************************************************************/
      
            if(click_button(ctrmvx-r, sezy+2*(d+h)))             /* click add */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               nv++;                                           /* add a new motor */
               n_v = nv-1;                                     /* go on last motor insert in array */
           
               if(pthread_create(&thread_v[n_v], &my_attr_v[n_v], control_vel , &t_param_v[n_v]))
                  allegro_message("System control velocity thread not create!\n");
                
           
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
               vertical_layout(ctrmvx, ctrmvy, &nv, &n_v, "SYSTEM WITH CONTROL IN VELOCITY");        /* set buttons for velocity control motor */
            }
      
            if(click_button(ctrmvx+d/2, sezy+2*(d+h)))           /* clcik the Next button*/
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               page = WORK_PAGE;
               pthread_mutex_unlock(&mux_page);
               
               n_v = (n_v + 1) % nv;                           /* lo scorrimento è ciclico */
           
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
               vertical_layout(ctrmvx, ctrmvy, &nv, &n_v, "SYSTEM WITH CONTROL IN VELOCITY");        /* set buttons for velocity control motor */
            }
      
            if(click_button(ctrmvx-r, sezy+d+h))                 /* click the PID set parameters */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               page = PID_PARAM_PAGE;
               column = VEL_COLUMN;
               pthread_mutex_unlock(&mux_page);
               
               
               control_pid_par();
            }
      
            if(click_button(ctrmvx+d/2, sezy+d+h))               /* click the motor set parameters */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               page = MOTOR_PARAM_PAGE;
               column = VEL_COLUMN;
               pthread_mutex_unlock(&mux_page);
               
               
               set_motor_par();
            }

/***************************************************** click in torque section *****************************************************************/
      
            if(click_button(ctrmtx-r, sezy+2*(d+h)))             /* click add */
            { 
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               nt++;                                           /* add a new motor */
               n_t = nt-1;                                     /* go on last motor insert in array */
           
               if(pthread_create(&thread_t[n_t], &my_attr_t[n_t], control_tor , &t_param_t[n_t]))
                  allegro_message("System control torque thread not create!\n");
                 
           
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
               vertical_layout(ctrmtx, ctrmty, &nt, &n_t, "SYSTEM WITH CONTROL IN TORQUE");          /* set buttons for torque control motor */
            }

            if(click_button(ctrmtx+d/2, sezy+2*(d+h)))           /* click the Next button*/
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               n_t = (n_t + 1) % nt;                           /* lo scorrimento è ciclico */
           
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
               vertical_layout(ctrmtx, ctrmty, &nt, &n_t, "SYSTEM WITH CONTROL IN TORQUE");          /* set buttons for torque control motor */
            }

            if(click_button(ctrmtx-r, sezy+d+h))                 /* click the PID set parameters */
            {
               
               pthread_mutex_lock(&mux_page);
               but = 0;
               page = PID_PARAM_PAGE;
               column = TOR_COLUMN;
               pthread_mutex_unlock(&mux_page);
              
               control_pid_par();
            }

            if(click_button(ctrmtx+d/2, sezy+d+h))               /* click the motor set parameters */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               page = MOTOR_PARAM_PAGE;
               column = TOR_COLUMN;
               pthread_mutex_unlock(&mux_page);
               
               
               set_motor_par();
            }    
      
      
/*******************************************************************************************************************************************/     
                 
            if(click_button(rectx1, recty1))                                /* press clear button */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
            }
     
            if(click_button(rectx1, recty1+d+h))                                        /* click on trend's button*/
            {
               pthread_mutex_lock(&mux_page);
               but = 1;
               pthread_mutex_unlock(&mux_page);
               
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);             /* clear the space for display new things */
               clock_gettime(CLOCK_MONOTONIC, &start_p);
               clock_gettime(CLOCK_MONOTONIC, &start_v);
               clock_gettime(CLOCK_MONOTONIC, &start_t);
               
            }
      
            if(click_button(rectx1, recty1+2*(d+h)))                                       /* click on gain's button  */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);                /* clear the space for display new things */
                                                                                    /** if there aren't motor don't display the gains **/
               if(np > 0)
                  display_gains(ctrmpx, ctrmpy, &PID_p[n_p]);                             /* display gains for position control motor */
               if(nv > 0)
                  display_gains(ctrmvx, ctrmvy, &PID_v[n_v]);                             /* display gains for velocity control motor */
               if(nt > 0)
                  display_gains(ctrmtx, ctrmty, &PID_t[n_t]);                             /* display gains for torque control motor */
            }  
      
            if(click_button(rectx1, recty1+3*(d+h)))                                         /* click on mot_par's button  */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               rectfill(screen, 0, sezy-1, sezx-1, sezy-d-dy-10, COL_BG);                  /* clear the space for display new things */
                                                                                /** if there aren't motor don't display the parameters **/
               if(np > 0)
                  display_mot_par(ctrmpx, ctrmpy, &motor_p[n_p]);                             /* display parameters of motor in position control */
               if(nv > 0)
                  display_mot_par(ctrmvx, ctrmvy, &motor_v[n_v]);                             /* display parameters of motor in velocity control */
               if(nt > 0)
                  display_mot_par(ctrmtx, ctrmty, &motor_t[n_t]);                             /* display parameters of motor in torque control */
            }  
      
      /*******************************************************************************************************************************************/     
      
            if(click_button(rectx1, recty1+4*(d+h)))                 /* click the delete System position */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               if(n_p = np-1)
               {  
                  pthread_cancel(thread_p[n_p]);
           
                  read_data(&PID_p[n_p], &motor_p[n_p], "system_delete.txt");          /* all parameters of the system setted NULL */
                  out_ctr_p[n_p] = 0;
           
                  np--;
                  n_p = np-1;
               }
           
               layout();
            }
      
       /*******************************************************************************************************************************************/     
      
            if(click_button(rectx1, recty1+5*(d+h)))                 /* click the delete System velocity */
            {
               pthread_mutex_lock(&mux_page);
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               if(n_v = nv-1)
               {  
                  pthread_cancel(thread_v[n_v]);
           
                  read_data(&PID_v[n_v], &motor_v[n_v], "system_delete.txt");          /* all parameters of the system setted NULL */
                  out_ctr_v[n_v] = 0;
           
                  nv--;
                  n_v = nv-1;
               }

               layout();
            }
      
      /*******************************************************************************************************************************************/     
      
             if(click_button(rectx1, recty1+6*(d+h)))                 /* click the delete System torque */
             {
                pthread_mutex_lock(&mux_page);
                but = 0;
                pthread_mutex_unlock(&mux_page);
                
                if(n_t = nt-1)
                {  
                   pthread_cancel(thread_t[n_t]);
           
                   read_data(&PID_t[n_t], &motor_t[n_t], "system_delete.txt");          /* all parameters of the system setted NULL */
                   out_ctr_t[n_t] = 0;
           
                   nt--;
                   n_t = nt-1;
                }

                layout();
             }
      
      /*******************************************************************************************************************************************/        
             if(click_button(rectx1, sezy+(d+h)))                                                    /* start demo */
             {
                 pthread_mutex_lock(&mux_page);
                 but = 1;
                 pthread_mutex_unlock(&mux_page);
                  
                 if(np == 0 && nv == 0 && nt ==0)
                 {   
                    if(pthread_create(&thread_g, &my_attr_g, display_task , &t_param_g))
                       allegro_message("Graphical thread not create!\n");
                 }
      /*******************************************************************************************************************************************/       
                 np++;
                 n_p = np-1;
                 read_data(&PID_p[n_p], &motor_p[n_p], "data_motor.txt");                                          /* insert the motor's parameter for position control */
                 read_data(&PID_p[n_p], &motor_p[n_p], "data_PID_pos.txt");                                          /* insert the motor's parameter for position control */
            
                 if(pthread_create(&thread_p[n_p], &my_attr_p[n_p], control_pos , &t_param_p[n_p]))
                    allegro_message("System control position thread not create!\n");
                 
            
     /*******************************************************************************************************************************************/             
                 nv++;
                 n_v = nv-1;
                 read_data(&PID_v[n_v], &motor_v[n_v], "data_motor.txt");                                          /* insert the motor's parameter for velocity control */
                 read_data(&PID_v[n_v], &motor_v[n_v], "data_PID_vel.txt");                                          /* insert the motor's parameter for velocity control */
            
                 if(pthread_create(&thread_v[n_v], &my_attr_v[n_v], control_vel , &t_param_v[n_v]))
                    allegro_message("System control velocity thread not create!\n");
            
            
     /*******************************************************************************************************************************************/ 
                 nt++;
                 n_t = nt-1;
                 read_data(&PID_t[n_t], &motor_t[n_t], "data_motor.txt");                                          /* insert the motor's parameter for torque control */
                 read_data(&PID_t[n_t], &motor_t[n_t], "data_PID_tor.txt");                                          /* insert the motor's parameter for velocity control */
            
                 if(pthread_create(&thread_t[n_t], &my_attr_t[n_t], control_tor , &t_param_t[n_t]))
                    allegro_message("System control torque thread not create!\n");
            
                 layout();
             }
         
}
