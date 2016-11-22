#include "global.h"
#include "functions.h"




void set_motor_par()
{
     
     clear_to_color(screen, COL_BG);
     
     //show_mouse(screen);
     layout_base("Set motor parameters");
              
     /* command watch screen set buttons on the right */
     
     button_set(rectx1, recty1, "previus");
    
     /* shared screen for set the motor */
     
     textout_centre_ex(screen, font, "SET THE PARAMETERS OF MOTOR", sezx/2, d, COL_BUT, BG);
     
     button_set(motparx1, motpary1-d/2, "Terminal Inductance (L)");
     button_set(motparx1, motpary1+2*d, "Terminal Resistence (R)");
     button_set(motparx1, motpary1+4.5*d, "Back-EMF constant (kb)");
     button_set(motparx1, motpary1+7*d, "Torque Constant (kt)");
     button_set(motparx1, motpary1+9.5*d, "Motor and Load Inercia (J)");
     button_set(motparx1, motpary1+12*d, "Friction Constant (b)");
     
     
}  



void display_mot_par(int x, int y, struct select_motor *motor)
{
     textout_ex(screen,font, "MOTOR PARAMETERS", x-r-d/2, y+r+2*d, COL_MOT_REF, BG);
           
     sprintf(p, "Terminal Inductance (L): %.3f", (*motor).L);
     textout_ex(screen,font, p, x-r-d/2, y+r+d+2*(d+h)/2, COL_MOT_REF, BG);
     sprintf(p, "Terminal Resistence (R): %.3f", (*motor).R);
     textout_ex(screen,font, p, x-r-d/2, y+r+d+3*(d+h)/2, COL_MOT_REF, BG);
     sprintf(p, "Back-EMF constant (kb): %.3f", (*motor).kb);
     textout_ex(screen,font, p, x-r-d/2, y+r+d+4*(d+h)/2, COL_MOT_REF, BG);
     sprintf(p, "Torque Constant (kt): %.3f", (*motor).kt);
     textout_ex(screen,font, p, x-r-d/2, y+r+d+5*(d+h)/2, COL_MOT_REF, BG);
     sprintf(p, "Motor and Load Inercia (J): %.3f", (*motor).J);
     textout_ex(screen,font, p, x-r-d/2, y+r+d+6*(d+h)/2, COL_MOT_REF, BG);
     sprintf(p, "Friction constant (b): %.3f", (*motor).b);
     textout_ex(screen,font, p, x-r-d/2, y+r+d+7*(d+h)/2, COL_MOT_REF, BG);
}

void mouse_task_set_motor_par(struct select_motor *motor, int sem)
{
            if(click_button(motparx1,motpary1-d/2))                            /* click (L) */
            {
               textout_ex(screen, font, "Insert the value of terminal inductance (L): ", motparx1+d+2*l, motpary1-d/2-h/2, COL_MOT_REF, BG);
               get_string(p, motparx1+8*l, motpary1-d/2-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*motor).L);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(motparx1, motpary1+2*d))                       /* click (R) */
            {
               textout_ex(screen, font, "Insert the value of terminal Resistence (R): ", motparx1+d+2*l, motpary1+2*d-h/2, COL_MOT_REF, BG);
               get_string(p, motparx1+8*l, motpary1+2*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*motor).R);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(motparx1, motpary1+4.5*d))                       /* click (kb) */
            {
               textout_ex(screen, font, "Insert the value of back-EMF (kb): ", motparx1+d+2*l, motpary1+4.5*d-h/2, COL_MOT_REF, BG);
               get_string(p, motparx1+8*l, motpary1+4.5*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*motor).kb);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(motparx1, motpary1+7*d))                       /* click (kt) */
            {
               textout_ex(screen, font, "Insert the value of torque constant (kt): ", motparx1+d+2*l, motpary1+7*d-h/2, COL_MOT_REF, BG);
               get_string(p, motparx1+8*l, motpary1+7*d-h/2, COL_MOT_REF, BG);
                  
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*motor).kt);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(motparx1, motpary1+9.5*d))                      /* click (J) */
            {
               textout_ex(screen, font, "Insert the value of motor and load Inercia (J): ", motparx1+d+2*l, motpary1+9.5*d-h/2, COL_MOT_REF, BG);
               get_string(p, motparx1+8*l, motpary1+9.5*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*motor).J);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(motparx1, motpary1+12*d))                      /* click (b) */
            {
               textout_ex(screen, font, "Insert the value of friction constant (b): ", motparx1+d+2*l, motpary1+12*d-h/2, COL_MOT_REF, BG);
               get_string(p, motparx1+8*l, motpary1+12*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*motor).b);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(rectx1, recty1))                            /* click previus */
            {
               pthread_mutex_lock(&mux_page);
               page = WORK_PAGE;
               but = 0;
               pthread_mutex_unlock(&mux_page);
               layout();
            }
}








































