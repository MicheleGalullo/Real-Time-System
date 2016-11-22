#include "global.h"
#include "functions.h"


void control_pid_par()
{
    
     clear_to_color(screen, COL_BG);
     
     //show_mouse(screen);
     layout_base("Control PID parameters");
               
     /* command watch screen set buttons on the right */
     
     button_set(rectx1, recty1, "Previus");
     
     /* shared screen for PID controller */
     
     textout_centre_ex(screen, font, "SET THE GAINS FOR CONTROLLER", sezx/2, d, COL_BUT, BG);
     
     button_set(pidx1,pidy1+2*d, "Proportional gain");
     button_set(pidx1, pidy1+5*d, "Integral action gain");
     button_set(pidx1, pidy1+8*d, "Derivative action gain");
     button_set(pidx1, pidy1+11*d, "reference");
     
}


void display_gains(int x, int y, struct controller *PID)
{
     textout_ex(screen,font, "GAINS OF PID CONTORLLER", x-r, y+r+2*d, COL_MOT_REF, BG);
           
     sprintf(p, "proportional gain: %.3f", (*PID).KP);
     textout_ex(screen,font, p, x-r, y+r+d+2*(d+h/2), COL_MOT_REF, BG);
     sprintf(p, "integral gain: %.3f", (*PID).KI);
     textout_ex(screen,font, p, x-r, y+r+d+3*(d+h/2), COL_MOT_REF, BG);
     sprintf(p, "derivative gain: %.3f", (*PID).KD);
     textout_ex(screen,font, p, x-r, y+r+d+4*(d+h/2), COL_MOT_REF, BG);
     sprintf(p, "reference: %.3f", (*PID).ref);
     textout_ex(screen,font, p, x-r, y+r+d+5*(d+h/2), COL_MOT_REF, BG);
}

void mouse_task_set_PID_par(struct controller *PID, int sem)
{
            if(click_button(pidx1,pidy1+2*d))                                                   /* set the proportional gain */
            {
               textout_ex(screen, font, "Insert the value of proportional gain (KP): ", pidx1+3.5*l, pidy1+2*d-h/2, COL_MOT_REF, BG);
               get_string(p, pidx1+9*l, pidy1+2*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*PID).KP);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(pidx1, pidy1+5*d))                                                  /* set the integral gain */
            {
               textout_ex(screen, font, "Insert the value of integral gain (KI): ", pidx1+3.5*l, pidy1+5*d-h/2, COL_MOT_REF, BG);
               get_string(p, pidx1+9*l, pidy1+5*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*PID).KI);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(pidx1, pidy1+8*d))                                                 /* set the derivative gain */
            {
               textout_ex(screen, font, "Insert the value of derivative gain (KD): ", pidx1+3.5*l, pidy1+8*d-h/2, COL_MOT_REF, BG);
               get_string(p, pidx1+9*l, pidy1+8*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*PID).KD);
               pthread_mutex_unlock(&mux_us[sem]);
            }
            if(click_button(pidx1, pidy1+11*d))                                                 /* set the reference gain */
            {
               textout_ex(screen, font, "Insert the value of reference (rad, rad/sec, Nm): ", pidx1+3.5*l, pidy1+11*d-h/2, COL_MOT_REF, BG);
               get_string(p, pidx1+9*l, pidy1+11*d-h/2, COL_MOT_REF, BG);
               
               pthread_mutex_lock(&mux_us[sem]);
               sscanf(p, "%f", &(*PID).ref);
               pthread_mutex_unlock(&mux_us[sem]);
               
               pthread_mutex_lock(&mux_page);
               page = WORK_PAGE;
               but = 1;
               pthread_mutex_unlock(&mux_page);
               
               layout();
            }
            if(click_button(rectx1, recty1))                                                    /* click previus */
            {   
               pthread_mutex_lock(&mux_page);
               page = WORK_PAGE;
               but = 0;
               pthread_mutex_unlock(&mux_page);
               
               layout();
            }
}
