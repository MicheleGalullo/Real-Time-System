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
#include "functions.h"


void initialize_system_threads()
{
     int i;
     
     /* threads for control in position */
     
     for(i = 0; i < Nm; i++)   
     {
        t_param_p[i].arg = i;
        t_param_p[i].period = SYS_PERIOD;
        t_param_p[i].deadline = SYS_DEADLINE;
        t_param_p[i].priority = SYS_PRIORITY;
        t_param_p[i].dmiss = 0;
        
        pthread_attr_init(&my_attr_p[i]);
        pthread_attr_setinheritsched(&my_attr_p[i], PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&my_attr_p[i], SCHED_RR);
        s_param_p.sched_priority = t_param_p[i].priority;
        pthread_attr_setschedparam(&my_attr_p[i], &s_param_p);
        
        pthread_mutex_init(&mux_p[i], NULL);
        
     }      
 
     /* threads for control in velocity */
     
     for(i = 0; i < Nm; i++)   
     {
        t_param_v[i].arg = i;
        t_param_v[i].period = SYS_PERIOD;
        t_param_v[i].deadline = SYS_DEADLINE;
        t_param_v[i].priority = SYS_PRIORITY;
        t_param_v[i].dmiss = 0;
        
        pthread_attr_init(&my_attr_v[i]);
        pthread_attr_setinheritsched(&my_attr_v[i], PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&my_attr_v[i], SCHED_RR);
        s_param_v.sched_priority = t_param_v[i].priority;
        pthread_attr_setschedparam(&my_attr_v[i], &s_param_v);
        
        pthread_mutex_init(&mux_v[i], NULL);
        
     }
     
     /* threads for control in torque */
     
     for(i = 0; i < Nm; i++)   
     {
        t_param_t[i].arg = i;
        t_param_t[i].period = SYS_PERIOD;
        t_param_t[i].deadline = SYS_DEADLINE;
        t_param_t[i].priority = SYS_PRIORITY;
        t_param_t[i].dmiss = 0;
        
        pthread_attr_init(&my_attr_t[i]);
        pthread_attr_setinheritsched(&my_attr_t[i], PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&my_attr_t[i], SCHED_RR);
        s_param_t.sched_priority = t_param_t[i].priority;
        pthread_attr_setschedparam(&my_attr_t[i], &s_param_t);
        
        pthread_mutex_init(&mux_t[i], NULL); 
     }         
     
     for(i = 0; i < 3; i++)
        pthread_mutex_init(&mux_us[i], NULL);
    
    pthread_mutex_init(&mux_page, NULL);
     
}



void initialize_graph_threads()
{

     
        t_param_g.period = GRAPHIC_PERIOD;
        t_param_g.deadline = GRAPHIC_DEADLINE;
        t_param_g.priority = GRAPHIC_PRIORITY;
        t_param_g.dmiss = 0;
        
        pthread_attr_init(&my_attr_g);
        pthread_attr_setinheritsched(&my_attr_g, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&my_attr_g, SCHED_RR);
        s_param_g.sched_priority = t_param_g.priority;
        pthread_attr_setschedparam(&my_attr_g, &s_param_g);
        
        
}

void mouse_thread()
{
     t_param_m.period = MOUSE_PERIOD;
     t_param_m.deadline = MOUSE_DEADLINE;
     t_param_m.priority = MOUSE_PRIORITY;
     t_param_m.dmiss = 0;
        
     pthread_attr_init(&my_attr_m);
     pthread_attr_setinheritsched(&my_attr_m, PTHREAD_EXPLICIT_SCHED);
     pthread_attr_setschedpolicy(&my_attr_m, SCHED_RR);
     s_param_m.sched_priority = t_param_m.priority;
     pthread_attr_setschedparam(&my_attr_m, &s_param_m);
     
     if(pthread_create(&thread_m, &my_attr_m, mouse_task , &t_param_m))
        allegro_message("Mouse thread not create!\n");
}



int main()
{
    
    if(allegro_init() != 0)
    {
       allegro_message("Cannot initilize Allegro!\n");
       return 1;
    }
    
    if (install_keyboard())
    {
       allegro_message("Cannot initalize keyboard input!\n");
       return 1;
    }
    
    if(install_mouse() == -1)
    {
       allegro_message("Error !! %s", allegro_error);
    }
    
    /* create a windows for graphics options */
    
    set_color_depth(8);
    set_gfx_mode(GFX_AUTODETECT_WINDOWED, scrx, scry, 0, 0);
    
    
    
    initialize_system_threads();
    
    layout();
    
    initialize_graph_threads();
    mouse_thread();
    
    
    pthread_join(thread_m, NULL);
    pthread_join(thread_g, NULL);
    
    int i;
    for(i = 0; i < np; i++)
       pthread_join(thread_p[i], NULL);
     
    for(i = 0; i < nv; i++)
       pthread_join(thread_v[i], NULL);

    for(i = 0; i < nt; i++)
       pthread_join(thread_t[i], NULL);
    
    
    allegro_exit();
   
    for(i = 0; i < Nm; i++)
    {
        pthread_attr_destroy(&my_attr_p[i]);
        pthread_attr_destroy(&my_attr_v[i]);
        pthread_attr_destroy(&my_attr_t[i]); 
        
    }
    
    pthread_attr_destroy(&my_attr_g);
    pthread_attr_destroy(&my_attr_m);
    
   
    return 0;
    
}END_OF_MAIN();   



