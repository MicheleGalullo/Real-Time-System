#include "global.h"
#include "functions.h"


/****************************************************************************************************************************************************************************/
/****************************************************************************************************************************************************************************/
/************************************************************************** UTILITY FUNCTION ********************************************************************************/
/****************************************************************************************************************************************************************************/
/****************************************************************************************************************************************************************************/


void get_keycodes(char *scan, char *ascii)
{
     int k;
     k = readkey();            // block until key is pressed
     *ascii = k;               // get ascii code
     *scan = k >> 8;           // get scan code
}

void get_string(char *str, int x, int y, int c, int b)
{
     char ascii, scan, s[2];
     int i = 0;
     
     do
     {
       get_keycodes(&scan, &ascii);
       if(scan != KEY_ENTER)
         {
           s[0] = ascii;
           s[1] = '\0';
           textout_ex(screen, font, s, x, y, c, b);
           x = x + 8;
           str[i++] = ascii;
         }
     }
     while(scan != KEY_ENTER);
     
     str[i] = '\0';
      
}

bool click_button(int x, int y) //forse serve stringa
{
    int mx, my; 
    
    if(mouse_b & 1)                                 /* press */
    {  
       scare_mouse();
       mx = mouse_x;
       my = mouse_y;
       unscare_mouse();
    }   
    
    rest(50);
    if((my <= y && my >= y-h) && (mx >= x && mx <= x+l))
       return true;
    else return false;   
       
      /* while((my <= y && my >= y-h) && (mx >= x && mx <= x+l)) 
       {  
          if((mouse_b & 1) == 0)                        
          {
             return true; 
          }
       }
     }
     return false; */
 
}


/****************************************************************************************************************************************************************************/
/************************************************************************** READ FUNCTIONS **********************************************************************************/
/****************************************************************************************************************************************************************************/

void read_data(struct controller *PID, struct select_motor *motor, char q[100])
{
     char s[10], c[10];
     FILE *fptr;
     if ((fptr = fopen( q, "r")) == NULL)
     {
         printf("Error! opening file");
         exit(1);         /* Program exits if file pointer returns NULL. */
     }
     while(fscanf(fptr,"%s %s", c, s) != EOF)
     { 
         /* read data for motor */
        
         if(strcmp(c, "L") == 0)
            (*motor).L = atof(s);
            
         if(strcmp(c, "R") == 0)
            (*motor).R = atof(s);
        
         if(strcmp(c, "J") == 0)
            (*motor).J = atof(s);
        
         if(strcmp(c, "b") == 0)
            (*motor).b = atof(s);
        
         if(strcmp(c, "kb") == 0)
            (*motor).kb = atof(s);
        
         if(strcmp(c, "kt") == 0)   
            (*motor).kt = atof(s);
         
         if(strcmp(c, "V") == 0)   
            (*motor).V = atof(s);
           
         /* read data for PID */
        
         if(strcmp(c, "KI") == 0)   
            (*PID).KI = atof(s);
        
         if(strcmp(c, "KP") == 0)   
            (*PID).KP = atof(s);   
        
         if(strcmp(c, "KD") == 0)   
            (*PID).KD = atof(s);
        
         if(strcmp(c, "ref") == 0)   
            (*PID).ref = atof(s);   
     }
     fclose(fptr);
   
}


/****************************************************************************************************************************************************************************/
/************************************************************************* THREADS FUNCTIONS ********************************************************************************/
/****************************************************************************************************************************************************************************/


void time_add_ms(struct timespec *t, int ms)
{
     t->tv_sec += ms/1000;
     t->tv_nsec += (ms%1000)*1000000;
     if(t->tv_nsec > 1000000000)  
     {
        t->tv_nsec -= 1000000000;
        t->tv_sec += 1;
     }
}     
     
void time_copy(struct timespec *td, struct timespec ts)
{
     td->tv_sec = ts.tv_sec;
     td->tv_nsec = ts.tv_nsec;
}

int time_cmp(struct timespec t1, struct timespec t2)
{
    if(t1.tv_sec > t2.tv_sec)   return 1; 
    if(t1.tv_sec < t2.tv_sec)   return -1;
    if(t1.tv_nsec > t2.tv_nsec) return 1;
    if(t1.tv_nsec < t2.tv_nsec) return -1;
    return 0;
}

void set_period(struct task_par *tp)
{
     struct timespec t;
     clock_gettime(CLOCK_MONOTONIC, &t);
     time_copy(&(tp->at), t);
     time_copy(&(tp->dl), t);
     time_add_ms(&(tp->at), tp->period);
     time_add_ms(&(tp->dl), tp->deadline);
}

void wait_for_period(struct task_par *tp)
{
     clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp->at), NULL);
     time_add_ms(&(tp->at), tp->period);
     time_add_ms(&(tp->dl), tp->period);
}

int deadline_miss(struct task_par *tp)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if(time_cmp(now, tp->dl) > 0)
    {
       tp->dmiss++;
       return 1;
    }
    return 0;
}

/****************************************************************************************************************************************************************************/
/********************************************************************* CONTROLLER FUNCTIONS *********************************************************************************/
/****************************************************************************************************************************************************************************/



void set_par(float *Ki, float *Kp, float *Kd, float *J, float *L, float *b, float *R, float *Ke, float *Kt, float *ref, struct controller *PID, struct select_motor *motor)
{

     *Kp = (*PID).KP;
     *Ki = (*PID).KI;
     *Kd = (*PID).KD;   
     *ref = (*PID).ref;
     
     *J = (*motor).J;  
     *b = (*motor).b;
     *R = (*motor).R; 
     *Ke = (*motor).kb;
     *Kt = (*motor).kt;
     *L =  (*motor).L;     
}

