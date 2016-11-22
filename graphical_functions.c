#include "global.h"
#include "functions.h"



/****************************************************************************************************************************************************************************/
/********************************************************************** GRAPHICAL FUNCTIONS *********************************************************************************/
/****************************************************************************************************************************************************************************/





void button_set(int x, int y, char s[40])  // mettere stringa 20 definire dopo
{
     rectfill(screen, x, y, x+l, y-h, COL_BUT);
     textout_centre_ex(screen, font, s, x+l/2, y-h-dh, COL_MOT_REF, BG);
} 

void geo_motor(int x, int y, long double theta)
{
     int dr = 10; 
     circlefill(screen, x, y, r, COL_MOT);
     triangle(screen, x+(r+dr/2), y, x+(r+3*dr/2), y+dr, x+(r+3*dr/2), y-dr, COL_REF);
     line(screen, x, y, x+(r)*cos(theta*(PI/180)), y+(r)*sin(theta*(PI/180)), COL_MOT_REF);
}

void axis(int x0, int y0, char s[40])
{
     /* x axis */
     
     line(screen, x0, y0, x0+dx, y0, COL_BUT);        /* (sezxm-d) the lenght of x axis is the same of lenght of space for motor */
     triangle(screen, x0+sezxm-3*d/2, y0, x0+dx-5, y0+5, x0+dx-5, y0-5, COL_BUT);
     textout_centre_ex(screen, font, "time", x0+sezxm-d-15, y0+10, COL_BUT, BG);
    
     /* y axis */
     
     line(screen, x0, y0, x0, y0-dy, COL_BUT);
     triangle(screen, x0, y0-dy, x0-5, y0-dy+5, x0+5, y0-dy+5, COL_BUT);
     textout_centre_ex(screen, font, s, x0, y0-dy-10, COL_BUT, BG);
}


void layout_base(char s[100])
{
     set_window_title(s);
     clear_to_color(screen, COL_BG);
     
     /* shared screen */
     
     line(screen, sezx, 0, sezx, sezy, COL_BUT);
     line(screen, 0, sezy, sezx, sezy, COL_BUT);
     textout_centre_ex(screen, font, "COMMAND WATCH", sezx + (scrx-sezx)/2, d/2, COL_BUT, BG);
}





void vertical_layout(int x, int y, int *n, int *n_n, char s[100])
{
     
     textout_centre_ex(screen, font, s, x, y+r+d/2, COL_BUT, BG);
     if(*n > 0)
     {
        button_set(x-r, sezy+d+h, "Set PID");
        button_set(x+d/2, sezy+d+h, "Set motor");
     }
     button_set(x-r, sezy+2*(d+h), "Add");
     
     if(*n > 1)                                           /* the next button is displayed only there are more than 1 of motor */
     {   
        button_set(x+d/2, sezy+2*(d+h), "Next");
        
        sprintf(p,"%d/%d",*n_n+1,*n);                     /* print the numebr of motor that we watch, plus 1 so we can forget the array */     
        textout_ex(screen, font, p, x-r, y-r, COL_BUT, COL_BG);
     }
        
}

