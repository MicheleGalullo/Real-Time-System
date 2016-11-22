#include "global.h"
#include "functions.h"

void tf_position(struct par_mgt *par_mgt, float Ki, float Kp, float Kd, float J, float L, float b, float R, float Ke, float Kt)
{
      (*par_mgt).K1 = Kp+Ki*Ts/2+Kd/Ts;
      (*par_mgt).K2 = Ki*Ts/2-Kp-2*Kd/Ts;
      (*par_mgt).K3 = Kd/Ts; 

      (*par_mgt).D = (4*J*L)+2*Ts*(J*R+b*L)+Ts*Ts*(b*R+Ke*Kt);
      (*par_mgt).E = (2*Ts*Ts*(b*R+Ke*Kt)-8*J*L)/(*par_mgt).D;
      (*par_mgt).F = ((4*J*L)-2*Ts*(J*R+b*L)+Ts*Ts*(b*R+Ke*Kt))/(*par_mgt).D;
      (*par_mgt).G = (Kt*Ts*Ts)/(*par_mgt).D;
      (*par_mgt).H = 2*(*par_mgt).G;
      (*par_mgt).I = (*par_mgt).G;

      (*par_mgt).a1 = (*par_mgt).K1*(*par_mgt).G; 
      (*par_mgt).a2 = (*par_mgt).K1*(*par_mgt).H+(*par_mgt).K2*(*par_mgt).G; 
      (*par_mgt).a3 = (*par_mgt).K1*(*par_mgt).I+(*par_mgt).K2*(*par_mgt).H+(*par_mgt).K3*(*par_mgt).G; 
      (*par_mgt).a4 = (*par_mgt).K2*(*par_mgt).I+(*par_mgt).K3*(*par_mgt).H; 
      (*par_mgt).a5 = (*par_mgt).K3*(*par_mgt).I;
      (*par_mgt).b1 = Ts*(*par_mgt).a1; 
      (*par_mgt).b2 = Ts*((*par_mgt).a1+(*par_mgt).a2); 
      (*par_mgt).b3 = Ts*((*par_mgt).a2+(*par_mgt).a3); 
      (*par_mgt).b4 = Ts*((*par_mgt).a3+(*par_mgt).a4); 
      (*par_mgt).b5 = Ts*((*par_mgt).a4+(*par_mgt).a5); 
      (*par_mgt).b6 = Ts*(*par_mgt).a5;
      
      // transfer function position control
      
      (*par_mgt).d21 = 2+(*par_mgt).b1; 
      (*par_mgt).d22 = (2*(*par_mgt).E-4+(*par_mgt).b2)/(*par_mgt).d21; 
      (*par_mgt).d23 = (2*(*par_mgt).F-4*(*par_mgt).E+2+(*par_mgt).b3)/(*par_mgt).d21; 
      (*par_mgt).d24 = (2*(*par_mgt).E-4*(*par_mgt).F+(*par_mgt).b4)/(*par_mgt).d21; 
      (*par_mgt).d25 = (2*(*par_mgt).F+(*par_mgt).b5)/(*par_mgt).d21; 
      (*par_mgt).d26 = (*par_mgt).b6/(*par_mgt).d21;
      (*par_mgt).n21 = (*par_mgt).b1/(*par_mgt).d21; 
      (*par_mgt).n22 = (*par_mgt).b2/(*par_mgt).d21; 
      (*par_mgt).n23 = (*par_mgt).b3/(*par_mgt).d21; 
      (*par_mgt).n24 = (*par_mgt).b4/(*par_mgt).d21; 
      (*par_mgt).n25 = (*par_mgt).b5/(*par_mgt).d21; 
      (*par_mgt).n26 = (*par_mgt).b6/(*par_mgt).d21;

}

void control_position_compute(struct par_mgt par_mgt, float ref, float buff_in[], float aux_i[], float aux_p[], float out_pos[], int i)
{
     int j, k;
     
     if(i == 0)
           {
              for(k = 0; k < 5; k++)
              {
                  aux_i[k] = 0;
                  aux_p[k] = 0;
              }
              
              for(j = 0; j < 5; j++)
              {
                  if(j == 0)
                  {
                     buff_in[j] = ref;
                     out_pos[j] = par_mgt.n21*buff_in[j]+par_mgt.n22*aux_i[0]+par_mgt.n23*aux_i[1]+par_mgt.n24*aux_i[2]+par_mgt.n25*aux_i[3]
                                  +par_mgt.n26*aux_i[4]-par_mgt.d22*aux_p[0]-par_mgt.d23*aux_p[1]-par_mgt.d24*aux_p[2]-par_mgt.d25*aux_p[3]
                                  -par_mgt.d26*aux_p[4];
                  }
                  else 
                  {
                     buff_in[j] = aux_i[j-1];
                     out_pos[j] = aux_p[j-1];
                  }
              }
           }
           else
           {
              for(k = 0; k < 5; k++)
              {
                  aux_i[k] = buff_in[k];
                  aux_p[k] = out_pos[k];
              }
              for(j = 0; j < 5; j++)
              {
                  if(j == 0)
                  {
                      buff_in[j] = ref;
                      out_pos[j] = par_mgt.n21*buff_in[j]+par_mgt.n22*aux_i[0]+par_mgt.n23*aux_i[1]+par_mgt.n24*aux_i[2]+par_mgt.n25*aux_i[3]
                                   +par_mgt.n26*aux_i[4]-par_mgt.d22*aux_p[0]-par_mgt.d23*aux_p[1]-par_mgt.d24*aux_p[2]-par_mgt.d25*aux_p[3]
                                   -par_mgt.d26*aux_p[4];
                  }
                  else
                  {
                      buff_in[j] = aux_i[j-1];
                      out_pos[j] = aux_p[j-1];
                  }
              }
           }
     
     
}
