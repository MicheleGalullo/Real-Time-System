#include "global.h"
#include "functions.h"

void tf_velocity(struct par_mgt *par_mgt, float Ki, float Kp, float Kd, float J, float L, float b, float R, float Ke, float Kt)
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
      
      // transfer function velocity control
      
      (*par_mgt).d11 = (*par_mgt).a1+1; 
      (*par_mgt).d12 = ((*par_mgt).E-1+(*par_mgt).a2)/(*par_mgt).d11; 
      (*par_mgt).d13 = ((*par_mgt).F-(*par_mgt).E+(*par_mgt).a3)/(*par_mgt).d11; 
      (*par_mgt).d14 = ((*par_mgt).a4-(*par_mgt).F)/(*par_mgt).d11; 
      (*par_mgt).d15 = (*par_mgt).a5/(*par_mgt).d11;
      (*par_mgt).n11 = (*par_mgt).a1/(*par_mgt).d11; 
      (*par_mgt).n12 = (*par_mgt).a2/(*par_mgt).d11; 
      (*par_mgt).n13 = (*par_mgt).a3/(*par_mgt).d11; 
      (*par_mgt).n14 = (*par_mgt).a4/(*par_mgt).d11; 
      (*par_mgt).n15 = (*par_mgt).a5/(*par_mgt).d11;
          
}


void control_velocity_compute(struct par_mgt par_mgt, float ref, float buff_in[], float aux_i[], float aux_v[], float out_vel[], int i)
{
     int j, k;
     
     if(i == 0)
           {
              for(k = 0; k < 5; k++)
              {
                  aux_i[k] = 0;
                  aux_v[k] = 0;
              }
              for(j = 0; j < 5; j++)
              {
                  if(j == 0)
                  {
                     buff_in[j] = ref;
                     out_vel[j] = par_mgt.n11*buff_in[j]+par_mgt.n12*aux_i[0]+par_mgt.n13*aux_i[1]+par_mgt.n14*aux_i[2]+par_mgt.n15*aux_i[3]
                                 -par_mgt.d12*aux_v[0]-par_mgt.d13*aux_v[1]-par_mgt.d14*aux_v[2]-par_mgt.d15*aux_v[3];
                  }
                else
                {
                   buff_in[j] = aux_i[j-1];
                   out_vel[j] = aux_v[j-1];
                }
              }
           }
           else
           {
              for(k = 0; k < 5; k++)
              {
                  aux_i[k] = buff_in[k];
                  aux_v[k] = *(out_vel +k);
              }
              for(j = 0; j < 5; j++)
              {
                  if(j == 0)
                  {
                     buff_in[j] = ref;
                     out_vel[j] = par_mgt.n11*buff_in[j]+par_mgt.n12*aux_i[0]+par_mgt.n13*aux_i[1]+par_mgt.n14*aux_i[2]+par_mgt.n15*aux_i[3]
                                  -par_mgt.d12*aux_v[0]-par_mgt.d13*aux_v[1]-par_mgt.d14*aux_v[2]-par_mgt.d15*aux_v[3];
                  }
                  else
                  {
                     buff_in[j] = aux_i[j-1];
                     out_vel[j] = aux_v[j-1];
                  }
              }
           }
}
