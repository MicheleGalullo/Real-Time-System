#include "global.h"
#include "functions.h"

void tf_torque(struct par_mgt *par_mgt, float Ki, float Kp, float Kd, float J, float L, float b, float R, float Ke, float Kt)
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
      (*par_mgt).x = 2*J+b*Ts; 
      (*par_mgt).y = b*Ts-2*J;
      (*par_mgt).c1 = ((*par_mgt).a1*(*par_mgt).x)/Ts; 
      (*par_mgt).c2 = ((*par_mgt).a2*(*par_mgt).x+(*par_mgt).a1*(*par_mgt).y)/Ts; 
      (*par_mgt).c3 = ((*par_mgt).a3*(*par_mgt).x+(*par_mgt).a2*(*par_mgt).y)/Ts;
      (*par_mgt).c4 = ((*par_mgt).a4*(*par_mgt).x+(*par_mgt).a3*(*par_mgt).y)/Ts; 
      (*par_mgt).c5 = ((*par_mgt).a5*(*par_mgt).x+(*par_mgt).a4*(*par_mgt).y)/Ts;
      (*par_mgt).c6 = ((*par_mgt).a5*(*par_mgt).y)/Ts; 
      
      // transfer function torque control
      
      (*par_mgt).d31 = 1+(*par_mgt).c1; 
      (*par_mgt).d32 = ((*par_mgt).E-2+(*par_mgt).c2)/(*par_mgt).d31; 
      (*par_mgt).d33 = ((*par_mgt).F-2*(*par_mgt).E+1+(*par_mgt).c3)/(*par_mgt).d31; 
      (*par_mgt).d34 = ((*par_mgt).E-2*(*par_mgt).F+(*par_mgt).c4)/(*par_mgt).d31; 
      (*par_mgt).d35 = ((*par_mgt).F+(*par_mgt).c5)/(*par_mgt).d31; 
      (*par_mgt).d36 = (*par_mgt).c6/(*par_mgt).d31;
      (*par_mgt).n31 = (*par_mgt).c1/(*par_mgt).d31; 
      (*par_mgt).n32 = (*par_mgt).c2/(*par_mgt).d31; 
      (*par_mgt).n33 = (*par_mgt).c3/(*par_mgt).d31; 
      (*par_mgt).n34 = (*par_mgt).c4/(*par_mgt).d31; 
      (*par_mgt).n35 = (*par_mgt).c5/(*par_mgt).d31; 
      (*par_mgt).n36 = (*par_mgt).c6/(*par_mgt).d31;
            
}

void control_torque_compute(struct par_mgt par_mgt, float ref, float buff_in[], float aux_i[], float aux_t[], float out_tor[], int i)
{           
          
           int j, k;
           
           if(i == 0)
           {
              for(k = 0; k < 5; k++)
              {
                  aux_i[k] = 0;
                  aux_t[k] = 0;
              }
              for(j = 0; j < 5; j++)
              {
                  if(j == 0)
                  {
                    buff_in[j] = ref;
                    
                    out_tor[j] = par_mgt.n31*buff_in[j]+par_mgt.n32*aux_i[0]+par_mgt.n33*aux_i[1]+par_mgt.n34*aux_i[2]
                                 +par_mgt.n35*aux_i[3]+par_mgt.n36*aux_i[4]-par_mgt.d32*aux_t[0]-par_mgt.d33*aux_t[1]
                                 -par_mgt.d34*aux_t[2]-par_mgt.d35*aux_t[3]-par_mgt.d36*aux_t[4];
                  }
                else
                {
                   buff_in[j] = aux_i[j-1];
                   out_tor[j] = aux_t[j-1];
                }
              }
           }
           else
           {
              for(k = 0; k < 5; k++)
              {
                  aux_i[k] = buff_in[k];
                  aux_t[k] = out_tor[k];
              }
              for(j = 0; j < 5; j++)
              {
                  if(j == 0)
                  {
                     buff_in[j] = ref;
                     
                     out_tor[j] = par_mgt.n31*buff_in[j]+par_mgt.n32*aux_i[0]+par_mgt.n33*aux_i[1]+par_mgt.n34*aux_i[2]
                                  +par_mgt.n35*aux_i[3]+par_mgt.n36*aux_i[4]-par_mgt.d32*aux_t[0]-par_mgt.d33*aux_t[1]
                                  -par_mgt.d34*aux_t[2]-par_mgt.d35*aux_t[3]-par_mgt.d36*aux_t[4];
                  }
                  else
                  {
                     buff_in[j] = aux_i[j-1];
                     out_tor[j] = aux_t[j-1];
                  }
              }
           }
}
