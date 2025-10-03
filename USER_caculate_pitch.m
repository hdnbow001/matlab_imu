function m_pitch = USER_caculate_pitch(act_acc_y, act_acc_z, act_acc_x)

if ((act_acc_z == 0) && (act_acc_y == 0))
  m_pitch = ((act_acc_x/fabsf(act_acc_x))*(U_M_PI/2));
else
  %m_pitch = atan(act_acc_x/sqrt(act_acc_y.^2+act_acc_z.^2));
  m_pitch = atan(act_acc_x/abs(act_acc_z));
end 	
            
if((act_acc_z > 0) && (act_acc_x > 0))
  m_pitch = m_pitch;
elseif((act_acc_z < 0) && (act_acc_x > 0))
  m_pitch =pi-m_pitch;
elseif((act_acc_z > 0) && (act_acc_x < 0))
  m_pitch =2*pi+m_pitch;
else
  m_pitch =pi-m_pitch;
end