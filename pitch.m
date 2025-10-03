%%
clc
clear all
close all

%%

            
act_acc_x=-0.978;
act_acc_y=-0.23;
act_acc_z=-0.01:0.0001:0.01;
for i=1:201
x_pitch(i)=USER_caculate_pitch(act_acc_y, act_acc_z(i), act_acc_x);
x_pitch_angle(i)=rad2deg(x_pitch(i));
end

stem(act_acc_z,x_pitch);
hold
stem(act_acc_z,x_pitch_angle);