clc;
clear all;
close all;

% fs=5;   %������
% T=20;   %ʱ��
% n=round(T*fs);  %���������
% t=linspace(0,T,n);
s = serial('COM9'); % �������ڶ���
set(s, 'BaudRate', 115200); % ���ò�����
f_dataSwitch = 1;     %% 0 �ɼ����� 1 ���� 2 ֹͣ

flag = false;
try
    fopen(s);
    flag = true;
catch
    display( '���ڴ�ʧ��');
end
%%
t1=[0];
t2=[0];
t3=[0];
m1=[0];
m2=[0];
m3=[0];
g=9.8;
figure;
subplot(3,1,1);
p1 = plot(t1,m1,'EraseMode','background','MarkerSize',5); %����P���ؿ��趨��plot������
axis([x x+10 -2*g 2*g]);  %�����趨plot����ķ�Χ��ǰ2Ϊx�᣻��2Ϊy��
subplot(3,1,2);
p2 = plot(t2,m2,'EraseMode','background','MarkerSize',5); %����P���ؿ��趨��plot������\
axis([x x+10 -2*g 2*g]);  %�����趨plot����ķ�Χ��ǰ2Ϊx�᣻��2Ϊy��
subplot(3,1,3);
p3 = plot(t3,m3,'EraseMode','background','MarkerSize',5); %����P���ؿ��趨��plot������
axis([x x+10 -2*g 2*g]);  %�����趨plot����ķ�Χ��ǰ2Ϊx�᣻��2Ϊy��
grid on;
%% ��ȡ����
acc_k = 19 / 32768;
gyro_k = 1000 / 32768;
step=0;
%%
while (flag&&step<=1000)
    f_data = fread(s, 17);
    pause(0.01);
    step=step+1;
    GYRO_x = fun_hex2dec(f_data(5,1), f_data(6,1)) * gyro_k;
    GYRO_y = fun_hex2dec(f_data(7,1), f_data(8,1)) * gyro_k;
    GYRO_z = fun_hex2dec(f_data(9,1), f_data(10,1)) * gyro_k;
    ACC_x = fun_hex2dec(f_data(11,1), f_data(12,1)) * acc_k;
    ACC_y = fun_hex2dec(f_data(13,1), f_data(14,1)) * acc_k;
    ACC_z = fun_hex2dec(f_data(15,1), f_data(16,1)) * acc_k
    %--------------------------%    
    t1=[t1 step];  %-��vectorĩβ���һ��ֵ
    m1=[m1 ACC_z]; %-��vectorĩβ���һ��ֵ
    set(p1,'XData',t1,'YData',m1);   %-��plot����ķ���ֵ�趨����
    set(p2,'XData',t2,'YData',m2);   %-��plot����ķ���ֵ�趨����
    set(p3,'XData',t3,'YData',m3);   %-��plot����ķ���ֵ�趨����
%     stem(t1,m1);
%     hold;
    x=x+1;   %����ÿ������x����+0.1 
    axis([x x+5 -2*g 2*g]);  %����ʵ�������Ჽ��
    drawnow  limitrate %����drawnow���Ǳ��룬���ǿ�������ܣ������Ļ���ָ��
    %--------------------------%
%     if ACC_z == 0 && ACC_y ~= 0
%         roll = 90;
%     else
%         roll = rad2deg(atan(ACC_y / ACC_z));
%     end
%     
%     if ACC_z == 0 && ACC_y == 0
%         pitch = (ACC_x / abs(ACC_x)) * 90;
%     else
%         pitch = rad2deg(atan(ACC_x / sqrt(ACC_y^2 + ACC_z^2)));
%     end
%     
%     z_deg = rad2deg(acos(cos(deg2rad(roll)) * cos(deg2rad(pitch))))
        
end

disp('nn=0');
 fclose(s); % �رմ�������
 delete(s); % ɾ�����ڶ���
    


