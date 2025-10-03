clc;
clear;
close all;

fs=5;   %������
T=20;   %ʱ��
n=round(T*fs);  %���������
t=linspace(0,T,n);
s = serial('COM7'); % �������ڶ���
set(s, 'BaudRate', 115200); % ���ò�����
dataSwitch = 0;     %% 0 �ɼ����� 1 ���� 2 ֹͣ

flag = false;
try
    fopen(s);
    flag = true;
catch
    display( '���ڴ�ʧ��');
end



%% ����/�رշ���
if flag
    if dataSwitch == 1
        hexdata = 'aa 55 05 00 00 00 00 00 00 00 00 00 00 00 00 CA 0D'; %%����
        asciiValues = hex2dec(split(hexdata));
        fwrite(s,asciiValues);
        data = fread(s, 17);
    elseif  dataSwitch == 2
        hexdata = 'aa 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0D';  %%�ر�
        asciiValues = hex2dec(split(hexdata));
        fwrite(s,asciiValues);
%         data = fread(s, 17);
    else
        acc_k = 2 / 32768;
        gyro_k = 2000 / 32768;
        data = zeros(17,n);
        
        ACC_x = 0;
        t = 0;
        figure;
        h = plot(t, ACC_x, 'r');
        ylim([0 360])
        hold on;
        
        for i = 1 : 1000
            data(:, i) = fread(s, 17);
            %ACC_x = fun_hex2dec(data(5,i), data(6,i)); %����show roll angle
            ACC_x = fun_hex2dec(data(7,i), data(8,i)); %����show pitch angle

            if length(h.XData) > 10
                ydata = [h.YData(2:end),ACC_x];
                xdata = [h.XData(2:end),i];
            else
                ydata = [h.YData(1:end),ACC_x];
                xdata = [h.XData(1:end),i]
            end
            set(h, 'XData', xdata, 'YData', ydata); 
            drawnow;
%             pause(0.2);
        end
    end
end

 fclose(s); % �رմ�������
 delete(s); % ɾ�����ڶ���
    


