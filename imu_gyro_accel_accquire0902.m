%ע�⡪��ʹ�ô˳���֮ǰ����Ҫ����sscoem�ϴ򿪴��ڣ���imu�·�03ָ�Ȼ��ر�sscom�ϵĴ���
%�κ�û�����maxPoints = 2000�Ĳ�����Ϊ������Ҫ�ֶ�ִ��fclose(s)���д��ڹرգ���
clc;
clear;
close all;

fs = 5;   % ������
T = 20;   % ʱ��
n = round(T*fs);  % ���������
s = serial('COM7'); % �������ڶ���
set(s, 'BaudRate', 115200); % ���ò�����
dataSwitch = 0;     % 0 �ɼ����� 1 ���� 2 ֹͣ

flag = false;
try
    fopen(s);
    flag = true;
    disp('���ڴ򿪳ɹ�');
catch
    error('���ڴ�ʧ��');
end

% ����/�رշ���
if flag
    if dataSwitch == 1
        hexdata = 'aa 55 03 00 00 00 00 00 00 00 00 00 00 00 00 CA 0D'; % ����
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
        data = fread(s, 17);
    elseif dataSwitch == 2
        hexdata = 'aa 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0D'; % �ر�
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
    else
        % ��ʼ�����ݴ洢
        maxPoints = 1000;
        timeData = zeros(1, maxPoints);
        
        % GYRO��������
        gyroXData = zeros(1, maxPoints, 'int16');
        gyroYData = zeros(1, maxPoints, 'int16');
        gyroZData = zeros(1, maxPoints, 'int16');
        
        % ACCEL��������
        accelXData = zeros(1, maxPoints, 'int16');
        accelYData = zeros(1, maxPoints, 'int16');
        accelZData = zeros(1, maxPoints, 'int16');
        
        % ����ͼ�δ��ں���ͼ - 3x2����
        figure('Position', [50, 50, 1400, 900], 'Name', 'IMU����������ʵʱ���');
        
        % GYRO X������
        subplot(3, 2, 1);
        h_gyro_x = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
        hold on;
        % ���0�ο���
        h_zero_x = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('GYRO X������');
        xlabel('������');
        ylabel('���ٶ�');
        ylim([-32768 32767]); % ͳһ�����귶Χ
        grid on;
        
        % GYRO Y������
        subplot(3, 2, 2);
        h_gyro_y = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
        hold on;
        % ���0�ο���
        h_zero_y = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('GYRO Y������');
        xlabel('������');
        ylabel('���ٶ�');
        ylim([-32768 32767]); % ͳһ�����귶Χ
        grid on;
        
        % GYRO Z������
        subplot(3, 2, 3);
        h_gyro_z = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
        hold on;
        % ���0�ο���
        h_zero_z = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('GYRO Z������');
        xlabel('������');
        ylabel('���ٶ�');
        ylim([-32768 32767]); % ͳһ�����귶Χ
        grid on;
        
        % ACCEL X������
        subplot(3, 2, 4);
        h_accel_x = plot(NaN, NaN, 'm-', 'LineWidth', 1.5);
        hold on;
        % ���0�ο���
        h_zero_accel_x = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('ACCEL X������');
        xlabel('������');
        ylabel('���ٶ�');
        ylim([-32768 32767]); % ͳһ�����귶Χ
        grid on;
        
        % ACCEL Y������
        subplot(3, 2, 5);
        h_accel_y = plot(NaN, NaN, 'c-', 'LineWidth', 1.5);
        hold on;
        % ���0�ο���
        h_zero_accel_y = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('ACCEL Y������');
        xlabel('������');
        ylabel('���ٶ�');
        ylim([-32768 32767]); % ͳһ�����귶Χ
        grid on;
        
        % ACCEL Z������
        subplot(3, 2, 6);
        h_accel_z = plot(NaN, NaN, 'k-', 'LineWidth', 1.5);
        hold on;
        % ���0�ο���
        h_zero_accel_z = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('ACCEL Z������');
        xlabel('������');
        ylabel('���ٶ�');
        ylim([-32768 32767]); % ͳһ�����귶Χ
        grid on;
        
        % ֡ͬ��
        syncBytes = [170, 85]; % AA 55
        syncFound = false;
        
        % ��ʾ����
        fprintf('������\tGYRO_X(HEX)\tGYRO_X(INT16)\tGYRO_Y(HEX)\tGYRO_Y(INT16)\tGYRO_Z(HEX)\tGYRO_Z(INT16)\tACCEL_X(HEX)\tACCEL_X(INT16)\tACCEL_Y(HEX)\tACCEL_Y(INT16)\tACCEL_Z(HEX)\tACCEL_Z(INT16)\n');
        fprintf('------\t-----------\t-------------\t-----------\t-------------\t-----------\t-------------\t------------\t-------------\t------------\t-------------\t------------\t-------------\n');
        
        while ~syncFound
            byte1 = fread(s, 1, 'uint8');
            if ~isempty(byte1) && byte1 == syncBytes(1)
                byte2 = fread(s, 1, 'uint8');
                if ~isempty(byte2) && byte2 == syncBytes(2)
                    syncFound = true;
                    disp('֡ͬ���ɹ�');
                end
            end
        end
        
        % ���ݲɼ�ѭ��
        for i = 1:maxPoints
            try
                % ��ȡ�������ݰ� (17�ֽ�)
                dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
                
                % ��ȡԭʼHEXֵ
                gyroX_hex = sprintf('%02X%02X', dataPacket(5), dataPacket(6));
                gyroY_hex = sprintf('%02X%02X', dataPacket(7), dataPacket(8));
                gyroZ_hex = sprintf('%02X%02X', dataPacket(9), dataPacket(10));
                accelX_hex = sprintf('%02X%02X', dataPacket(11), dataPacket(12));
                accelY_hex = sprintf('%02X%02X', dataPacket(13), dataPacket(14));
                accelZ_hex = sprintf('%02X%02X', dataPacket(15), dataPacket(16));
                
                % ת��Ϊint16�з�������
                gyroX_int = typecast(uint16(hex2dec(gyroX_hex)), 'int16');
                gyroY_int = typecast(uint16(hex2dec(gyroY_hex)), 'int16');
                gyroZ_int = typecast(uint16(hex2dec(gyroZ_hex)), 'int16');
                accelX_int = typecast(uint16(hex2dec(accelX_hex)), 'int16');
                accelY_int = typecast(uint16(hex2dec(accelY_hex)), 'int16');
                accelZ_int = typecast(uint16(hex2dec(accelZ_hex)), 'int16');
                
                % �洢����
                gyroXData(i) = gyroX_int;
                gyroYData(i) = gyroY_int;
                gyroZData(i) = gyroZ_int;
                accelXData(i) = accelX_int;
                accelYData(i) = accelY_int;
                accelZData(i) = accelZ_int;
                timeData(i) = i;
                
                % ÿ10�����������һ��HEX��INT16ֵ
                if mod(i, 10) == 0 || i == 1
                    fprintf('%d\t%s\t%d\t\t%s\t%d\t\t%s\t%d\t\t%s\t%d\t\t%s\t%d\t\t%s\t%d\n', ...
                        i, ...
                        gyroX_hex, gyroX_int, ...
                        gyroY_hex, gyroY_int, ...
                        gyroZ_hex, gyroZ_int, ...
                        accelX_hex, accelX_int, ...
                        accelY_hex, accelY_int, ...
                        accelZ_hex, accelZ_int);
                end
                
                % ����GYRO����ͼ��
                subplot(3, 2, 1);
                set(h_gyro_x, 'XData', timeData(1:i), 'YData', double(gyroXData(1:i)));
                
                subplot(3, 2, 2);
                set(h_gyro_y, 'XData', timeData(1:i), 'YData', double(gyroYData(1:i)));
                
                subplot(3, 2, 3);
                set(h_gyro_z, 'XData', timeData(1:i), 'YData', double(gyroZData(1:i)));
                
                % ����ACCEL����ͼ��
                subplot(3, 2, 4);
                set(h_accel_x, 'XData', timeData(1:i), 'YData', double(accelXData(1:i)));
                
                subplot(3, 2, 5);
                set(h_accel_y, 'XData', timeData(1:i), 'YData', double(accelYData(1:i)));
                
                subplot(3, 2, 6);
                set(h_accel_z, 'XData', timeData(1:i), 'YData', double(accelZData(1:i)));
                
                % ��̬����X�᷶Χ
                if i > 50
                    for j = 1:6
                        subplot(3, 2, j);
                        xlim([max(1, i-50), min(maxPoints, i+10)]);
                    end
                end
                
                drawnow limitrate; % ���Ƹ���Ƶ�����������
                
                % �����һ֡��ͬ���ֽ�
                nextBytes = fread(s, 2, 'uint8');
                if length(nextBytes) == 2 && isequal(nextBytes', syncBytes)
                    % ͬ����ȷ������
                    continue;
                else
                    % ����ͬ��
                    syncFound = false;
                    while ~syncFound
                        byte1 = fread(s, 1, 'uint8');
                        if ~isempty(byte1) && byte1 == syncBytes(1)
                            byte2 = fread(s, 1, 'uint8');
                            if ~isempty(byte2) && byte2 == syncBytes(2)
                                syncFound = true;
                                disp('����ͬ���ɹ�');
                            end
                        end
                    end
                end
                
            catch ME
                warning('���ݶ�ȡ����: %s', ME.message);
                % ��������ͬ��
                syncFound = false;
                while ~syncFound
                    byte1 = fread(s, 1, 'uint8');
                    if ~isempty(byte1) && byte1 == syncBytes(1)
                        byte2 = fread(s, 1, 'uint8');
                        if ~isempty(byte2) && byte2 == syncBytes(2)
                            syncFound = true;
                            disp('���������ͬ���ɹ�');
                        end
                    end
                end
            end
        end
    end
end

fclose(s); % �رմ�������
delete(s); % ɾ�����ڶ���