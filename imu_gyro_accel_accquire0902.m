%注意——使用此程序之前，需要先在sscoem上打开串口，给imu下发03指令，然后关闭sscom上的串口
%任何没有完成maxPoints = 2000的采样行为，都需要手动执行fclose(s)进行串口关闭！！
clc;
clear;
close all;

fs = 5;   % 采样率
T = 20;   % 时宽
n = round(T*fs);  % 采样点个数
s = serial('COM7'); % 创建串口对象
set(s, 'BaudRate', 115200); % 设置波特率
dataSwitch = 0;     % 0 采集数据 1 启动 2 停止

flag = false;
try
    fopen(s);
    flag = true;
    disp('串口打开成功');
catch
    error('串口打开失败');
end

% 启动/关闭发送
if flag
    if dataSwitch == 1
        hexdata = 'aa 55 03 00 00 00 00 00 00 00 00 00 00 00 00 CA 0D'; % 启动
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
        data = fread(s, 17);
    elseif dataSwitch == 2
        hexdata = 'aa 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0D'; % 关闭
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
    else
        % 初始化数据存储
        maxPoints = 1000;
        timeData = zeros(1, maxPoints);
        
        % GYRO三轴数据
        gyroXData = zeros(1, maxPoints, 'int16');
        gyroYData = zeros(1, maxPoints, 'int16');
        gyroZData = zeros(1, maxPoints, 'int16');
        
        % ACCEL三轴数据
        accelXData = zeros(1, maxPoints, 'int16');
        accelYData = zeros(1, maxPoints, 'int16');
        accelZData = zeros(1, maxPoints, 'int16');
        
        % 创建图形窗口和子图 - 3x2布局
        figure('Position', [50, 50, 1400, 900], 'Name', 'IMU传感器数据实时监控');
        
        % GYRO X轴数据
        subplot(3, 2, 1);
        h_gyro_x = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
        hold on;
        % 添加0参考线
        h_zero_x = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('GYRO X轴数据');
        xlabel('采样点');
        ylabel('角速度');
        ylim([-32768 32767]); % 统一纵坐标范围
        grid on;
        
        % GYRO Y轴数据
        subplot(3, 2, 2);
        h_gyro_y = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
        hold on;
        % 添加0参考线
        h_zero_y = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('GYRO Y轴数据');
        xlabel('采样点');
        ylabel('角速度');
        ylim([-32768 32767]); % 统一纵坐标范围
        grid on;
        
        % GYRO Z轴数据
        subplot(3, 2, 3);
        h_gyro_z = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
        hold on;
        % 添加0参考线
        h_zero_z = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('GYRO Z轴数据');
        xlabel('采样点');
        ylabel('角速度');
        ylim([-32768 32767]); % 统一纵坐标范围
        grid on;
        
        % ACCEL X轴数据
        subplot(3, 2, 4);
        h_accel_x = plot(NaN, NaN, 'm-', 'LineWidth', 1.5);
        hold on;
        % 添加0参考线
        h_zero_accel_x = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('ACCEL X轴数据');
        xlabel('采样点');
        ylabel('加速度');
        ylim([-32768 32767]); % 统一纵坐标范围
        grid on;
        
        % ACCEL Y轴数据
        subplot(3, 2, 5);
        h_accel_y = plot(NaN, NaN, 'c-', 'LineWidth', 1.5);
        hold on;
        % 添加0参考线
        h_zero_accel_y = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('ACCEL Y轴数据');
        xlabel('采样点');
        ylabel('加速度');
        ylim([-32768 32767]); % 统一纵坐标范围
        grid on;
        
        % ACCEL Z轴数据
        subplot(3, 2, 6);
        h_accel_z = plot(NaN, NaN, 'k-', 'LineWidth', 1.5);
        hold on;
        % 添加0参考线
        h_zero_accel_z = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
        title('ACCEL Z轴数据');
        xlabel('采样点');
        ylabel('加速度');
        ylim([-32768 32767]); % 统一纵坐标范围
        grid on;
        
        % 帧同步
        syncBytes = [170, 85]; % AA 55
        syncFound = false;
        
        % 显示标题
        fprintf('采样点\tGYRO_X(HEX)\tGYRO_X(INT16)\tGYRO_Y(HEX)\tGYRO_Y(INT16)\tGYRO_Z(HEX)\tGYRO_Z(INT16)\tACCEL_X(HEX)\tACCEL_X(INT16)\tACCEL_Y(HEX)\tACCEL_Y(INT16)\tACCEL_Z(HEX)\tACCEL_Z(INT16)\n');
        fprintf('------\t-----------\t-------------\t-----------\t-------------\t-----------\t-------------\t------------\t-------------\t------------\t-------------\t------------\t-------------\n');
        
        while ~syncFound
            byte1 = fread(s, 1, 'uint8');
            if ~isempty(byte1) && byte1 == syncBytes(1)
                byte2 = fread(s, 1, 'uint8');
                if ~isempty(byte2) && byte2 == syncBytes(2)
                    syncFound = true;
                    disp('帧同步成功');
                end
            end
        end
        
        % 数据采集循环
        for i = 1:maxPoints
            try
                % 读取完整数据包 (17字节)
                dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
                
                % 提取原始HEX值
                gyroX_hex = sprintf('%02X%02X', dataPacket(5), dataPacket(6));
                gyroY_hex = sprintf('%02X%02X', dataPacket(7), dataPacket(8));
                gyroZ_hex = sprintf('%02X%02X', dataPacket(9), dataPacket(10));
                accelX_hex = sprintf('%02X%02X', dataPacket(11), dataPacket(12));
                accelY_hex = sprintf('%02X%02X', dataPacket(13), dataPacket(14));
                accelZ_hex = sprintf('%02X%02X', dataPacket(15), dataPacket(16));
                
                % 转换为int16有符号整数
                gyroX_int = typecast(uint16(hex2dec(gyroX_hex)), 'int16');
                gyroY_int = typecast(uint16(hex2dec(gyroY_hex)), 'int16');
                gyroZ_int = typecast(uint16(hex2dec(gyroZ_hex)), 'int16');
                accelX_int = typecast(uint16(hex2dec(accelX_hex)), 'int16');
                accelY_int = typecast(uint16(hex2dec(accelY_hex)), 'int16');
                accelZ_int = typecast(uint16(hex2dec(accelZ_hex)), 'int16');
                
                % 存储数据
                gyroXData(i) = gyroX_int;
                gyroYData(i) = gyroY_int;
                gyroZData(i) = gyroZ_int;
                accelXData(i) = accelX_int;
                accelYData(i) = accelY_int;
                accelZData(i) = accelZ_int;
                timeData(i) = i;
                
                % 每10个采样点输出一次HEX和INT16值
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
                
                % 更新GYRO三轴图形
                subplot(3, 2, 1);
                set(h_gyro_x, 'XData', timeData(1:i), 'YData', double(gyroXData(1:i)));
                
                subplot(3, 2, 2);
                set(h_gyro_y, 'XData', timeData(1:i), 'YData', double(gyroYData(1:i)));
                
                subplot(3, 2, 3);
                set(h_gyro_z, 'XData', timeData(1:i), 'YData', double(gyroZData(1:i)));
                
                % 更新ACCEL三轴图形
                subplot(3, 2, 4);
                set(h_accel_x, 'XData', timeData(1:i), 'YData', double(accelXData(1:i)));
                
                subplot(3, 2, 5);
                set(h_accel_y, 'XData', timeData(1:i), 'YData', double(accelYData(1:i)));
                
                subplot(3, 2, 6);
                set(h_accel_z, 'XData', timeData(1:i), 'YData', double(accelZData(1:i)));
                
                % 动态调整X轴范围
                if i > 50
                    for j = 1:6
                        subplot(3, 2, j);
                        xlim([max(1, i-50), min(maxPoints, i+10)]);
                    end
                end
                
                drawnow limitrate; % 限制更新频率以提高性能
                
                % 检查下一帧的同步字节
                nextBytes = fread(s, 2, 'uint8');
                if length(nextBytes) == 2 && isequal(nextBytes', syncBytes)
                    % 同步正确，继续
                    continue;
                else
                    % 重新同步
                    syncFound = false;
                    while ~syncFound
                        byte1 = fread(s, 1, 'uint8');
                        if ~isempty(byte1) && byte1 == syncBytes(1)
                            byte2 = fread(s, 1, 'uint8');
                            if ~isempty(byte2) && byte2 == syncBytes(2)
                                syncFound = true;
                                disp('重新同步成功');
                            end
                        end
                    end
                end
                
            catch ME
                warning('数据读取错误: %s', ME.message);
                % 尝试重新同步
                syncFound = false;
                while ~syncFound
                    byte1 = fread(s, 1, 'uint8');
                    if ~isempty(byte1) && byte1 == syncBytes(1)
                        byte2 = fread(s, 1, 'uint8');
                        if ~isempty(byte2) && byte2 == syncBytes(2)
                            syncFound = true;
                            disp('错误后重新同步成功');
                        end
                    end
                end
            end
        end
    end
end

fclose(s); % 关闭串口连接
delete(s); % 删除串口对象