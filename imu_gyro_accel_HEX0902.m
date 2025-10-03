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
        hexdata = 'aa 55 05 00 00 00 00 00 00 00 00 00 00 00 00 CA 0D'; % 启动
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
        data = fread(s, 17);
    elseif dataSwitch == 2
        hexdata = 'aa 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0D'; % 关闭
        asciiValues = hex2dec(split(hexdata));
        fwrite(s, asciiValues);
    else
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
        maxPoints = 500; % 采集50个点用于显示
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
                
                % 显示HEX值和INT16值
                fprintf('%d\t%s\t%d\t\t%s\t%d\t\t%s\t%d\t\t%s\t%d\t\t%s\t%d\t\t%s\t%d\n', ...
                    i, ...
                    gyroX_hex, gyroX_int, ...
                    gyroY_hex, gyroY_int, ...
                    gyroZ_hex, gyroZ_int, ...
                    accelX_hex, accelX_int, ...
                    accelY_hex, accelY_int, ...
                    accelZ_hex, accelZ_int);
                
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