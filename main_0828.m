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
    disp('UART open success！');
catch
    error('UART open fail！');
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
        % 初始化数据存储
        maxPoints = 1000;
        timeData = zeros(1, maxPoints);
        pitchData = zeros(1, maxPoints);
        byte4Data = zeros(1, maxPoints);
        
        % 创建图形窗口和子图
        figure;
        
        % 第一个子图：俯仰角
        subplot(2, 1, 1);
        h1 = plot(NaN, NaN, 'r-');
        title('pitch');
        xlabel('samples');
        ylabel('pitch angle (°)');
        ylim([0 360]);
        grid on;
        
        % 第二个子图：第四个字节的值
        subplot(2, 1, 2);
        h2 = plot(NaN, NaN, 'b-');
        title('position change counter');
        xlabel('samples');
        ylabel('counter');
        ylim([0 255]); % 假设字节值范围是0-255
        grid on;
        
        % 帧同步
        syncBytes = [170, 85];
        syncFound = false;
        
        while ~syncFound
            byte1 = fread(s, 1, 'uint8');
            if ~isempty(byte1) && byte1 == syncBytes(1)
                byte2 = fread(s, 1, 'uint8');
                if ~isempty(byte2) && byte2 == syncBytes(2)
                    syncFound = true;
                    disp('Fram synced');
                end
            end
        end
        
        % 数据采集循环
        for i = 1:maxPoints
            % 读取完整数据包
            dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
            
            % 提取并处理数据
            % 1. 俯仰角 (第7-8字节)
            pitchValue = fun_hex2dec(dataPacket(7), dataPacket(8));
            
            % 2. 第四个字节的值
            byte4Value = double(dataPacket(4)); % 直接转换为数值
            
            % 存储数据
            pitchData(i) = pitchValue;
            byte4Data(i) = byte4Value;
            timeData(i) = i;
            
            % 更新俯仰角图形
            set(h1, 'XData', timeData(1:i), 'YData', pitchData(1:i));
            
            % 更新第四个字节图形
            set(h2, 'XData', timeData(1:i), 'YData', byte4Data(1:i));
            
            % 动态调整X轴范围
            if i > 50
                subplot(2, 1, 1);
                xlim([max(1, i-50), i+10]);
                subplot(2, 1, 2);
                xlim([max(1, i-50), i+10]);
            end
            
            drawnow;
            
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
        end
    end
end

fclose(s); % 关闭串口连接
delete(s); % 删除串口对象