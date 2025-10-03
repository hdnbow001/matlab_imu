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
        % 初始化数据存储
        maxPoints = 1000;
        timeData = zeros(1, maxPoints);
        pitchData = zeros(1, maxPoints);
        byte4Data = zeros(1, maxPoints);
        diffByte4Data = zeros(1, maxPoints); % 存储差值
        
        % 创建图形窗口和子图
        figure('Position', [100, 100, 800, 900]); % 设置窗口大小
        
        % 第一个子图：俯仰角
        subplot(3, 1, 1);
        h1 = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
        title('pitch');
        xlabel('samples');
        ylabel('angle(°)');
        ylim([0 360]);
        grid on;
        
        % 第二个子图：第四个字节的值
        subplot(3, 1, 2);
        h2 = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
        title('psotion change counter');
        xlabel('samples');
        ylabel('value from POR');
        ylim([0 255]); % 假设字节值范围是0-255
        grid on;
        
        % 第三个子图：第四个字节的差值
        subplot(3, 1, 3);
        h3 = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
        title('trigger pulse');
        xlabel('smaple');
        ylabel('Delta');
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
                    disp('帧同步成功');
                end
            end
        end
        
        % 初始化前一值
        prevByte4Value = 0;
        
        % 数据采集循环
        for i = 1:maxPoints
            try
                % 读取完整数据包
                dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
                
                % 提取并处理数据
                % 1. 俯仰角 (第7-8字节)
                pitchValue = fun_hex2dec(dataPacket(7), dataPacket(8));
                
                % 2. 第四个字节的值
                byte4Value = double(dataPacket(4)); % 直接转换为数值
                
                % 3. 计算第四个字节的差值
                if i == 1
                    diffValue = 0; % 第一个点没有前一个点，差值为0
                else
                    diffValue = byte4Value - prevByte4Value;
                end
                
                % 更新前一值
                prevByte4Value = byte4Value;
                
                % 存储数据
                pitchData(i) = pitchValue;
                byte4Data(i) = byte4Value;
                diffByte4Data(i) = diffValue;
                timeData(i) = i;
                
                % 更新俯仰角图形
                subplot(3, 1, 1);
                set(h1, 'XData', timeData(1:i), 'YData', pitchData(1:i));
                
                % 更新第四个字节图形
                subplot(3, 1, 2);
                set(h2, 'XData', timeData(1:i), 'YData', byte4Data(1:i));
                
                % 更新第四个字节差值图形
                subplot(3, 1, 3);
                set(h3, 'XData', timeData(1:i), 'YData', diffByte4Data(1:i));
                
                % 动态调整X轴范围
                if i > 50
                    subplot(3, 1, 1);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    subplot(3, 1, 2);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    subplot(3, 1, 3);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    
                    % 自动调整差值图的Y轴范围
                    visibleDiffData = diffByte4Data(max(1, i-50):i);
                    if ~isempty(visibleDiffData)
                        yRange = max(abs(visibleDiffData)) * 1.1;
                        if yRange == 0
                            yRange = 1; % 避免yRange为0的情况
                        end
                        ylim([-yRange, yRange]);
                    end
                end
                
                % 添加图例和数值显示
                if i == 1
                    subplot(3, 1, 1);
                    legend('俯仰角', 'Location', 'best');
                    subplot(3, 1, 2);
                    legend('字节4值', 'Location', 'best');
                    subplot(3, 1, 3);
                    legend('变化率', 'Location', 'best');
                end
                
                % 实时显示当前值
                if mod(i, 10) == 0 || i == 1
                    fprintf('采样点 %d: 俯仰角=%.2f°, 字节4值=%d, 变化率=%d\n', ...
                        i, pitchValue, byte4Value, diffValue);
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