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
        maxPoints = 2000;
        timeData = zeros(1, maxPoints);
        pitchData = zeros(1, maxPoints);
        byte4Data = zeros(1, maxPoints);
        diffByte4Data = zeros(1, maxPoints); % 存储差值
        
        % 创建图形窗口和子图
        figure('Position', [100, 100, 1000, 900]); % 设置窗口大小
        
        % 第一个子图：圆周面上的俯仰角矢量表示
        subplot(3, 1, 1);
        % 绘制参考圆
        theta = linspace(0, 2*pi, 100);
        plot(cos(theta), sin(theta), 'k--');
        hold on;
        axis equal;
        grid on;
        xlim([-1.2, 1.2]);
        ylim([-1.2, 1.2]);
        title('俯仰角矢量表示');
        xlabel('X轴');
        ylabel('Y轴');
        
        % 初始化俯仰角矢量
        pitchVector = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        currentAngleText = text(-1.1, 1.0, '角度: 0°', 'FontSize', 10);
        
        % 添加角度标记
        text(1.05, 0, '0°', 'FontSize', 8);
        text(0, 1.05, '90°', 'FontSize', 8);
        text(-1.05, 0, '180°', 'FontSize', 8);
        text(0, -1.05, '270°', 'FontSize', 8);
        
        % 第二个子图：第四个字节的值
        subplot(3, 1, 2);
        h2 = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
        title('第四个字节实时数据');
        xlabel('采样点');
        ylabel('数值');
        ylim([0 255]); % 假设字节值范围是0-255
        grid on;
        
        % 第三个子图：第四个字节的差值
        subplot(3, 1, 3);
        h3 = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
        title('第四个字节变化率');
        xlabel('采样点');
        ylabel('差值');
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
                
                % 更新俯仰角矢量表示
                subplot(3, 1, 1);
                % 计算矢量的终点坐标
                angle_rad = deg2rad(pitchValue);
                x_end = cos(angle_rad);
                y_end = sin(angle_rad);
                
                % 更新矢量
                set(pitchVector, 'UData', x_end, 'VData', y_end);
                
                % 更新角度文本
                set(currentAngleText, 'String', sprintf('角度: %.1f°', pitchValue));
                
                % 更新第四个字节图形
                subplot(3, 1, 2);
                set(h2, 'XData', timeData(1:i), 'YData', byte4Data(1:i));
                
                % 更新第四个字节差值图形
                subplot(3, 1, 3);
                set(h3, 'XData', timeData(1:i), 'YData', diffByte4Data(1:i));
                
                % 动态调整X轴范围
                if i > 50
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