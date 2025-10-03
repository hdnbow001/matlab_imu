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
        pitchData1 = zeros(1, maxPoints);  % 第7-8字节的角度
        pitchData2 = zeros(1, maxPoints);  % 第9-10字节的角度
        byte4Data = zeros(1, maxPoints);
        diffByte4Data = zeros(1, maxPoints); % 第4字节的差值
        byte16Data = zeros(1, maxPoints);    % 第16字节的数值
        diffByte16Data = zeros(1, maxPoints); % 第16字节的差值
        
        % 创建图形窗口和子图 - 2x3布局
        figure('Position', [50, 50, 1400, 800], 'Name', '串口数据实时监控'); % 设置窗口大小
        
        % 第一行第一列：第一个角度的圆周矢量表示
        subplot(2, 3, 1);
        % 绘制参考圆
        theta = linspace(0, 2*pi, 100);
        plot(cos(theta), sin(theta), 'k--');
        hold on;
        axis equal;
        grid on;
        xlim([-1.2, 1.2]);
        ylim([-1.2, 1.2]);
        title('角度1 (7-8字节) 矢量表示');
        xlabel('X轴');
        ylabel('Y轴');
        
        % 初始化第一个俯仰角矢量（红色）
        pitchVector1 = quiver(0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % 添加角度文本
        currentAngleText1 = text(-1.1, 1.0, '角度1: 0°', 'FontSize', 10, 'Color', 'r');
        
        % 添加角度标记
        text(1.05, 0, '0°', 'FontSize', 8);
        text(0, 1.05, '90°', 'FontSize', 8);
        text(-1.05, 0, '180°', 'FontSize', 8);
        text(0, -1.05, '270°', 'FontSize', 8);
        
        % 第一行第二列：第四个字节的值
        subplot(2, 3, 2);
        h4 = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
        title('第四个字节实时数据');
        xlabel('采样点');
        ylabel('数值');
        ylim([0 255]); % 假设字节值范围是0-255
        grid on;
        
        % 第一行第三列：第四个字节的差值
        subplot(2, 3, 3);
        h4_diff = plot(NaN, NaN, 'm-', 'LineWidth', 1.5);
        title('第四个字节变化率');
        xlabel('采样点');
        ylabel('差值');
        grid on;
        
        % 第二行第一列：第二个角度的圆周矢量表示
        subplot(2, 3, 4);
        % 绘制参考圆
        plot(cos(theta), sin(theta), 'k--');
        hold on;
        axis equal;
        grid on;
        xlim([-1.2, 1.2]);
        ylim([-1.2, 1.2]);
        title('角度2 (9-10字节) 矢量表示');
        xlabel('X轴');
        ylabel('Y轴');
        
        % 初始化第二个俯仰角矢量（蓝色）
        pitchVector2 = quiver(0, 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
        
        % 添加角度文本
        currentAngleText2 = text(-1.1, 1.0, '角度2: 0°', 'FontSize', 10, 'Color', 'b');
        
        % 添加角度标记
        text(1.05, 0, '0°', 'FontSize', 8);
        text(0, 1.05, '90°', 'FontSize', 8);
        text(-1.05, 0, '180°', 'FontSize', 8);
        text(0, -1.05, '270°', 'FontSize', 8);
        
        % 第二行第二列：第十六字节的值
        subplot(2, 3, 5);
        h16 = plot(NaN, NaN, 'c-', 'LineWidth', 1.5);
        title('第十六字节实时数据');
        xlabel('采样点');
        ylabel('数值');
        ylim([0 255]); % 假设字节值范围是0-255
        grid on;
        
        % 第二行第三列：第十六字节的差值
        subplot(2, 3, 6);
        h16_diff = plot(NaN, NaN, 'k-', 'LineWidth', 1.5);
        title('第十六字节变化率');
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
        prevByte16Value = 0;
        
        % 数据采集循环
        for i = 1:maxPoints
            try
                % 读取完整数据包
                dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
                
                % 提取并处理数据
                % 1. 第一个俯仰角 (第7-8字节)
                pitchValue1 = fun_hex2dec(dataPacket(7), dataPacket(8));
                
                % 2. 第二个俯仰角 (第9-10字节)
                pitchValue2 = fun_hex2dec(dataPacket(9), dataPacket(10));
                
                % 3. 第四个字节的值
                byte4Value = double(dataPacket(4)); % 直接转换为数值
                
                % 4. 计算第四个字节的差值
                if i == 1
                    diff4Value = 0; % 第一个点没有前一个点，差值为0
                else
                    diff4Value = byte4Value - prevByte4Value;
                end
                
                % 5. 第十六字节的值
                byte16Value = double(dataPacket(16)); % 直接转换为数值
                
                % 6. 计算第十六字节的差值
                if i == 1
                    diff16Value = 0; % 第一个点没有前一个点，差值为0
                else
                    diff16Value = byte16Value - prevByte16Value;
                end
                
                % 更新前一值
                prevByte4Value = byte4Value;
                prevByte16Value = byte16Value;
                
                % 存储数据
                pitchData1(i) = pitchValue1;
                pitchData2(i) = pitchValue2;
                byte4Data(i) = byte4Value;
                diffByte4Data(i) = diff4Value;
                byte16Data(i) = byte16Value;
                diffByte16Data(i) = diff16Value;
                timeData(i) = i;
                
                % 更新第一个角度矢量表示
                subplot(2, 3, 1);
                angle_rad1 = deg2rad(pitchValue1);
                x_end1 = cos(angle_rad1);
                y_end1 = sin(angle_rad1);
                set(pitchVector1, 'UData', x_end1, 'VData', y_end1);
                set(currentAngleText1, 'String', sprintf('角度1: %.1f°', pitchValue1));
                
                % 更新第四个字节图形
                subplot(2, 3, 2);
                set(h4, 'XData', timeData(1:i), 'YData', byte4Data(1:i));
                
                % 更新第四个字节差值图形
                subplot(2, 3, 3);
                set(h4_diff, 'XData', timeData(1:i), 'YData', diffByte4Data(1:i));
                
                % 更新第二个角度矢量表示
                subplot(2, 3, 4);
                angle_rad2 = deg2rad(pitchValue2);
                x_end2 = cos(angle_rad2);
                y_end2 = sin(angle_rad2);
                set(pitchVector2, 'UData', x_end2, 'VData', y_end2);
                set(currentAngleText2, 'String', sprintf('角度2: %.1f°', pitchValue2));
                
                % 更新第十六字节图形
                subplot(2, 3, 5);
                set(h16, 'XData', timeData(1:i), 'YData', byte16Data(1:i));
                
                % 更新第十六字节差值图形
                subplot(2, 3, 6);
                set(h16_diff, 'XData', timeData(1:i), 'YData', diffByte16Data(1:i));
                
                % 动态调整X轴范围
                if i > 50
                    subplot(2, 3, 2);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    subplot(2, 3, 3);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    subplot(2, 3, 5);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    subplot(2, 3, 6);
                    xlim([max(1, i-50), min(maxPoints, i+10)]);
                    
                    % 自动调整差值图的Y轴范围
                    visibleDiff4Data = diffByte4Data(max(1, i-50):i);
                    if ~isempty(visibleDiff4Data)
                        yRange4 = max(abs(visibleDiff4Data)) * 1.1;
                        if yRange4 == 0
                            yRange4 = 1; % 避免yRange为0的情况
                        end
                        subplot(2, 3, 3);
                        ylim([-yRange4, yRange4]);
                    end
                    
                    visibleDiff16Data = diffByte16Data(max(1, i-50):i);
                    if ~isempty(visibleDiff16Data)
                        yRange16 = max(abs(visibleDiff16Data)) * 1.1;
                        if yRange16 == 0
                            yRange16 = 1; % 避免yRange为0的情况
                        end
                        subplot(2, 3, 6);
                        ylim([-yRange16, yRange16]);
                    end
                end
                
                % 添加图例
                if i == 1
                    subplot(2, 3, 2);
                    legend('字节4值', 'Location', 'best');
                    subplot(2, 3, 3);
                    legend('字节4变化率', 'Location', 'best');
                    subplot(2, 3, 5);
                    legend('字节16值', 'Location', 'best');
                    subplot(2, 3, 6);
                    legend('字节16变化率', 'Location', 'best');
                end
                
                % 实时显示当前值
                if mod(i, 10) == 0 || i == 1
                    fprintf('采样点 %d: 角度1=%.2f°, 角度2=%.2f°, 字节4值=%d, 字节4变化率=%d, 字节16值=%d, 字节16变化率=%d\n', ...
                        i, pitchValue1, pitchValue2, byte4Value, diff4Value, byte16Value, diff16Value);
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
        
        % 数据采集完成后，显示统计信息
        fprintf('\n数据采集完成\n');
        fprintf('角度1统计: 最小值=%.2f°, 最大值=%.2f°, 平均值=%.2f°\n', ...
            min(pitchData1), max(pitchData1), mean(pitchData1));
        fprintf('角度2统计: 最小值=%.2f°, 最大值=%.2f°, 平均值=%.2f°\n', ...
            min(pitchData2), max(pitchData2), mean(pitchData2));
        fprintf('字节4统计: 最小值=%d, 最大值=%d, 平均值=%.2f\n', ...
            min(byte4Data), max(byte4Data), mean(byte4Data));
        fprintf('字节16统计: 最小值=%d, 最大值=%d, 平均值=%.2f\n', ...
            min(byte16Data), max(byte16Data), mean(byte16Data));
        
        % 可选：保存数据到文件
%         save('sensor_data.mat', 'timeData', 'pitchData1', 'pitchData2', ...
%             'byte4Data', 'diffByte4Data', 'byte16Data', 'diffByte16Data');
%         disp('数据已保存到 sensor_data.mat');
    end
end

fclose(s); % 关闭串口连接
delete(s); % 删除串口对象