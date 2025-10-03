% 主程序
function main()
    % 参数设置
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
            maxPoints = 1000; % 采样点数设为1000
            
            % GYRO三轴数据（原始和补偿后）
            gyroXData = zeros(1, maxPoints, 'int16');
            gyroYData = zeros(1, maxPoints, 'int16');
            gyroZData = zeros(1, maxPoints, 'int16');
            gyroXCompensated = zeros(1, maxPoints);
            gyroYCompensated = zeros(1, maxPoints);
            gyroZCompensated = zeros(1, maxPoints);
            
            % ACCEL三轴数据
            accelXData = zeros(1, maxPoints, 'int16');
            accelYData = zeros(1, maxPoints, 'int16');
            accelZData = zeros(1, maxPoints, 'int16');
            
            % 动态零偏估计参数
            alpha = 0.001; % 滤波系数（越小越平滑但响应越慢）
            dynamic_bias_gyroX = 0;
            dynamic_bias_gyroY = 0;
            dynamic_bias_gyroZ = 0;
            
            % 静态零偏校准参数
            calibrationSamples = 200; % 校准采样点数
            bias_gyroX = 0;
            bias_gyroY = 0;
            bias_gyroZ = 0;
            
            % 创建图形窗口和子图 - 3x3布局
            figure('Position', [50, 50, 1400, 900], 'Name', 'IMU传感器数据实时监控（带动态零偏补偿和姿态显示）');
            
            % GYRO X轴数据（补偿后）
            subplot(3, 3, 1);
            h_gyro_x = plot(NaN, NaN, 'r-', 'LineWidth', 1.5);
            hold on;
            % 添加0参考线
            h_zero_x = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
            title('GYRO X轴数据（补偿后）');
            xlabel('采样点');
            ylabel('角速度');
            ylim([-32768 32767]); % 统一纵坐标范围
            grid on;
            
            % GYRO Y轴数据（补偿后）
            subplot(3, 3, 2);
            h_gyro_y = plot(NaN, NaN, 'g-', 'LineWidth', 1.5);
            hold on;
            % 添加0参考线
            h_zero_y = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
            title('GYRO Y轴数据（补偿后）');
            xlabel('采样点');
            ylabel('角速度');
            ylim([-32768 32767]); % 统一纵坐标范围
            grid on;
            
            % GYRO Z轴数据（补偿后）
            subplot(3, 3, 3);
            h_gyro_z = plot(NaN, NaN, 'b-', 'LineWidth', 1.5);
            hold on;
            % 添加0参考线
            h_zero_z = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
            title('GYRO Z轴数据（补偿后）');
            xlabel('采样点');
            ylabel('角速度');
            ylim([-32768 32767]); % 统一纵坐标范围
            grid on;
            
            % ACCEL X轴数据
            subplot(3, 3, 4);
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
            subplot(3, 3, 5);
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
            subplot(3, 3, 6);
            h_accel_z = plot(NaN, NaN, 'k-', 'LineWidth', 1.5);
            hold on;
            % 添加0参考线
            h_zero_accel_z = plot([0 maxPoints], [0 0], 'k--', 'LineWidth', 1);
            title('ACCEL Z轴数据');
            xlabel('采样点');
            ylabel('加速度');
            ylim([-32768 32767]); % 统一纵坐标范围
            grid on;
            
            % 姿态显示子图 (3x3布局的第7-9位置合并为一个大图)
            attitude_subplot = subplot(3, 3, [7, 8, 9]);
            % 初始化姿态显示
            [attitudeQuiver, attitudeText] = initAttitudeDisplay(attitude_subplot);
            
            % 帧同步
            syncBytes = [170, 85]; % AA 55
            syncFound = false;
            
            % 显示标题
            fprintf('采样点\tGYRO_X(HEX)\tGYRO_X(INT16)\tGYRO_Y(HEX)\tGYRO_Y(INT16)\tGYRO_Z(HEX)\tGYRO_Z(INT16)\tACCEL_X(HEX)\tACCEL_X(INT16)\tACCEL_Y(HEX)\tACCEL_Y(INT16)\tACCEL_Z(HEX)\tACCEL_Z(INT16)\n');
            fprintf('------\t-----------\t-------------\t-----------\t-------------\t-----------\t-------------\t------------\t-------------\t------------\t-------------\t------------\t-------------\n');
            
            % 零偏校准阶段
            disp('开始零偏校准，请保持传感器静止...');
            gyroX_calib = zeros(1, calibrationSamples);
            gyroY_calib = zeros(1, calibrationSamples);
            gyroZ_calib = zeros(1, calibrationSamples);
            
            for calib_i = 1:calibrationSamples
                % 读取完整数据包 (17字节)
                dataPacket = [syncBytes'; fread(s, 15, 'uint8')];
                
                % 提取原始HEX值
                gyroX_hex = sprintf('%02X%02X', dataPacket(5), dataPacket(6));
                gyroY_hex = sprintf('%02X%02X', dataPacket(7), dataPacket(8));
                gyroZ_hex = sprintf('%02X%02X', dataPacket(9), dataPacket(10));
                
                % 转换为int16有符号整数
                gyroX_calib(calib_i) = double(typecast(uint16(hex2dec(gyroX_hex)), 'int16'));
                gyroY_calib(calib_i) = double(typecast(uint16(hex2dec(gyroY_hex)), 'int16'));
                gyroZ_calib(calib_i) = double(typecast(uint16(hex2dec(gyroZ_hex)), 'int16'));
                
                % 检查下一帧的同步字节
                nextBytes = fread(s, 2, 'uint8');
                if ~(length(nextBytes) == 2 && isequal(nextBytes', syncBytes))
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
            
            % 计算零偏值（平均值）
            bias_gyroX = mean(gyroX_calib);
            bias_gyroY = mean(gyroY_calib);
            bias_gyroZ = mean(gyroZ_calib);
            
            fprintf('零偏校准完成：\n');
            fprintf('GYRO X零偏: %.2f\n', bias_gyroX);
            fprintf('GYRO Y零偏: %.2f\n', bias_gyroY);
            fprintf('GYRO Z零偏: %.2f\n', bias_gyroZ);
            fprintf('开始数据采集...\n');
            
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
                    
                    % 使用零偏补偿函数
                    [gyroX_final, gyroY_final, gyroZ_final, dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ] = ...
                        gyroBiasCompensation(...
                        gyroX_int, gyroY_int, gyroZ_int, ...
                        bias_gyroX, bias_gyroY, bias_gyroZ, ...
                        dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ, ...
                        alpha);
                    
                    % 存储数据
                    gyroXData(i) = gyroX_int;
                    gyroYData(i) = gyroY_int;
                    gyroZData(i) = gyroZ_int;
                    gyroXCompensated(i) = gyroX_final;
                    gyroYCompensated(i) = gyroY_final;
                    gyroZCompensated(i) = gyroZ_final;
                    accelXData(i) = accelX_int;
                    accelYData(i) = accelY_int;
                    accelZData(i) = accelZ_int;
                    
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
                        
                        % 显示当前动态零偏估计值
                        if i > 1
                            fprintf('动态零偏估计: X=%.2f, Y=%.2f, Z=%.2f\n', ...
                                dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ);
                        end
                    end
                    
                    % 更新GYRO三轴图形（使用补偿后的数据）
                    subplot(3, 3, 1);
                    set(h_gyro_x, 'XData', 1:i, 'YData', gyroXCompensated(1:i));
                    
                    subplot(3, 3, 2);
                    set(h_gyro_y, 'XData', 1:i, 'YData', gyroYCompensated(1:i));
                    
                    subplot(3, 3, 3);
                    set(h_gyro_z, 'XData', 1:i, 'YData', gyroZCompensated(1:i));
                    
                    % 更新ACCEL三轴图形
                    subplot(3, 3, 4);
                    set(h_accel_x, 'XData', 1:i, 'YData', double(accelXData(1:i)));
                    
                    subplot(3, 3, 5);
                    set(h_accel_y, 'XData', 1:i, 'YData', double(accelYData(1:i)));
                    
                    subplot(3, 3, 6);
                    set(h_accel_z, 'XData', 1:i, 'YData', double(accelZData(1:i)));
                    
                    % 更新姿态显示（每10个点更新一次以提高性能）
                    if mod(i, 10) == 0 || i == 1
                        updateAttitudeDisplay(attitudeQuiver, attitudeText, ...
                            double(accelXData(i)), double(accelYData(i)), double(accelZData(i)), ...
                            gyroXCompensated(i), gyroYCompensated(i), gyroZCompensated(i));
                    end
                    
                    % 动态调整X轴范围
                    if i > 50
                        for j = 1:6
                            subplot(3, 3, j);
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
            
            % 数据采集完成后，显示完成信息
            fprintf('\n数据采集完成\n');
        end
    end
    
    fclose(s); % 关闭串口连接
    delete(s); % 删除串口对象
end