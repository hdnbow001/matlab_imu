function main_()
    % 参数设置
    fs = 10;   % 采样率 10Hz
    T = 100;   % 时宽 (100秒)
    n = round(T*fs);  % 采样点个数 (1000个)
    s = serial('COM7'); % 创建串口对象
    set(s, 'BaudRate', 115200); % 设置波特率
    
    % 传感器参数
    accel_range = 2; % ±2G
    gyro_range = 250; % ±250dps
    
    % 自适应互补滤波参数
    adaptive_alpha_min = 0.85;  % 最小滤波系数
    adaptive_alpha_max = 0.98;  % 最大滤波系数
    convergence_time = 0.8;     % 收敛时间0.8秒
    convergence_samples = round(convergence_time * fs); % 收敛所需采样点数
    
    % 运动状态检测参数
    motion_threshold = 0.15;     % 运动检测阈值(G)
    stationary_threshold = 0.05; % 静止检测阈值(G)
    
    % 初始化自适应权重
    accel_weight = 1.0;         % 初始阶段更信任加速度计
    gyro_weight = 0.0;
    
    flag = false;
    try
        fopen(s);
        flag = true;
        disp('串口打开成功');
    catch
        error('串口打开失败');
    end

    if flag
        % 初始化数据存储
        maxPoints = n; % 采样点数设为1000
        
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
        
        % 姿态角度数据
        pitchAngles = zeros(1, maxPoints);
        rollAngles = zeros(1, maxPoints);
        
        % 存储原始加速度计姿态角
        accPitchAngles = zeros(1, maxPoints);
        accRollAngles = zeros(1, maxPoints);
        
        % 加速度计矢量幅值（用于运动检测）
        accel_magnitude = zeros(1, maxPoints);
        
        % 动态零偏估计参数
        alpha = 0.001; % 滤波系数
        dynamic_bias_gyroX = 0;
        dynamic_bias_gyroY = 0;
        dynamic_bias_gyroZ = 0;
        
        % 静态零偏校准参数
        calibrationSamples = 50; % 校准采样点数
        bias_gyroX = 0;
        bias_gyroY = 0;
        bias_gyroZ = 0;
        
        % 融合角度初始化
        fusedPitch = 0; % 融合后的俯仰角
        fusedRoll = 0;  % 融合后的滚转角
        
        % 创建图形窗口 - 只保留姿态显示
        fig = figure('Position', [50, 50, 800, 600], 'Name', 'IMU传感器姿态显示');
        
        % 姿态显示子图
        attitude_subplot = subplot(1, 1, 1);
        % 初始化姿态显示
        [attitudeQuiver, attitudeText, attitudeSphere, attitudeAxes] = initAttitudeDisplay(attitude_subplot);
        
        % 帧同步
        syncBytes = [170, 85]; % AA 55
        syncFound = false;
        
        % 时间参数
        dt = 1/fs; % 采样间隔
        
        % 显示标题
        fprintf('采样点\tGYRO_X(HEX)\tGYRO_X(dps)\tGYRO_Y(HEX)\tGYRO_Y(dps)\tGYRO_Z(HEX)\tGYRO_Z(dps)\tACCEL_X(HEX)\tACCEL_X(G)\tACCEL_Y(HEX)\tACCEL_Y(G)\tACCEL_Z(HEX)\tACCEL_Z(G)\n');
        fprintf('------\t-----------\t-----------\t-----------\t-----------\t-----------\t-----------\t------------\t----------\t------------\t----------\t------------\t----------\n');
        
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
                
                % 计算加速度计矢量幅值（用于运动检测）
                accel_magnitude(i) = sqrt(double(accelXData(i))^2 + double(accelYData(i))^2 + double(accelZData(i))^2);
                
                % 计算加速度计姿态角度
                [accPitch, accRoll] = calculateAttitude(...
                    double(accelXData(i)), double(accelYData(i)), double(accelZData(i)), accel_range);
                
                % 确保加速度计角度在0-360°范围内
                accPitch = mod(accPitch, 360);
                accRoll = mod(accRoll, 360);
                
                % 存储原始加速度计姿态角
                accPitchAngles(i) = accPitch;
                accRollAngles(i) = accRoll;
                
                % 使用自适应互补滤波融合姿态
                if i == 1
                    % 初始化融合角度
                    fusedPitch = accPitch;
                    fusedRoll = accRoll;
                else
                    % 将陀螺仪数据转换为度/秒
                    gyroX_dps = (double(gyroXCompensated(i)) / 32768) * gyro_range;
                    gyroY_dps = (double(gyroYCompensated(i)) / 32768) * gyro_range;
                    
                    % 调用自适应传感器融合函数
                    [fusedPitch, fusedRoll, accel_weight] = adaptiveSensorFusion(...
                        accPitch, accRoll, gyroX_dps, gyroY_dps, ...
                        fusedPitch, fusedRoll, dt, i, convergence_samples, ...
                        accel_magnitude, motion_threshold);
                end
                
                % 确保融合后的角度在0-360°范围内
                pitchAngles(i) = mod(fusedPitch, 360);
                rollAngles(i) = mod(fusedRoll, 360);
                
                % 每10个采样点输出一次数据
                if mod(i, 10) == 0 || i == 1
                    % 转换为物理单位
                    gyroX_dps = (double(gyroX_int) / 32768) * gyro_range;
                    gyroY_dps = (double(gyroY_int) / 32768) * gyro_range;
                    gyroZ_dps = (double(gyroZ_int) / 32768) * gyro_range;
                    accelX_g = (double(accelX_int) / 32768) * accel_range;
                    accelY_g = (double(accelY_int) / 32768) * accel_range;
                    accelZ_g = (double(accelZ_int) / 32768) * accel_range;
                    
                    fprintf('%d\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\t\t%s\t%.2f\n', ...
                        i, ...
                        gyroX_hex, gyroX_dps, ...
                        gyroY_hex, gyroY_dps, ...
                        gyroZ_hex, gyroZ_dps, ...
                        accelX_hex, accelX_g, ...
                        accelY_hex, accelY_g, ...
                        accelZ_hex, accelZ_g);
                    
                    % 显示当前动态零偏估计值
                    if i > 1
                        fprintf('动态零偏估计: X=%.2f, Y=%.2f, Z=%.2f\n', ...
                            dynamic_bias_gyroX, dynamic_bias_gyroY, dynamic_bias_gyroZ);
                    end
                end
                
                % 更新姿态显示（每10个点更新一次以提高性能）
                if mod(i, 10) == 0 || i == 1
                    % 显示互补滤波融合后的角度
                    displayPitch = pitchAngles(i);
                    displayRoll = rollAngles(i);
                    displayYaw = 0; % yaw角度固定为0

                    % 调试输出
                    fprintf('姿态角 - 加速度计: pitch=%.1f°, roll=%.1f° | 融合后: pitch=%.1f°, roll=%.1f°\n', ...
                        accPitchAngles(i), accRollAngles(i), displayPitch, displayRoll);

                    updateAttitudeDisplay(attitudeQuiver, attitudeText, attitudeSphere, ...
                        double(accelXData(i)), double(accelYData(i)), double(accelZData(i)), ...
                        gyroXCompensated(i), gyroYCompensated(i), gyroZCompensated(i), attitudeAxes, accel_range, gyro_range, displayPitch, displayRoll, displayYaw);
                end
                
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
                % 重新获取图形句柄
                fig = gcf;
                attitude_subplot = subplot(1, 1, 1);
            end
        end
        
        % 数据采集完成后，显示完成信息
        fprintf('\n数据采集完成\n');
        
        % 显示最终姿态结果
        fprintf('\n=== 最终姿态结果 ===\n');
        fprintf('最终俯仰角(Pitch): %.1f°\n', pitchAngles(end));
        fprintf('最终滚转角(Roll): %.1f°\n', rollAngles(end));
        fprintf('==================\n');
    end
    
    fclose(s); % 关闭串口连接
    delete(s); % 删除串口对象
end