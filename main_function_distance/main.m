function main()
    % 参数设置
    fs = 10;   % 采样率 10Hz
    T = 100;   % 时宽 (100秒)
    n = round(T*fs);  % 采样点个数 (1000个)
    %%s = serial('COM7'); % 创建串口对象-在UIH开发机上
    s = serial('COM3'); % 创建串口对象-在E5开发机上
    set(s, 'BaudRate', 115200); % 设置波特率
    
    % 传感器参数
    accel_range = 2; % ±2G
    gyro_range = 250; % ±250dps
    window_size = 5 * fs; % 5秒窗口大小 (50个采样点)
    
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
        
        % 位移数据
        displacementX = zeros(1, maxPoints);
        displacementY = zeros(1, maxPoints);
        displacementZ = zeros(1, maxPoints);
        
        % 姿态角度数据
        pitchAngles = zeros(1, maxPoints);
        rollAngles = zeros(1, maxPoints);
        yawAngles = zeros(1, maxPoints); % 新增偏航角存储
        
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
        
        % 互补滤波参数
        compAlpha = 0.98; % 互补滤波系数
        fusedPitch = 0; % 融合后的俯仰角
        fusedRoll = 0;  % 融合后的滚转角
        fusedYaw = 0;   % 融合后的偏航角
        
        % 零速度检测参数
        stationaryThreshold = [0.05, 2]; % [加速度方差阈值, 角速度均值阈值]
        
        % 创建图形窗口 - 1x2布局
        fig = figure('Position', [50, 50, 1400, 600], 'Name', 'IMU传感器姿态和位移显示');
        
        % 姿态显示子图
        attitude_subplot = subplot(1, 2, 1);
        % 初始化姿态显示
        [attitudeQuiver, attitudeText, attitudeSphere, attitudeAxes] = initAttitudeDisplay(attitude_subplot);
        
        % 位移显示子图
        displacement_subplot = subplot(1, 2, 2);
        % 初始化位移显示
        [h_displacement, h_displacement_text, displacementAxes] = initDisplacementDisplay(displacement_subplot);
        
        % 创建调试图表
        debugFig = figure('Position', [100, 100, 1200, 800], 'Name', '传感器数据调试');
        subplot(3,1,1);
        h_accel_x = plot(NaN, NaN, 'r-');
        hold on;
        h_accel_y = plot(NaN, NaN, 'g-');
        h_accel_z = plot(NaN, NaN, 'b-');
        title('线性加速度 (m/s?)');
        legend('X', 'Y', 'Z');
        grid on;
        
        subplot(3,1,2);
        h_velocity_x = plot(NaN, NaN, 'r-');
        hold on;
        h_velocity_y = plot(NaN, NaN, 'g-');
        h_velocity_z = plot(NaN, NaN, 'b-');
        title('速度 (m/s)');
        legend('X', 'Y', 'Z');
        grid on;
        
        subplot(3,1,3);
        h_displacement_x = plot(NaN, NaN, 'r-');
        hold on;
        h_displacement_y = plot(NaN, NaN, 'g-');
        h_displacement_z = plot(NaN, NaN, 'b-');
        title('位移 (m)');
        legend('X', 'Y', 'Z');
        grid on;
        
        % 帧同步
        syncBytes = [170, 85]; % AA 55
        syncFound = false;
        
        % 位移计算参数
        dt = 1/fs; % 采样间隔
        current_window_start = 1; % 当前窗口起始索引
        window_count = 1; % 窗口计数器
        
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
                
                % 计算加速度计姿态角度
                [accPitch, accRoll] = calculateAttitude(...
                    double(accelXData(i)), double(accelYData(i)), double(accelZData(i)), accel_range);
                
                % 互补滤波融合姿态
                if i == 1
                    % 初始化融合角度
                    fusedPitch = accPitch;
                    fusedRoll = accRoll;
                    fusedYaw = 0;
                else
                    % 将陀螺仪数据转换为度/秒
                    gyroX_dps = (gyroXCompensated(i) / 32768) * gyro_range;
                    gyroY_dps = (gyroYCompensated(i) / 32768) * gyro_range;
                    gyroZ_dps = (gyroZCompensated(i) / 32768) * gyro_range;
                    
                    % 使用陀螺仪积分更新角度
                    fusedPitch = fusedPitch + gyroX_dps * dt;
                    fusedRoll = fusedRoll + gyroY_dps * dt;
                    fusedYaw = fusedYaw + gyroZ_dps * dt;
                    
                    % 使用互补滤波融合加速度计的角度
                    fusedPitch = compAlpha * fusedPitch + (1 - compAlpha) * accPitch;
                    fusedRoll = compAlpha * fusedRoll + (1 - compAlpha) * accRoll;
                end
                
                % 存储融合后的角度
                pitchAngles(i) = fusedPitch;
                rollAngles(i) = fusedRoll;
                yawAngles(i) = fusedYaw;
                
                % 检查是否需要开始新的5秒窗口
                if mod(i-1, window_size) == 0
                    current_window_start = i;
                    window_count = window_count + 1;
                    % 重置位移数据
                    displacementX(i) = 0;
                    displacementY(i) = 0;
                    displacementZ(i) = 0;
                end
                
                % 计算位移（当前5秒窗口内）
                if i > current_window_start
                    window_indices = current_window_start:i;
                    [displacementX(i), displacementY(i), displacementZ(i)] = ...
                        calculateDisplacement(...
                        double(accelXData(window_indices)), ...
                        double(accelYData(window_indices)), ...
                        double(accelZData(window_indices)), ...
                        double(gyroXCompensated(window_indices)), ...
                        double(gyroYCompensated(window_indices)), ...
                        double(gyroZCompensated(window_indices)), ...
                        pitchAngles(window_indices), ...
                        rollAngles(window_indices), ...
                        dt, accel_range, gyro_range, window_count);
                else
                    % 窗口的第一个点，位移为0
                    displacementX(i) = 0;
                    displacementY(i) = 0;
                    displacementZ(i) = 0;
                end
                
                % 每10个采样点输出一次数据
                if mod(i, 10) == 0 || i == 1
                    % 转换为物理单位
                    gyroX_dps = (gyroX_int / 32768) * gyro_range;
                    gyroY_dps = (gyroY_int / 32768) * gyro_range;
                    gyroZ_dps = (gyroZ_int / 32768) * gyro_range;
                    accelX_g = (accelX_int / 32768) * accel_range;
                    accelY_g = (accelY_int / 32768) * accel_range;
                    accelZ_g = (accelZ_int / 32768) * accel_range;
                    
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
                
                % 更新位移显示
                updateDisplacementDisplay(h_displacement, h_displacement_text, ...
                    displacementX, displacementY, displacementZ, i, displacementAxes, current_window_start, window_count);
                
                % 更新姿态显示（每10个点更新一次以提高性能）
                if mod(i, 10) == 0 || i == 1
                    updateAttitudeDisplay(attitudeQuiver, attitudeText, attitudeSphere, ...
                        double(accelXData(i)), double(accelYData(i)), double(accelZData(i)), ...
                        gyroXCompensated(i), gyroYCompensated(i), gyroZCompensated(i), attitudeAxes, accel_range, gyro_range, pitchAngles(i), rollAngles(i), yawAngles(i));
                end
                
                % 更新调试图表
                if i > 1
                    % 计算线性加速度、速度和位移用于调试
                    window_indices = max(1, i-50):i;
                    [linAccelX, linAccelY, linAccelZ, velX, velY, velZ] = ...
                        calculateDebugData(...
                        double(accelXData(window_indices)), ...
                        double(accelYData(window_indices)), ...
                        double(accelZData(window_indices)), ...
                        pitchAngles(window_indices), ...
                        rollAngles(window_indices), ...
                        dt, accel_range);
                    
                    % 分别更新每条曲线
                    set(h_accel_x, 'XData', window_indices, 'YData', linAccelX);
                    set(h_accel_y, 'XData', window_indices, 'YData', linAccelY);
                    set(h_accel_z, 'XData', window_indices, 'YData', linAccelZ);
                    
                    set(h_velocity_x, 'XData', window_indices, 'YData', velX);
                    set(h_velocity_y, 'XData', window_indices, 'YData', velY);
                    set(h_velocity_z, 'XData', window_indices, 'YData', velZ);
                    
                    set(h_displacement_x, 'XData', window_indices, 'YData', displacementX(window_indices));
                    set(h_displacement_y, 'XData', window_indices, 'YData', displacementY(window_indices));
                    set(h_displacement_z, 'XData', window_indices, 'YData', displacementZ(window_indices));
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
                %fprintf('数据读取错误: %s\n', ME.message);
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
                attitude_subplot = subplot(1, 2, 1);
                displacement_subplot = subplot(1, 2, 2);
            end
        end
        
        % 数据采集完成后，显示完成信息
        fprintf('\n数据采集完成\n');
    end
    
    fclose(s); % 关闭串口连接
    delete(s); % 删除串口对象
end