    % 基于现有主循环扩展手势跟踪功能
    % 现有数据流:串口数据 → 传感器解析 → 姿态计算 → 姿态显示
    % 新增手势跟踪数据流:传感器数据 → 手势检测 → 轨迹计算 → 轨迹显示/识别
%%
function main()
    % 参数设置
    fs = 20;   % 采样率 20Hz
    T = 50;   % 时宽 (50秒)
    n = round(T*fs);  % 采样点个数 (1000个)
    %s = serial('COM7'); % uih开发机创建串口对象
    s = serial('COM3'); % hd开发机创建串口对象
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
    
    % === 新增：手势跟踪参数 ===
    gesture_threshold = 0.3;     % 手势检测阈值(G)
    gesture_buffer_size = 50;    % 手势数据缓冲区大小（2.5秒数据）
    
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
        
        % === 新增：手势跟踪数据存储 ===
        gesture_trajectory_x = zeros(1, maxPoints);
        gesture_trajectory_y = zeros(1, maxPoints);
        gesture_trajectory_z = zeros(1, maxPoints);
        is_gesture_active = false(1, maxPoints);
        gesture_velocity_x = zeros(1, maxPoints);
        gesture_velocity_y = zeros(1, maxPoints);
        gesture_velocity_z = zeros(1, maxPoints);
        
        % 动态零偏估计参数
        alpha = 0.001; % 滤波系数
        dynamic_bias_gyroX = 0;
        dynamic_bias_gyroY = 0;
        dynamic_bias_gyroZ = 0;
        
        % 静态零偏校准参数
        calibrationSamples = 50; % 校准采样点数
        
        % 融合角度初始化
        fusedPitch = 0; % 融合后的俯仰角
        fusedRoll = 0;  % 融合后的滚转角
        
        % 创建图形窗口 - 姿态显示和手势跟踪
        fig = figure('Position', [50, 50, 1200, 600], 'Name', 'IMU传感器姿态与手势跟踪');
        
        % 姿态显示子图
        attitude_subplot = subplot(1, 2, 1);
        % 初始化姿态显示
        [attitudeQuiver, attitudeText, attitudeSphere, attitudeAxes] = initAttitudeDisplay(attitude_subplot);
        
        % === 新增：手势轨迹显示子图 ===
        gesture_subplot = subplot(1, 2, 2);
        title(gesture_subplot, '实时手势轨迹');
        xlabel(gesture_subplot, 'X (m)'); 
        ylabel(gesture_subplot, 'Y (m)'); 
        zlabel(gesture_subplot, 'Z (m)');
        grid(gesture_subplot, 'on'); 
        hold(gesture_subplot, 'on');
        axis(gesture_subplot, 'equal');
        view(gesture_subplot, 3);
        
        % 初始化手势轨迹图形对象
        gesture_trajectory_plot = plot3(gesture_subplot, 0, 0, 0, 'b-', 'LineWidth', 2);
        gesture_points_plot = scatter3(gesture_subplot, 0, 0, 0, 20, 'filled', 'r');
        gesture_start_plot = plot3(gesture_subplot, 0, 0, 0, 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
        gesture_end_plot = plot3(gesture_subplot, 0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        % 帧同步
        syncBytes = [170, 85]; % AA 55
        syncFound = false;
        
        % 时间参数
        dt = 1/fs; % 采样间隔
        
        % 显示标题
        fprintf('采样点\tGYRO_X(HEX)\tGYRO_X(dps)\tGYRO_Y(HEX)\tGYRO_Y(dps)\tGYRO_Z(HEX)\tGYRO_Z(dps)\tACCEL_X(HEX)\tACCEL_X(G)\tACCEL_Y(HEX)\tACCEL_Y(G)\tACCEL_Z(HEX)\tACCEL_Z(G)\n');
        fprintf('------\t-----------\t-----------\t-----------\t-----------\t-----------\t-----------\t------------\t----------\t------------\t----------\t------------\t----------\n');
        
        % === 使用函数封装零偏校准 ===
        [bias_gyroX, bias_gyroY, bias_gyroZ] = gyroBiasCalibration(s, calibrationSamples, syncBytes);
        
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
                
                % === 修复：手势跟踪处理（确保数据类型正确）===
                if i > 1
                    % 确定数据缓冲区范围
                    buffer_start = max(1, i - gesture_buffer_size + 1);
                    buffer_indices = buffer_start:i;
                    
                    % 提取缓冲区数据并确保为double类型
                    accel_buffer = double([accelXData(buffer_indices); 
                                           accelYData(buffer_indices);
                                           accelZData(buffer_indices)]);
                    gyro_buffer = double([gyroXCompensated(buffer_indices);
                                          gyroYCompensated(buffer_indices); 
                                          gyroZCompensated(buffer_indices)]);
                    pitch_buffer = double(pitchAngles(buffer_indices));
                    roll_buffer = double(rollAngles(buffer_indices));
                    
                    % 执行手势跟踪
                    [trajectory, velocity, active] = trackGesture(...
                        accel_buffer, gyro_buffer, pitch_buffer, roll_buffer, dt, gesture_threshold);
                    
                    % 存储结果（只取最新点）
                    if ~isempty(trajectory) && ~isempty(velocity) && ~isempty(active)
                        gesture_trajectory_x(i) = trajectory(1, end);
                        gesture_trajectory_y(i) = trajectory(2, end);
                        gesture_trajectory_z(i) = trajectory(3, end);
                        gesture_velocity_x(i) = velocity(1, end);
                        gesture_velocity_y(i) = velocity(2, end);
                        gesture_velocity_z(i) = velocity(3, end);
                        is_gesture_active(i) = active(end);
                    end
                end
                
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
                
                % 更新姿态显示（每5个点更新一次以提高性能）
                if mod(i, 5) == 0 || i == 1
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
                
                % === 新增：更新手势轨迹显示（每5个点更新一次）===
                if mod(i, 5) == 0 || i == 1
                    updateGestureDisplay(gesture_trajectory_plot, gesture_points_plot, ...
                                       gesture_start_plot, gesture_end_plot, ...
                                       gesture_trajectory_x, gesture_trajectory_y, gesture_trajectory_z, ...
                                       is_gesture_active, i, gesture_subplot);
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
                attitude_subplot = subplot(1, 2, 1);
                gesture_subplot = subplot(1, 2, 2);
            end
        end
        
        % 数据采集完成后，显示完成信息
        fprintf('\n数据采集完成\n');
        
        % 显示最终姿态结果
        fprintf('\n=== 最终姿态结果 ===\n');
        fprintf('最终俯仰角(Pitch): %.1f°\n', pitchAngles(end));
        fprintf('最终滚转角(Roll): %.1f°\n', rollAngles(end));
        
        % === 新增：显示手势统计信息 ===
        active_gesture_points = sum(is_gesture_active);
        if active_gesture_points > 0
            fprintf('\n=== 手势跟踪统计 ===\n');
            fprintf('检测到手势活动的采样点数: %d/%d\n', active_gesture_points, maxPoints);
            fprintf('手势活动比例: %.1f%%\n', (active_gesture_points/maxPoints)*100);
            
            % 计算最大轨迹范围
            active_indices = find(is_gesture_active);
            if length(active_indices) >= 2
                max_range_x = max(gesture_trajectory_x(active_indices)) - min(gesture_trajectory_x(active_indices));
                max_range_y = max(gesture_trajectory_y(active_indices)) - min(gesture_trajectory_y(active_indices));
                max_range_z = max(gesture_trajectory_z(active_indices)) - min(gesture_trajectory_z(active_indices));
                fprintf('手势轨迹范围: X=%.3fm, Y=%.3fm, Z=%.3fm\n', max_range_x, max_range_y, max_range_z);
            end
        end
        fprintf('==================\n');
    end
    
    fclose(s); % 关闭串口连接
    delete(s); % 删除串口对象
end
