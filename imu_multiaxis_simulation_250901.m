function imu_multiaxis_simulation()
    % IMU_MULTIAXIS_SIMULATION 多轴IMU加速度数据仿真与位移解算
    % 模拟在X、Y、Z三个轴上同时有运动的情况
    
    close all; clear all; clc;
    
    %% 参数设置
    Fs = 100;           % 采样率 100 Hz
    dt = 1/Fs;          % 采样时间间隔
    T = 10;             % 总时间 10秒
    t = (0:dt:T-dt)';   % 时间向量
    N = length(t);
    
    fprintf('多轴IMU仿真与位移解算...\n');
    fprintf('采样率: %.0f Hz\n', Fs);
    fprintf('持续时间: %.1f 秒\n', T);
    fprintf('数据点数: %d\n', N);
    
    %% 1. 模拟三维运动轨迹
    fprintf('\n1. 模拟三维运动轨迹...\n');
    
    % 定义三维运动参数 - 螺旋上升运动
    omega_x = 0.5 * pi;  % X轴角频率 (rad/s)
    omega_y = 0.8 * pi;  % Y轴角频率 (rad/s)
    omega_z = 0.3 * pi;  % Z轴角频率 (rad/s)
    
    A_x = 2.0;          % X轴振幅 (m/s?)
    A_y = 1.5;          % Y轴振幅 (m/s?)
    A_z = 1.2;          % Z轴振幅 (m/s?)
    
    % 生成理论上的运动加速度 (无噪声无偏差)
    accel_motion = zeros(N, 3);
    accel_motion(:, 1) = -A_x * omega_x^2 * sin(omega_x * t);  % X轴加速度
    accel_motion(:, 2) = -A_y * omega_y^2 * cos(omega_y * t);  % Y轴加速度
    accel_motion(:, 3) = -A_z * omega_z^2 * sin(omega_z * t);  % Z轴加速度
    
    % 通过积分计算理论速度和位移 (用于验证)
    velocity_theoretical = cumtrapz(dt, accel_motion);
    position_theoretical = cumtrapz(dt, velocity_theoretical);
    
    %% 2. 模拟IMU测量数据
    fprintf('2. 模拟IMU测量数据...\n');
    
    % IMU测量值 = 运动加速度 + 重力加速度 + 噪声 + 偏差
    g = 9.81;
    gravity = repmat([0, 0, g], N, 1);  % NED坐标系：重力为 [0, 0, 9.81]
    
    % 添加噪声和偏差
    bias = [0.05, -0.03, 0.08];         % 三轴不同的偏差
    noise_level = 0.1;                  % 噪声水平
    
    accel_bias = repmat(bias, N, 1);
    accel_noise = noise_level * randn(N, 3);
    
    % 合成"测量到"的加速度计数据 (本体坐标系)
    accel_body = accel_motion + gravity + accel_bias + accel_noise;
    
    % 模拟陀螺仪数据 - 小幅随机旋转
    gyro_noise_level = 0.02; % 陀螺仪噪声水平 (rad/s)
    gyro_body = gyro_noise_level * randn(N, 3);
    
    % 添加一些小幅周期性旋转
    gyro_body(:, 1) = gyro_body(:, 1) + 0.1 * sin(0.5 * pi * t);
    gyro_body(:, 2) = gyro_body(:, 2) + 0.08 * cos(0.3 * pi * t);
    
    fprintf('IMU数据模拟完成\n');
    
    %% 3. 姿态解算（使用互补滤波）
    fprintf('3. 进行姿态解算（互补滤波）...\n');
    
    % 初始化姿态角（滚转、俯仰、偏航）
    pitch = zeros(N, 1);
    roll = zeros(N, 1);
    yaw = zeros(N, 1);
    
    % 互补滤波系数
    alpha = 0.98;
    
    for i = 2:N
        % 用陀螺仪积分得到姿态
        gyro_rates = gyro_body(i, :);
        pitch_gyro = pitch(i-1) + gyro_rates(1) * dt;
        roll_gyro  = roll(i-1)  + gyro_rates(2) * dt;
        yaw_gyro   = yaw(i-1)   + gyro_rates(3) * dt;

        % 用加速度计计算滚转和俯仰角
        acc = accel_body(i, :);
        roll_acc = atan2(acc(2), sign(acc(3)) * sqrt(acc(1)^2 + acc(3)^2));
        pitch_acc = atan2(-acc(1), sqrt(acc(2)^2 + acc(3)^2));

        % 互补滤波融合
        roll(i) = alpha * roll_gyro + (1 - alpha) * roll_acc;
        pitch(i) = alpha * pitch_gyro + (1 - alpha) * pitch_acc;
        yaw(i) = yaw_gyro; % 加速度计无法观测偏航，完全依赖陀螺仪
    end
    
    fprintf('姿态解算完成\n');
    
    %% 4. 坐标变换与重力补偿
    fprintf('4. 进行坐标变换与重力补偿...\n');
    
    accel_global = zeros(N, 3); % 全局坐标系下的加速度
    
    for i = 1:N
        % 根据当前姿态角构建旋转矩阵 R（从本体坐标系到全局坐标系）
        % 使用 Z-Y-X (偏航-俯仰-滚转) 旋转顺序
        cr = cos(roll(i));  sr = sin(roll(i));
        cp = cos(pitch(i)); sp = sin(pitch(i));
        cy = cos(yaw(i));   sy = sin(yaw(i));

        R = [cy*cp,   cy*sp*sr - sy*cr,   cy*sp*cr + sy*sr;
             sy*cp,   sy*sp*sr + cy*cr,   sy*sp*cr - cy*sr;
             -sp,     cp*sr,              cp*cr];

        % 将本体坐标系下的加速度旋转到全局坐标系
        accel_global_i = R * accel_body(i, :)';

        % 重力补偿：减去全局坐标系中的重力向量 [0; 0; g]
        accel_global(i, :) = accel_global_i' - [0, 0, g];
    end
    
    fprintf('坐标变换与重力补偿完成\n');
    
    %% 5. 滤波与第一次积分（得到速度）
    fprintf('5. 进行加速度滤波与速度积分...\n');
    
    % 高通滤波去除加速度中的低频偏差
    fc = 0.1; % 截止频率 0.1 Hz
    [b, a] = butter(2, fc/(Fs/2), 'high');
    accel_global_filt = filtfilt(b, a, accel_global);
    
    % 对滤波后的全局加速度进行积分得到速度
    velocity = cumtrapz(dt, accel_global_filt);
    
    % 简单的ZUPT - 假设开始和结束时静止
    zupt_start = 1:50;      % 前0.5秒
    zupt_end = (N-49):N;    % 最后0.5秒
    velocity(zupt_start, :) = 0;
    velocity(zupt_end, :) = 0;
    
    fprintf('速度计算完成\n');
    
    %% 6. 第二次积分（得到位移）
    fprintf('6. 进行位移积分...\n');
    
    % 对速度进行积分得到位移
    position = cumtrapz(dt, velocity);
    
    fprintf('位移计算完成\n');
    
    %% 7. 可视化结果
    fprintf('7. 生成结果可视化...\n');
    
    % 创建一个大图窗
    figure('Position', [100, 100, 1400, 1000], 'Name', '多轴IMU位移解算结果');
    
    % 绘制原始加速度数据
    subplot(3, 4, 1);
    plot(t, accel_body(:,1), 'r', t, accel_body(:,2), 'g', t, accel_body(:,3), 'b');
    title('原始加速度数据 (本体坐标系)');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制姿态角
    subplot(3, 4, 2);
    plot(t, rad2deg(roll), 'r', t, rad2deg(pitch), 'g', t, rad2deg(yaw), 'b');
    title('估算的姿态角');
    xlabel('时间 (s)');
    ylabel('角度 (度)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;
    
    % 绘制全局坐标系下的加速度
    subplot(3, 4, 3);
    plot(t, accel_global(:,1), 'r', t, accel_global(:,2), 'g', t, accel_global(:,3), 'b');
    title('全局坐标系加速度 (重力补偿前)');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制滤波后的加速度
    subplot(3, 4, 4);
    plot(t, accel_global_filt(:,1), 'r', t, accel_global_filt(:,2), 'g', t, accel_global_filt(:,3), 'b');
    title('滤波后的线性加速度 (全局坐标系)');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制速度
    subplot(3, 4, 5);
    plot(t, velocity(:,1), 'r', t, velocity(:,2), 'g', t, velocity(:,3), 'b');
    title('速度');
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制位移
    subplot(3, 4, 6);
    plot(t, position(:,1), 'r', t, position(:,2), 'g', t, position(:,3), 'b');
    title('估算位移');
    xlabel('时间 (s)');
    ylabel('位移 (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制理论位移
    subplot(3, 4, 7);
    plot(t, position_theoretical(:,1), 'r', t, position_theoretical(:,2), 'g', t, position_theoretical(:,3), 'b');
    title('理论位移');
    xlabel('时间 (s)');
    ylabel('位移 (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制位移误差
    subplot(3, 4, 8);
    error_x = position(:,1) - position_theoretical(:,1);
    error_y = position(:,2) - position_theoretical(:,2);
    error_z = position(:,3) - position_theoretical(:,3);
    plot(t, error_x, 'r', t, error_y, 'g', t, error_z, 'b');
    title('位移误差');
    xlabel('时间 (s)');
    ylabel('误差 (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制3D理论轨迹
    subplot(3, 4, 9);
    plot3(position_theoretical(:,1), position_theoretical(:,2), position_theoretical(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(position_theoretical(1,1), position_theoretical(1,2), position_theoretical(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(position_theoretical(end,1), position_theoretical(end,2), position_theoretical(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    hold off;
    title('理论3D轨迹');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('轨迹', '起点', '终点', 'Location', 'best');
    grid on;
    axis equal;
    
    % 绘制3D估算轨迹
    subplot(3, 4, 10);
    plot3(position(:,1), position(:,2), position(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(position(1,1), position(1,2), position(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(position(end,1), position(end,2), position(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    hold off;
    title('估算3D轨迹');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('轨迹', '起点', '终点', 'Location', 'best');
    grid on;
    axis equal;
    
    % 绘制3D轨迹对比
    subplot(3, 4, [11, 12]);
    plot3(position_theoretical(:,1), position_theoretical(:,2), position_theoretical(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(position(:,1), position(:,2), position(:,3), 'r--', 'LineWidth', 1.5);
    hold off;
    title('3D轨迹对比');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('理论轨迹', '估算轨迹', 'Location', 'best');
    grid on;
    axis equal;
    
    % 保存图像
    saveas(gcf, 'imu_multiaxis_results.png');
    fprintf('结果已保存为 imu_multiaxis_results.png\n');
    
    %% 8. 显示关键结果
    fprintf('\n=== 解算结果摘要 ===\n');
    
    % 计算最终位移和误差
    final_position = position(end, :);
    final_position_theoretical = position_theoretical(end, :);
    error = final_position - final_position_theoretical;
    error_percentage = abs(error) ./ abs(final_position_theoretical) * 100;
    
    fprintf('理论最终位移: X=%.3fm, Y=%.3fm, Z=%.3fm\n', final_position_theoretical);
    fprintf('估算最终位移: X=%.3fm, Y=%.3fm, Z=%.3fm\n', final_position);
    fprintf('绝对误差:     X=%.3fm, Y=%.3fm, Z=%.3fm\n', error);
    fprintf('相对误差:     X=%.1f%%, Y=%.1f%%, Z=%.1f%%\n', error_percentage);
    
    % 计算RMS误差
    rms_error = sqrt(mean((position - position_theoretical).^2));
    fprintf('RMS误差:      X=%.3fm, Y=%.3fm, Z=%.3fm\n', rms_error);
    
    fprintf('\n处理完成！\n');
    
    % 提示用户关于多轴运动的特点
    fprintf('\n多轴运动特点：\n');
    fprintf('1. 加速度、速度和位移在三个轴上同时存在\n');
    fprintf('2. 各轴之间的耦合效应更加明显\n');
    fprintf('3. 姿态变化对各轴数据的影响更加复杂\n');
    fprintf('4. 误差会在三个轴上同时累积和传播\n');
end