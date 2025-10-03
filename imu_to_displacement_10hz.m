function imu_to_displacement_10hz()
    % IMU_TO_DISPLACEMENT_10HZ 从IMU加速度数据计算位移 (10Hz采样率)
    % 此函数模拟IMU数据并演示如何通过两次积分计算位移
    
    close all; clear all; clc;
    
    %% 参数设置 (10Hz采样率)
    Fs = 10;            % 采样率 10 Hz
    dt = 1/Fs;          % 采样时间间隔
    T = 5;              % 总时间 5秒
    t = (0:dt:T-dt)';   % 时间向量
    N = length(t);
    
    fprintf('开始IMU位移解算 (10Hz采样率)...\n');
    fprintf('采样率: %.0f Hz\n', Fs);
    fprintf('持续时间: %.1f 秒\n', T);
    fprintf('数据点数: %d\n', N);
    
    %% 1. 模拟IMU数据
    fprintf('\n1. 模拟IMU数据...\n');
    
    % 模拟一段运动：初始静止，然后有加速度，最后再静止
    accel_motion = zeros(N, 3);
    % 10Hz下，1-2秒对应第11到第20个点，3-4秒对应第31到第40个点
    accel_motion(11:20, 1) = 2;       % X轴方向在1-2秒内有 2 m/s? 的加速度
    accel_motion(31:40, 1) = -1.5;    % X轴方向在3-4秒内有 -1.5 m/s? 的减速度
    
    % IMU测量值 = 运动加速度 + 重力加速度 + 噪声 + 偏差
    % 使用NED坐标系：重力为 [0, 0, 9.81]
    g = 9.81;
    gravity = repmat([0, 0, g], N, 1); 
    
    % 添加噪声和偏差 (由于采样率降低，相对噪声水平可能需要调整)
    bias = 0.1;                         % 加速度计偏差
    noise_level = 0.08;                 % 噪声水平 (略高于高采样率情况)
    accel_bias = bias * ones(N, 3);
    accel_noise = noise_level * randn(N, 3);
    
    % 合成"测量到"的加速度计数据 (本体坐标系)
    accel_body = accel_motion + gravity + accel_bias + accel_noise;
    
    % 模拟陀螺仪数据（本例中姿态不变，故角速度为0，仅含噪声）
    gyro_body = noise_level * randn(N, 3);
    
    fprintf('IMU数据模拟完成\n');
    
    %% 2. 姿态解算（使用互补滤波）
    fprintf('2. 进行姿态解算（互补滤波）...\n');
    
    % 初始化姿态角（滚转、俯仰、偏航）
    pitch = zeros(N, 1);
    roll = zeros(N, 1);
    yaw = zeros(N, 1);
    
    % 互补滤波系数 (可能需要调整以适应低采样率)
    alpha = 0.95; % 稍微降低陀螺仪的权重
    
    for i = 2:N
        % 用陀螺仪积分得到姿态（简易积分）
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
    
    %% 3. 坐标变换与重力补偿
    fprintf('3. 进行坐标变换与重力补偿...\n');
    
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
    
    %% 4. 滤波与第一次积分（得到速度）
    fprintf('4. 进行加速度滤波与速度积分...\n');
    
    % 高通滤波去除加速度中的低频偏差
    % 由于采样率降低，截止频率需要相应调整
    fc = 0.05; % 截止频率 0.05 Hz (相对于10Hz采样率)
    [b, a] = butter(2, fc/(Fs/2), 'high');
    accel_global_filt = filtfilt(b, a, accel_global);
    
    % 对滤波后的全局加速度进行积分得到速度
    velocity = cumtrapz(dt, accel_global_filt);
    
    % 假设运动开始和结束都是静止的，可以进行速度清零（一种简单的ZUPT）
    % 10Hz下，前0.5秒对应前5个点
    velocity(1:5, :) = 0;    % 假设前0.5秒静止
    velocity(end-4:end, :) = 0; % 假设最后0.5秒静止
    
    fprintf('速度计算完成\n');
    
    %% 5. 第二次积分（得到位移）
    fprintf('5. 进行位移积分...\n');
    
    % 对速度进行积分得到位移
    position = cumtrapz(dt, velocity);
    
    fprintf('位移计算完成\n');
    
    %% 6. 可视化结果
    fprintf('6. 生成结果可视化...\n');
    
    % 创建一个大图窗
    figure('Position', [100, 100, 1200, 800], 'Name', 'IMU位移解算结果 (10Hz采样率)');
    
    % 绘制原始加速度数据
    subplot(3, 3, 1);
    plot(t, accel_body(:,1), 'r', t, accel_body(:,2), 'g', t, accel_body(:,3), 'b');
    title('原始加速度数据 (本体坐标系)');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制姿态角
    subplot(3, 3, 2);
    plot(t, rad2deg(roll), 'r', t, rad2deg(pitch), 'g', t, rad2deg(yaw), 'b');
    title('估算的姿态角');
    xlabel('时间 (s)');
    ylabel('角度 (度)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;
    
    % 绘制全局坐标系下的加速度
    subplot(3, 3, 3);
    plot(t, accel_global(:,1), 'r', t, accel_global(:,2), 'g', t, accel_global(:,3), 'b');
    title('全局坐标系加速度 (重力补偿前)');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制滤波后的加速度
    subplot(3, 3, 4);
    plot(t, accel_global_filt(:,1), 'r', t, accel_global_filt(:,2), 'g', t, accel_global_filt(:,3), 'b');
    title('滤波后的线性加速度 (全局坐标系)');
    xlabel('时间 (s)');
    ylabel('加速度 (m/s^2)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制速度
    subplot(3, 3, 5);
    plot(t, velocity(:,1), 'r', t, velocity(:,2), 'g', t, velocity(:,3), 'b');
    title('速度');
    xlabel('时间 (s)');
    ylabel('速度 (m/s)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制位移
    subplot(3, 3, 6);
    plot(t, position(:,1), 'r', t, position(:,2), 'g', t, position(:,3), 'b');
    title('位移');
    xlabel('时间 (s)');
    ylabel('位移 (m)');
    legend('X', 'Y', 'Z');
    grid on;
    
    % 绘制3D轨迹
    subplot(3, 3, [7, 8, 9]);
    plot3(position(:,1), position(:,2), position(:,3), 'b-', 'LineWidth', 1.5);
    hold on;
    plot3(position(1,1), position(1,2), position(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot3(position(end,1), position(end,2), position(end,3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    hold off;
    title('3D轨迹 (10Hz采样率)');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    legend('轨迹', '起点', '终点', 'Location', 'best');
    grid on;
    axis equal;
    
    % 保存图像
    saveas(gcf, 'imu_displacement_results_10hz.png');
    fprintf('结果已保存为 imu_displacement_results_10hz.png\n');
    
    %% 7. 显示关键结果
    fprintf('\n=== 解算结果摘要 (10Hz采样率) ===\n');
    fprintf('最终位移: X=%.3fm, Y=%.3fm, Z=%.3fm\n', ...
            position(end,1), position(end,2), position(end,3));
    fprintf('最大位移: X=%.3fm, Y=%.3fm, Z=%.3fm\n', ...
            max(abs(position(:,1))), max(abs(position(:,2))), max(abs(position(:,3))));
    
    % 计算与理论值的误差（因为我们知道模拟的运动）
    % 对于10Hz采样率，我们需要重新计算理论值
    theoretical_velocity_x = cumtrapz(dt, accel_motion(:,1));
    theoretical_x_displacement = cumtrapz(dt, theoretical_velocity_x);
    theoretical_x_displacement = theoretical_x_displacement(end);
    
    actual_x_displacement = position(end,1);
    error_percentage = abs(actual_x_displacement - theoretical_x_displacement) / abs(theoretical_x_displacement) * 100;
    
    fprintf('X方向理论位移: %.3fm\n', theoretical_x_displacement);
    fprintf('X方向实际位移: %.3fm\n', actual_x_displacement);
    fprintf('X方向误差: %.1f%%\n', error_percentage);
    
    fprintf('\n处理完成！\n');
    
    % 提示用户关于低采样率的影响
    fprintf('\n注意：10Hz采样率相对于100Hz采样率，可能会导致以下影响：\n');
    fprintf('1. 积分精度降低，特别是对于快速变化的运动\n');
    fprintf('2. 滤波器性能下降，可能需要调整滤波器参数\n');
    fprintf('3. 姿态解算的响应速度变慢\n');
    fprintf('4. 运动细节可能丢失，特别是高频成分\n');
    fprintf('在实际应用中，应根据运动特性和精度要求选择合适的采样率。\n');
end