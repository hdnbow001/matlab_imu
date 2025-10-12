function [gesture_trajectory, gesture_velocity, is_gesture_active] = ...
         trackGesture_(accel_data, gyro_data, pitch_angles, roll_angles, dt, gesture_threshold)
    % 基于6轴IMU的手势轨迹跟踪
    % 输入：
    %   accel_data: 3×N矩阵 [accelX; accelY; accelZ]
    %   gyro_data: 3×N矩阵 [gyroX; gyroY; gyroZ]  
    %   pitch_angles: 1×N向量 - 俯仰角序列
    %   roll_angles: 1×N向量 - 滚转角序列
    %   dt: 采样时间间隔
    %   gesture_threshold: 手势开始/结束的加速度阈值
    % 输出：
    %   gesture_trajectory: 3×N矩阵 - 相对轨迹坐标
    %   gesture_velocity: 3×N矩阵 - 速度序列
    %   is_gesture_active: 1×N逻辑向量 - 手势活动状态
    
    [~, N] = size(accel_data);
    
    % 初始化输出变量
    gesture_trajectory = zeros(3, N);
    gesture_velocity = zeros(3, N);
    is_gesture_active = false(1, N);
    
    % 1. 确保所有输入数据都是double类型
    accel_data = double(accel_data);
    gyro_data = double(gyro_data);
    pitch_angles = double(pitch_angles);
    roll_angles = double(roll_angles);
    
    % 2. 手势活动检测
    gesture_active = detectGestureActivity(accel_data, gyro_data, gesture_threshold);
    is_gesture_active = gesture_active;
    
    if ~any(gesture_active)
        % 没有检测到手势活动
        return;
    end
    
    % 3. 找到手势开始和结束点
    gesture_start = find(gesture_active, 1, 'first');
    gesture_end = find(gesture_active, 1, 'last');
    
    % 只处理手势活动期间的数据
    active_indices = gesture_start:min(gesture_end, N);
    
    % 4.
    % 处理手势期间的传感器数据——按照active_indices数组的长度和每个对应位置上的数值进行双重循环判断：外层次数=矢量长度；内层次数=具体数值
    for i = active_indices
        % 4.1 构建旋转矩阵（基于当前姿态角）
        R = buildRotationMatrix(pitch_angles(i), roll_angles(i));
        
        % 4.2 将加速度转换到世界坐标系
        accel_sensor = [accel_data(1,i); accel_data(2,i); accel_data(3,i)];
        accel_world = R * accel_sensor;
        
        % 4.3 去除重力分量（假设世界坐标系Z轴向上）
        gravity = [0; 0; 9.81]; % 重力向量
        linear_accel = accel_world - gravity;
        
        % 4.4 存储线性加速度用于积分
        if i == gesture_start
            % 手势开始时初始化
            gesture_velocity(:,i) = [0; 0; 0];
            gesture_trajectory(:,i) = [0; 0; 0];
        else
            % 积分得到速度和位置
            gesture_velocity(:,i) = gesture_velocity(:,i-1) + linear_accel * dt;
            gesture_trajectory(:,i) = gesture_trajectory(:,i-1) + gesture_velocity(:,i) * dt;
        end
    end
    
    % 5. 轨迹后处理
    gesture_trajectory = postProcessTrajectory(gesture_trajectory, active_indices);
end