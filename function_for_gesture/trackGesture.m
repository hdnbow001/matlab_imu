function [gesture_trajectory, gesture_velocity, is_gesture_active] = ...
         trackGesture(accel_data, gyro_data, pitch_angles, roll_angles, dt, gesture_threshold)
    % 基于6轴IMU的手势轨迹跟踪 - 修复版本
    % 修复问题：1) 旋转运动误判 2) 初始位移漂移
    
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
    
    % === 新增：初始校准 ===
    % 使用手势开始前的一段静止数据校准初始重力方向
    if gesture_start > 10
        calib_start = max(1, gesture_start - 20);
        calib_end = gesture_start - 1;
        
        % 计算平均加速度作为重力方向估计
        calib_accel = mean(accel_data(:, calib_start:calib_end), 2);
        gravity_magnitude = norm(calib_accel);
        
        % 使用校准后的重力向量
        calibrated_gravity = calib_accel / gravity_magnitude * 9.81;
    else
        calibrated_gravity = [0; 0; 9.81]; % 备用值
    end
    
    % 4. 处理手势期间的传感器数据
    for i = active_indices
        % 4.1 构建旋转矩阵（基于当前姿态角）
        R = buildRotationMatrix(pitch_angles(i), roll_angles(i));
        
        % 4.2 将加速度转换到世界坐标系
        accel_sensor = [accel_data(1,i); accel_data(2,i); accel_data(3,i)];
        accel_world = R * accel_sensor;
        
        % 4.3 去除重力分量（使用校准后的重力向量）
        linear_accel = accel_world - calibrated_gravity;
        
        % === 新增：旋转运动检测与抑制 ===
        % 计算角速度模值检测旋转运动
        gyro_magnitude = norm([gyro_data(1,i), gyro_data(2,i), gyro_data(3,i)]);
        
        % 如果角速度很大，可能是纯旋转运动，抑制线性加速度
        %rotation_threshold = 50; % deg/s 阈值
        rotation_threshold = 10; % deg/s 阈值
        if gyro_magnitude > rotation_threshold
            % 降低旋转期间的线性加速度增益
            %rotation_suppression = 0.3;
            rotation_suppression = 0.1;
            linear_accel = linear_accel * rotation_suppression;
        end
        
        % 4.4 存储线性加速度用于积分
        if i == gesture_start
            % 手势开始时初始化
            gesture_velocity(:,i) = [0; 0; 0];
            gesture_trajectory(:,i) = [0; 0; 0];
        else
            % === 改进的积分方法 ===
            % 积分得到速度
            gesture_velocity(:,i) = gesture_velocity(:,i-1) + linear_accel * dt;
            
            % === 新增：零速度更新(ZUPT) ===
            % 当运动能量很低时，假设速度为零
            motion_energy = norm(linear_accel);
            %if motion_energy < 0.5 % 阈值可调整
            if motion_energy < 0.1 % 阈值可调整
                gesture_velocity(:,i) = gesture_velocity(:,i) * 0.8; % 速度衰减
            end
            
            % 积分得到位置
            gesture_trajectory(:,i) = gesture_trajectory(:,i-1) + gesture_velocity(:,i) * dt;
        end
    end
    
    % 5. 轨迹后处理 - 增强版本
    gesture_trajectory = enhancedPostProcessTrajectory(gesture_trajectory, active_indices, gesture_velocity);
end