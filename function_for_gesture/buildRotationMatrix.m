function R = buildRotationMatrix(pitch, roll)
    % 基于俯仰和滚转角构建旋转矩阵
    % 注意：假设偏航角为0（6轴IMU的限制）
    
    % 确保输入为double类型
    pitch = double(pitch);
    roll = double(roll);
    
    % 将角度转换为弧度
    pitch_rad = deg2rad(pitch);
    roll_rad = deg2rad(roll);
    
    % 俯仰旋转矩阵（绕Y轴）
    R_pitch = [cos(pitch_rad),  0, sin(pitch_rad);
               0,               1, 0;
               -sin(pitch_rad), 0, cos(pitch_rad)];
    
    % 滚转旋转矩阵（绕X轴）  
    R_roll = [1, 0,               0;
              0, cos(roll_rad), -sin(roll_rad);
              0, sin(roll_rad),  cos(roll_rad)];
    
    % 组合旋转矩阵（先滚转后俯仰）
    R = R_pitch * R_roll;
end