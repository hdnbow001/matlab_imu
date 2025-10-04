% 计算姿态角度函数
function [pitch, roll] = calculateAttitude(ax, ay, az, accel_range)
    % 将原始数据转换为G单位
    ax_g = (ax / 32768) * accel_range;
    ay_g = (ay / 32768) * accel_range;
    az_g = (az / 32768) * accel_range;
    
    % 计算俯仰角(pitch)和滚转角(roll)
    % 由于传感器z轴与重力方向相反，需要调整符号
    pitch = atan2(-ax_g, sqrt(ay_g^2 + az_g^2)) * 180/pi;
    roll = atan2(-ay_g, -az_g) * 180/pi;
end