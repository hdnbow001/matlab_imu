
function velocity = integrateAcceleration(acceleration, dt, stationary)
    % 正确的加速度积分
    n = length(acceleration);
    velocity = zeros(n, 1);
    
    current_vel = 0;
    for i = 1:n
        % 梯形积分
        if i == 1
            delta_v = acceleration(i) * dt;
        else
            delta_v = (acceleration(i-1) + acceleration(i)) * 0.5 * dt;
        end
        
        current_vel = current_vel + delta_v;
        
        % 在静止点重置速度
        if stationary(i)
            current_vel = 0;
        end
        
        velocity(i) = current_vel;
    end
end

