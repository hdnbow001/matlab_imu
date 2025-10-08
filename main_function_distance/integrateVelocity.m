function displacement = integrateVelocity(velocity, dt)
    % 速度积分
    n = length(velocity);
    displacement = zeros(n, 1);
    
    current_disp = 0;
    for i = 1:n
        % 梯形积分
        if i == 1
            delta_d = velocity(i) * dt;
        else
            delta_d = (velocity(i-1) + velocity(i)) * 0.5 * dt;
        end
        
        current_disp = current_disp + delta_d;
        displacement(i) = current_disp;
    end
end