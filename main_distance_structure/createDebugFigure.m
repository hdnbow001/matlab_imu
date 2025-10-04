function debugFig = createDebugFigure(guiHandles)
    % 创建调试数据显示窗口并返回句柄
    
    guiHandles.debugFig = figure('Position', [100, 100, 1200, 800], ...
        'Name', '传感器数据调试显示', 'NumberTitle', 'off');
    
    % 加速度数据显示
    subplot(3, 1, 1);
    guiHandles.h_accel_x = plot(NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X轴');
    hold on;
    guiHandles.h_accel_y = plot(NaN, NaN, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y轴');
    guiHandles.h_accel_z = plot(NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z轴');
    title('线性加速度 (m/s²)');
    legend('show');
    grid on;
    xlabel('采样点');
    ylabel('加速度 (m/s²)');
    
    % 速度数据显示
    subplot(3, 1, 2);
    guiHandles.h_velocity_x = plot(NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X轴');
    hold on;
    guiHandles.h_velocity_y = plot(NaN, NaN, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y轴');
    guiHandles.h_velocity_z = plot(NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z轴');
    title('速度 (m/s)');
    legend('show');
    grid on;
    xlabel('采样点');
    ylabel('速度 (m/s)');
    
    % 位移数据显示
    subplot(3, 1, 3);
    guiHandles.h_displacement_x = plot(NaN, NaN, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X轴');
    hold on;
    guiHandles.h_displacement_y = plot(NaN, NaN, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y轴');
    guiHandles.h_displacement_z = plot(NaN, NaN, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z轴');
    title('位移 (m)');
    legend('show');
    grid on;
    xlabel('采样点');
    ylabel('位移 (m)');
    
    debugFig = guiHandles.debugFig;
end