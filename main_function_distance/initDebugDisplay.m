function [debugFig, debugHandles] = initDebugDisplay()
    % 初始化调试显示 - 2行3列布局（窗口模式 vs 连续模式）
    
    debugFig = figure('Position', [100, 100, 1400, 800], 'Name', 'IMU调试信息 - 窗口模式 vs 连续模式');
    
    % === 第一行：窗口模式数据 ===
    
    % 窗口模式加速度
    debugHandles.window_accel_axes = subplot(2, 3, 1);
    hold on;
    debugHandles.window_accel_x = plot(0, 0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    debugHandles.window_accel_y = plot(0, 0, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    debugHandles.window_accel_z = plot(0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    title('窗口模式加速度 (m/s²)');
    xlabel('采样点');
    ylabel('加速度 (m/s²)');
    grid on;
    legend('show');
    
    % 窗口模式速度
    debugHandles.window_velocity_axes = subplot(2, 3, 2);
    hold on;
    debugHandles.window_velocity_x = plot(0, 0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    debugHandles.window_velocity_y = plot(0, 0, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    debugHandles.window_velocity_z = plot(0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    title('窗口模式速度 (m/s)');
    xlabel('采样点');
    ylabel('速度 (m/s)');
    grid on;
    legend('show');
    
    % 窗口模式位移
    debugHandles.window_displacement_axes = subplot(2, 3, 3);
    hold on;
    debugHandles.window_displacement_x = plot(0, 0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    debugHandles.window_displacement_y = plot(0, 0, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    debugHandles.window_displacement_z = plot(0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    title('窗口模式位移 (m)');
    xlabel('采样点');
    ylabel('位移 (m)');
    grid on;
    legend('show');
    
    % === 第二行：连续模式数据 ===
    
    % 连续模式加速度
    debugHandles.continuous_accel_axes = subplot(2, 3, 4);
    hold on;
    debugHandles.continuous_accel_x = plot(0, 0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    debugHandles.continuous_accel_y = plot(0, 0, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    debugHandles.continuous_accel_z = plot(0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    title('连续模式加速度 (m/s²)');
    xlabel('采样点');
    ylabel('加速度 (m/s²)');
    grid on;
    legend('show');
    
    % 连续模式速度
    debugHandles.continuous_velocity_axes = subplot(2, 3, 5);
    hold on;
    debugHandles.continuous_velocity_x = plot(0, 0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    debugHandles.continuous_velocity_y = plot(0, 0, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    debugHandles.continuous_velocity_z = plot(0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    title('连续模式速度 (m/s)');
    xlabel('采样点');
    ylabel('速度 (m/s)');
    grid on;
    legend('show');
    
    % 连续模式位移
    debugHandles.continuous_displacement_axes = subplot(2, 3, 6);
    hold on;
    debugHandles.continuous_displacement_x = plot(0, 0, 'r-', 'LineWidth', 1.5, 'DisplayName', 'X');
    debugHandles.continuous_displacement_y = plot(0, 0, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y');
    debugHandles.continuous_displacement_z = plot(0, 0, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Z');
    title('连续模式位移 (m)');
    xlabel('采样点');
    ylabel('位移 (m)');
    grid on;
    legend('show');
    
    % 调整子图间距
    set(debugFig, 'Color', 'white');
end