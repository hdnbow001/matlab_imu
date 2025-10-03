% 初始化姿态显示函数
function [attitudeQuiver, attitudeText] = initAttitudeDisplay(parentAxes)
    % 设置当前坐标轴
    axes(parentAxes);
    
    % 创建简化的球面坐标系
    [sx, sy, sz] = sphere(12); % 减少网格密度
    r = 0.99; % 稍微小于1以确保矢量可见
    h_sphere = surf(r*sx, r*sy, r*sz, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.1, 'FaceColor', [0.8 0.8 0.8]);
    hold on;
    axis equal;
    grid on;
    xlabel('X轴');
    ylabel('Y轴');
    zlabel('Z轴');
    title('传感器姿态球面表示');
    
    % 启用3D旋转
    rotate3d on;
    
    % 绘制坐标轴
    quiver3(0,0,0,1.2,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X轴
    quiver3(0,0,0,0,1.2,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y轴
    quiver3(0,0,0,0,0,1.2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z轴
    
    % 初始加速度矢量（零向量）
    attitudeQuiver = quiver3(0,0,0,0,0,0, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    % 添加文本信息
    text(1.3, 0, 0, 'X', 'Color', 'r', 'FontWeight', 'bold');
    text(0, 1.3, 0, 'Y', 'Color', 'g', 'FontWeight', 'bold');
    text(0, 0, 1.3, 'Z', 'Color', 'b', 'FontWeight', 'bold');
    
    % 添加角度信息文本
    attitudeText = text(-1.5, -1.5, -1.5, sprintf('俯仰角: 0.00°\n滚转角: 0.00°\n偏航角: 0.00°'), ...
        'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);
end