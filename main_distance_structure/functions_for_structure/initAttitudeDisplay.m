% 初始化姿态显示函数
function [attitudeQuiver, attitudeText, attitudeSphere, attitudeAxes] = initAttitudeDisplay(attitude_subplot)
    % 设置当前坐标轴
    axes(attitude_subplot);
    attitudeAxes = attitude_subplot; % 返回坐标轴句柄
    
    % 清除并重新设置
    cla(attitudeAxes);
    hold on;
    grid on;
    axis equal;
    xlim([-1.5, 1.5]);
    ylim([-1.5, 1.5]);
    zlim([-1.5, 1.5]);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('姿态显示');
    view(45, 25);
    
    % 绘制参考坐标系
    quiver3(0, 0, 0, 1, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, 1, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(0, 0, 0, 0, 0, 1, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    % 创建单位球体
    [x, y, z] = sphere(20);
    attitudeSphere = surf(x, y, z, 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'FaceColor', [0.8 0.8 0.8]);
    
    % 创建姿态箭头
    attitudeQuiver = quiver3(0, 0, 0, 0, 0, 1, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.8);
    
    % 修正文本创建 - 使用sprintf生成带换行的文本
    initialText = sprintf('俯仰角: 0.00°\n滚转角: 0.00°\n偏航角: 0.00°');
    % attitudeText = text(-1, -1, 1.2, initialText, ...
    %     'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    %     'VerticalAlignment', 'top', 'Interpreter', 'none');
    attitudeText = text(3, -2, 3, initialText, ...
    'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'right', 'Interpreter', 'none');

    
    hold off;
end