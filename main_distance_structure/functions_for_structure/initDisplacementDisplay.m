% 初始化位移显示函数
function [h_displacement, h_displacement_text, displacementAxes] = initDisplacementDisplay(parentAxes)
    % 设置当前坐标轴
    axes(parentAxes);
    
    % 创建3D位移轨迹图
    h_displacement = plot3(NaN, NaN, NaN, 'b-', 'LineWidth', 2);
    hold on;
    % 添加起点标记
    plot3(0, 0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    grid on;
    xlabel('X位移 (m)');
    ylabel('Y位移 (m)');
    zlabel('Z位移 (m)');
    title('传感器位移轨迹 (5秒窗口)');
    
    % 设置固定坐标轴范围 [-1, 1] 米
    xlim([-1, 1]);
    ylim([-1, 1]);
    zlim([-1, 1]);
    
    % 启用3D旋转
    rotate3d on;
    
    % 添加坐标轴
    line([-1 1], [0 0], [0 0], 'Color', 'r', 'LineStyle', '--');
    line([0 0], [-1 1], [0 0], 'Color', 'g', 'LineStyle', '--');
    line([0 0], [0 0], [-1 1], 'Color', 'b', 'LineStyle', '--');
    
    % 添加文本信息 - 使用元胞数组（与更新函数保持一致）
    text_cell = {
        '位移: (0.00, 0.00, 0.00) m', 
        '窗口: 1'
    };
    h_displacement_text = text(0, 0, 1.1, text_cell, ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
        'Interpreter', 'none'); % 重要：禁用LaTeX解释器
    
    % 保存坐标轴句柄
    displacementAxes = gca;
end