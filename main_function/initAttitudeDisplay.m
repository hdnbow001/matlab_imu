% ��ʼ����̬��ʾ����
function [attitudeQuiver, attitudeText] = initAttitudeDisplay(parentAxes)
    % ���õ�ǰ������
    axes(parentAxes);
    
    % �����򻯵���������ϵ
    [sx, sy, sz] = sphere(12); % ���������ܶ�
    r = 0.99; % ��΢С��1��ȷ��ʸ���ɼ�
    h_sphere = surf(r*sx, r*sy, r*sz, 'FaceAlpha', 0.05, 'EdgeAlpha', 0.1, 'FaceColor', [0.8 0.8 0.8]);
    hold on;
    axis equal;
    grid on;
    xlabel('X��');
    ylabel('Y��');
    zlabel('Z��');
    title('��������̬�����ʾ');
    
    % ����3D��ת
    rotate3d on;
    
    % ����������
    quiver3(0,0,0,1.2,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % X��
    quiver3(0,0,0,0,1.2,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Y��
    quiver3(0,0,0,0,0,1.2, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Z��
    
    % ��ʼ���ٶ�ʸ������������
    attitudeQuiver = quiver3(0,0,0,0,0,0, 'k', 'LineWidth', 3, 'MaxHeadSize', 0.5);
    
    % ����ı���Ϣ
    text(1.3, 0, 0, 'X', 'Color', 'r', 'FontWeight', 'bold');
    text(0, 1.3, 0, 'Y', 'Color', 'g', 'FontWeight', 'bold');
    text(0, 0, 1.3, 'Z', 'Color', 'b', 'FontWeight', 'bold');
    
    % ��ӽǶ���Ϣ�ı�
    attitudeText = text(-1.5, -1.5, -1.5, sprintf('������: 0.00��\n��ת��: 0.00��\nƫ����: 0.00��'), ...
        'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 10);
end