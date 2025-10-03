% �򻯵ĸĽ���ͨ�˲�������
function [filteredData, new_prev] = improvedHighPassFilterSimple(data, alpha, dt, prev)
    % ��ʼ���˲������������
    filteredData = zeros(size(data));
    
    % �����˲���ϵ��
    RC = 1 / (2 * pi * 0.1); % ��ֹƵ��0.1Hz
    alpha_val = dt / (RC + dt);
    
    % �����һ�����ݵ�
    if ~isempty(data)
        filteredData(1) = alpha_val * (prev + data(1));
    end
    
    % �Ժ������ݵ�Ӧ�øĽ��ĸ�ͨ�˲���
    for i = 2:length(data)
        filteredData(i) = alpha_val * (filteredData(i-1) + data(i) - data(i-1));
    end
    
    % ����ǰһ��ֵ
    if ~isempty(data)
        new_prev = filteredData(end);
    else
        new_prev = prev;
    end
end