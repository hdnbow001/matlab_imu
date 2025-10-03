% ��׼��ͨ�˲���ʵ��
function [filteredData, new_prev] = improvedHighPassFilter(data, prev,dt)
    if nargin < 3
        prev = 0;
    end
    
    % �����˲���ϵ�����̶���ֹƵ��0.1Hz��
    RC = 1 / (2 * pi * 0.1); % ��ֹƵ��0.1Hz
    alpha_val = dt / (RC + dt);
    
    filteredData = zeros(size(data));
    
    if ~isempty(data)
        % �����һ�����ݵ�
        filteredData(1) = alpha_val * (prev + data(1));
        
        % �Ժ������ݵ�Ӧ�øĽ��ĸ�ͨ�˲���
        for i = 2:length(data)
            filteredData(i) = alpha_val * (filteredData(i-1) + data(i) - data(i-1));
        end
        
        new_prev = filteredData(end);
    else
        new_prev = prev;
        filteredData = [];
    end
end