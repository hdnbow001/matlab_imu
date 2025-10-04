% 1. 参数初始化
function [sysParams, imuParams] = initializeSystemParameters()
    % 初始化系统运行参数和IMU传感器参数
    
    % 系统运行参数
    sysParams.sampleRate = 10;           % 采样率 10Hz
    sysParams.duration = 100;            % 总时长 100秒
    sysParams.maxPoints = round(sysParams.duration * sysParams.sampleRate); % 1000个采样点
    sysParams.dt = 1 / sysParams.sampleRate; % 采样时间间隔
    sysParams.syncBytes = [170, 85];     % 数据帧同步头 AA 55
    
    % 串口配置
    sysParams.comPort = 'COM3';          % 串口号-hd开发机
    %sysParams.comPort = 'COM7';          % 串口号-UIH开发机
    sysParams.baudRate = 115200;         % 波特率
    
    % IMU传感器参数
    imuParams.accelRange = 2;            % 加速度计量程 ±2G
    imuParams.gyroRange = 250;           % 陀螺仪量程 ±250dps
    imuParams.windowSize = 5 * sysParams.sampleRate; % 5秒滑动窗口
    
    % 算法参数
    imuParams.calibrationSamples = 200;  % 零偏校准采样点数
    imuParams.dynamicBiasAlpha = 0.001;  % 动态零偏估计滤波系数
    imuParams.complementaryFilterAlpha = 0.98; % 互补滤波系数
    
    fprintf('系统参数初始化完成\n');
    fprintf('采样率: %d Hz, 总时长: %d 秒, 总点数: %d\n', ...
        sysParams.sampleRate, sysParams.duration, sysParams.maxPoints);
end