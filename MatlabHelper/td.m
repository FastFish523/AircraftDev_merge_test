%% 1. 生成测试信号：带噪声的正弦波
clear; clc; close all;

data = load('result.dat');
sigma_evl = data(145474:end,35);
sigma_az = data(145474:end,36);
sigma_evl_dot = data(145474:end,37);
sigma_az_dot = data(145474:end,38);


% 理想信号
x_ideal = sigma_evl;
dx_ideal = sigma_evl_dot;  % 理论微分
t = 1:size(x_ideal);

%% 3. 调用跟踪微分器
% 参数设置
fs = 200;          % 采样率
r = 1000;     % 速度因子
h = 0.5;   % 滤波因子
dt = 1/fs;  % 采样时间

% 初始化
x1 = 0;     % 跟踪信号
x2 = 0;     % 微分信号
x_tracked = zeros(size(x_ideal));
x_diff = zeros(size(x_ideal));

% 主循环
for i = 1:length(x_ideal)
    [x1, x2] = simple_td(x_ideal(i), x1, x2, r, h, dt);
    x_tracked(i) = x1;
    x_diff(i) = x2;
end

%% 4. 绘制结果
figure('Position', [100, 100, 800, 600]);

% 子图1：信号跟踪
subplot(2,2,1);
plot(t, x_ideal, 'b.', 'MarkerSize', 3, 'DisplayName', '带噪测量');
hold on;
plot(t, x_ideal, 'k-', 'LineWidth', 1.5, 'DisplayName', '理想信号');
plot(t, x_tracked, 'r-', 'LineWidth', 1.2, 'DisplayName', '跟踪信号');
xlabel('时间 (s)'); ylabel('幅值');
title('信号跟踪效果');
legend('Location', 'best');
grid on;

% 子图2：微分提取
subplot(2,2,2);
plot(t, dx_ideal, 'k-', 'LineWidth', 1.5, 'DisplayName', '理论微分');
hold on;
plot(t, x_diff, 'r-', 'LineWidth', 1.2, 'DisplayName', '估计微分');
xlabel('时间 (s)'); ylabel('微分值');
title('微分信号提取');
legend('Location', 'best');
grid on;

% 子图3：跟踪误差
subplot(2,2,3);
error_track = x_tracked - x_ideal;
plot(t, error_track, 'b-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差');
title(sprintf('跟踪误差 (RMS=%.4f)', rms(error_track)));
grid on;
yline(0, 'k--');

% 子图4：微分误差
subplot(2,2,4);
error_diff = x_diff - dx_ideal;
plot(t, error_diff, 'r-', 'LineWidth', 1);
xlabel('时间 (s)'); ylabel('误差');
title(sprintf('微分误差 (RMS=%.4f)', rms(error_diff)));
grid on;
yline(0, 'k--');
