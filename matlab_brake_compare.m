%% 球杆系统双环 PID 仿真对比：纯 PD vs 加制动包络
% 被控对象：球位置 x 与摆杆角 θ 的关系（小角度近似）
%   x'' = (g/L) * θ   →  外环看到的是 1/s² 二阶双积分对象
% 内环：PD 跟踪 θ* → θ 足够快，这里直接近似 θ ≈ θ*
% 外环：位置误差 → PD → θ*（目标摆杆角）
% 制动包络：θ* 按 √(2*K_brake*|e|) 动态限幅，仅 e 在减小（靠近目标）时生效

clear; close all; clc;

%% === 公共参数 ===
g        = 9.81;        % m/s²
L_beam   = 0.50;        % m 摆杆有效长度
K_plant  = g / L_beam;  % ≈ 19.6 rad/s² → 角度 1 rad ≈ 球加速度 19.6 m/s²

Ts       = 0.02;        % s 控制周期 20ms (50Hz)
t_end    = 5.0;         % s 仿真总时长
N        = round(t_end / Ts);

% 外环 PD 增益（先调好纯 PD 版稳定，再看包络加成）
Kp = 25.0;              % rad/m
Kd = 1.8;               % rad·s/m

% 制动包络参数（加包络版用）
K_brake  = 400.0;       % rad²/m → 包络斜率增益
theta_min = 0.02;       % rad 包络下限（防卡死）
theta_max = 0.35;       % rad 绝对上限（≈20°，你的 OutMax 物理值）

%% === 场景 1：纯 PD 双环（无包络） ===
x1   = zeros(N,1);      % 球位置 m
xd1  = zeros(N,1);      % 球速度 m/s
e1   = zeros(N,1);      % 位置误差
theta1 = zeros(N,1);    % 摆杆角 rad
theta_raw1 = zeros(N,1);% PID 原始输出（未包络）

x_ref = 0.30;           % m 目标位置（球滚到 30cm 处）

for k = 2:N
    t = k * Ts;
    % 外环 PD
    e1(k)    = x_ref - x1(k-1);
    e_dot    = -xd1(k-1);          % de/dt = -dx/dt (x_ref 是常数)
    theta_raw1(k) = Kp * e1(k) + Kd * e_dot;
    theta1(k) = min(max(theta_raw1(k), -theta_max), theta_max);  % 纯固定限幅
    % 对象：二阶积分 x'' = K_plant * theta
    xdd = K_plant * theta1(k);
    xd1(k) = xd1(k-1) + xdd * Ts;
    x1(k)  = x1(k-1)  + xd1(k) * Ts;
end

%% === 场景 2：PD + 制动包络 ===
x2   = zeros(N,1);
xd2  = zeros(N,1);
e2   = zeros(N,1);
theta2 = zeros(N,1);
theta_raw2 = zeros(N,1);
env2 = zeros(N,1);      % 记录包络值（可视化用）

for k = 3:N  % 需要 Error0 和 Error1，从第3拍开始
    t = k * Ts;
    e2(k)    = x_ref - x2(k-1);
    e_dot    = -xd2(k-1);
    theta_raw2(k) = Kp * e2(k) + Kd * e_dot;

    % ---- 制动包络核心 ----
    env = sqrt(2.0 * K_brake * abs(e2(k)));
    env = max(env, theta_min);     % 下限防卡死
    env = min(env, theta_max);     % 绝对上限
    env2(k) = env;

    % 方向判断：误差在减小？（Error0 - Error1 < 0）
    approaching = (e2(k) - e2(k-1)) < 0;
    if approaching
        theta2(k) = min(max(theta_raw2(k), -env), env);  % 包络限幅
    else
        theta2(k) = min(max(theta_raw2(k), -theta_max), theta_max);  % 远离时放行
    end

    xdd = K_plant * theta2(k);
    xd2(k) = xd2(k-1) + xdd * Ts;
    x2(k)  = x2(k-1)  + xd2(k) * Ts;
end

%% === 场景 3：往返（阶跃 → 反向） ===
x3   = zeros(N,1);
xd3  = zeros(N,1);
e3   = zeros(N,1);
theta3 = zeros(N,1);
theta_raw3 = zeros(N,1);
env3 = zeros(N,1);
x_ref3 = zeros(N,1);

for k = 3:N
    t = k * Ts;
    if t < 2.0
        x_ref3(k) = 0.30;
    else
        x_ref3(k) = -0.30;
    end
    e3(k)    = x_ref3(k) - x3(k-1);
    e_dot    = -xd3(k-1);
    theta_raw3(k) = Kp * e3(k) + Kd * e_dot;

    env = sqrt(2.0 * K_brake * abs(e3(k)));
    env = max(env, theta_min);
    env = min(env, theta_max);
    env3(k) = env;

    approaching = (e3(k) - e3(k-1)) < 0;
    if approaching
        theta3(k) = min(max(theta_raw3(k), -env), env);
    else
        theta3(k) = min(max(theta_raw3(k), -theta_max), theta_max);
    end

    xdd = K_plant * theta3(k);
    xd3(k) = xd3(k-1) + xdd * Ts;
    x3(k)  = x3(k-1)  + xd3(k) * Ts;
end

%% === 场景 4：往返（阶跃 → 反向，纯 PD 无包络对比） ===
x4   = zeros(N,1);
xd4  = zeros(N,1);
e4   = zeros(N,1);
theta4 = zeros(N,1);
x_ref4 = zeros(N,1);

for k = 2:N
    t = k * Ts;
    if t < 2.0
        x_ref4(k) = 0.30;
    else
        x_ref4(k) = -0.30;
    end
    e4(k)    = x_ref4(k) - x4(k-1);
    e_dot    = -xd4(k-1);
    theta_raw = Kp * e4(k) + Kd * e_dot;
    theta4(k) = min(max(theta_raw, -theta_max), theta_max);

    xdd = K_plant * theta4(k);
    xd4(k) = xd4(k-1) + xdd * Ts;
    x4(k)  = x4(k-1)  + xd4(k) * Ts;
end

%% === 画图 ===
t = (0:N-1)' * Ts;

figure('Name','阶跃响应对比','Position',[100 100 1000 700]);

subplot(3,2,1);
plot(t, x_ref*ones(N,1), 'k--','LineWidth',1.5); hold on;
plot(t, x1*100, 'r','LineWidth',1.5);
plot(t, x2*100, 'b','LineWidth',1.5);
grid on; ylabel('球位置 (cm)');
title('球位置：纯PD(红) vs PD+包络(蓝)');
legend('目标','纯PD','PD+包络','Location','best');

subplot(3,2,3);
plot(t, theta1*180/pi, 'r','LineWidth',1.2); hold on;
plot(t, theta2*180/pi, 'b','LineWidth',1.2);
grid on; ylabel('摆杆角 (°)');
title('摆杆角输出');
legend('纯PD','PD+包络','Location','best');

subplot(3,2,5);
plot(t, theta_raw2*180/pi, 'k-','LineWidth',1.0); hold on;
plot(t, env2*180/pi, 'g--','LineWidth',1.5);
plot(t, -env2*180/pi, 'g--','LineWidth',1.5);
ylim([-30 30]);
grid on; ylabel('角度 (°)'); xlabel('时间 (s)');
title('包络 vs 原始PID输出(加包络场景)');
legend('PID原始输出','包络上限','Location','best');

subplot(3,2,2);
plot(t, x_ref3*100, 'k--','LineWidth',1.5); hold on;
plot(t, x4*100, 'r','LineWidth',1.5);
plot(t, x3*100, 'b','LineWidth',1.5);
grid on; ylabel('球位置 (cm)');
title('往返：纯PD(红) vs PD+包络(蓝)');
legend('目标','纯PD','PD+包络','Location','best');

subplot(3,2,4);
plot(t, theta4*180/pi, 'r','LineWidth',1.2); hold on;
plot(t, theta3*180/pi, 'b','LineWidth',1.2);
grid on; ylabel('摆杆角 (°)');
title('往返摆杆角输出');

subplot(3,2,6);
plot(t, xd4*100, 'r','LineWidth',1.2); hold on;
plot(t, xd3*100, 'b','LineWidth',1.2);
grid on; ylabel('球速度 (cm/s)'); xlabel('时间 (s)');
title('球速度：纯PD(红) vs PD+包络(蓝)');

sgtitle('制动包络效果仿真（对象 1/s², 内环快近似 θ≈θ*）','FontSize',14);

%% === 打印关键指标 ===
function overshoot = calc_overshoot(x, x_ref)
    [~, idx] = max(abs(x - x_ref));
    overshoot = abs(x(idx) - x_ref) / abs(x_ref) * 100;
end

fprintf('==== 阶跃 0→30cm 指标 ====\n');
fprintf('纯PD  超调: %.1f%%, 最终误差: %.2f cm\n', calc_overshoot(x1, x_ref), x1(end)*100 - x_ref*100);
fprintf('PD+包络 超调: %.1f%%, 最终误差: %.2f cm\n', calc_overshoot(x2, x_ref), x2(end)*100 - x_ref*100);
