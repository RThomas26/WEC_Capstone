%%% Preliminary Matlab Float Model %%%
%% No motor
clear all; close all;
% known parameters
m = 3; % float mass in kg
g = 9.81; % m/s^2
rho = 997; % density of water kg/m^2
xa = 0.219; % cross sectional area of float affected by buoyant force m^2

% paramters that are imported from WECSIM - need a file to pull from WEC
% SIM (these rn are from frq = 1 hz)
M_A8 = 6.68903; % added mass at infinity = constant number
M_A = 8.00710; % added mass (function of frequency) defined as an array
B = 25.66633; % radiation damping (function of frequency) defined as an array
% Force to Velocity transfer function - vertical heave velocity of float
K_s = rho * g * xa; % spring constant (buoyant force)
s = tf('s');
num = s;
den = s^2 * (m + M_A8) + s * (B + M_A) + K_s;
%den = s^2 * (m + M_A8 + M_A) + s * (B) + K_s;
G = num / den;

% PID control
Kp1 = 150;

Kp = 160;
Ki = 2000;
Kd = 1.05;

Kp2 = .3;
Ki2 = 0;
Kd2 = .1;

KpSim = 1834;
KiSim = 9541;
KdSim = 616;

Cpid2 = pid(Kp, Ki, Kd);
clear pid;
[Cpid, info] = pidtune(G, 'PID', 25)
clear pid;
C_pd = pid(Kp2, Ki2, Kd2)
G_p = G*Kp1; 
clear pid;
CpidSim = pid(KpSim, KiSim, KdSim);

% make closed loop
G_pd = feedback(C_pd*G, 1);
G_pid = feedback(Cpid*G, 1); 
G_pSys = feedback(G_p, 1);
G_pid2 = feedback(Cpid2*G, 1);
G_pidSim = feedback(CpidSim*G, 1);

% Simulate the system response to a step input
t = 0:0.01:5; % time vector for simulation

v = step(G, t); % compute the open loop step response

% compute the closed loop step response
vp = step(G_pSys, t); 
vpd = step(G_pd, t);
vpid = step(G_pid, t);
vpid2 = step(G_pid2, t);
vpidSim = step(G_pidSim, t);


% Proportional Controller
figure(1);
plot(t, v); % plot the vertical heave velocity response
hold on;
plot(t,vp);
grid on;
title('Proportional control');
legend('step response', 'P control K = 160');
hold off;

% PD control (no steady state errors currently)
figure(2);
plot(t, v); % plot the vertical heave velocity response
grid on;
title('Open Loop Step Response');
legend('step response');

figure(3);
plot(t, v); % plot the vertical heave velocity response
hold on;
grid on;
plot(t, vpidSim);
plot(t, vpid2);
plot(t, vpid);

xlabel('Time (s)');
ylabel('Response');
title('PID Control model');
legend('step response', 'Simulink Output','TRIAL PID', 'PID Response','Location', 'best');

hold off;