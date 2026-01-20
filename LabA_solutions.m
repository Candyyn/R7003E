fprintf('loading the physical parameters...');
g   = 9.8;
b_f = 0;
m_b = 0.381;
l_b = 0.112;
I_b = 0.00616;
m_w = 0.036;
l_w = 0.021;
I_w = 0.00000746;
R_m = 4.4;
L_m = 0;
b_m = 0;
K_e = 0.444;
K_t = 0.470;
fprintf('done\n');


%% 3.1 Derive the Equations of Motion
disp("--- 3.1 Derive the Equations of Motion");

% undre
a = ((I_w/l_w)+ l_w*m_b + l_w*m_w)
a3 = m_b*l_b*l_w;
a1 = -((K_e*K_t)/(R_m*l_w) + (b_f/l_w));
a2 = ((K_e*K_t)/(R_m) + (b_f));
a4 = (K_t/R_m);

% övre

gamma = I_b + m_b*l_b^2;
b4 = (m_b * l_b);
b1 = -((K_e*K_t)/R_m + b_f);
b2 = ((K_e*K_t)/(R_m*l_w) - (b_f/l_w));
b3 = m_b * l_b*g;
b5 = (K_t/R_m);




%% 3.3 Write the linearized Equations of Motion in State-Space form
disp("--- 3.3 Write the linearized Equations of Motion in State-Space form");

A1 = [
    1, 0, 0, 0;
    0, a, 0, a3;
    0, 0, 1, 0;
    0, b4, 0, gamma
    ];

A2 = [
    0, 1, 0, 0;
    0, a1, 0, a2;
    0, 0, 0, 1;
    0, b2, b3, b1
    ];

A = inv(A1)*A2 % Compare with 3.3.1 Troubleshoot

B1 = [
    0;
    a4;
    0;
    -b5
    ];

B = inv(A1)*B1; % Compare with 3.3.1 Troubleshoot
    

C = [
    0, 0, 1, 0;
    ];

D = 0;

%% 3.4 Determine the transfer function relative to system
disp("--- 3.4 Determine the transfer function relative to system");


SYS = ss(A,B,C,D);
ss2zp(A, B, C, D);

s = tf('s');
G = tf(SYS);

Zeros = zero(G);
Poles = pole(G);


[z, p, k] = ss2zp(A, B, C, D);
z = [ 0 ];
p = p(2:end, 1);

[num, den] = zp2tf(z, p, k);
G = tf(num, den);

Zeros = zero(G);
Poles = pole(G);

SYS = ss(A,B,C,D);
G = tf(SYS);             % This already gives the minimal TF
[z, p, k] = zpkdata(G,'v');


%pzplot(G) % What is this??
D_f = zeros(4, 2);


%% 3.5 Design a PID controller stabilizing the TF
disp("--- 3.5 Design a PID controller stabilizing the TF")

dp = abs([p(p < 0, :); -90]);

den = den(1, 2:end); % we are not interested in the coeff of s^3

p1 = dp(1);
p2 = dp(2);
p3 = dp(3);

%kP = (dp(1)*dp(2) + dp(2)*dp(3) + dp(1)*dp(3) - den(2)) / k;
%kI = (dp(1)*dp(2)*dp(3) - den(3)) / k;
%kD = (sum(dp) - den(1)) / k;

kI = (-15390 - p3*p2*p1 ) / 90.03;
kP = (-62.08 - (p3*p2 + p3*p1 + p2*p1) ) / 90.03;
kD = ( 475 - (p3 + p2 + p1) ) / 90.03;


disp(["kI", kI])
disp(["kP", kP])
disp(["kD", kD])

controller = pid(kP, kI, kD);
system = feedback(G, controller); % Gives us the closed loop system


[num, den] = tfdata(system);

% Function differ prob due to new matlab version
[zc, pc, kc] = zpkdata(system, 'v');  
%disp(pc)
%impulse(feedback(system, controller));
%feedback(system, controller);

%% 3.7 Check if everything is working as it should be
disp("--- 3.7 Check if everything is working as it should be")




push = inv(gamma) * [ l_w; l_b ];
% inv(A1);
b_f = [ 0 0; B(2) push(1); 0 0; B(4) push(2) ];
B_for_poking = [ B , [0; l_w/gamma; 0; l_b/gamma] ]; % Should be the same as b_f

Cf = eye(4);
C_for_full_observability = Cf;

Df = zeros(4, 2);
D_for_full_observability_and_poking = Df;



%% 3.8 Convert the controller to the discrete domain
disp("--- 3.8 Convert the controller to the discrete domain")

%gh = G * controller;  % Should be here?
%bandwidth(gh) + bandwidth(1 + gh);


sys_bw = bandwidth(controller*G) + bandwidth(1 + controller * G);
sampling_freq = sys_bw * 25; % From ch 3.8.6
sampling_freq = sampling_freq / (2 * pi);
fSamplingPeriod = 1 / sampling_freq;

%sampleTime = 0.0219;  % From ch 3.8.6
%fSamplingPeriod = sampleTime


% IF THIS SECTION CAUSES ERROR, RUN THE SIMULATION FOR LAB A

% set(0,'DefaultFigureVisible','on');   % in case it was turned off earlier
% figure('Visible','on');               % open a fresh window
% t = tiledlayout(2,2);
% ax1 = nexttile;
% plot(ax1, theta_b.time, theta_b.signals.values);
% title(ax1,'\theta_b(t)')
% ax2 = nexttile;
% plot(ax2, x_w.time, x_w.signals.values);
% title(ax2,'v_m(t)');
% ax3 = nexttile;
% plot(ax3, v_m.time, v_m.signals.values);
% title(ax3, '\theta_b^{lin}(t)');
% ax4 = nexttile;
% plot(ax4, d.time, d.signals.values);
% title(ax4, 'v_m^{lin}(t)');
% 
% 
% grid on; drawnow; 
% 


%Here was some plots
%controllerd = c2d(pid(kP, kI, kD, fSamplingPeriod), fSamplingPeriod, 'zoh')
%t = tiledlayout(2,2);
%ax1 = nexttile;
%ScopeData8 = theta_b;%
%plot(ScopeData8.time,ScopeData8.signals.values)
%title('\theta_b(t)')
%ax2 = nexttile; 
%plot(ax2, ScopeData11.time,ScopeData11.signals.values);
%title(ax2,'v_m(t)');
%ax3 = nexttile;
%plot(ax3, ScopeData7.time,ScopeData7.signals.values);
%title(ax3, '\theta_b^{lin}(t)');
%ax4 = nexttile;
%plot(ax4, ScopeData10.time,ScopeData10.signals.values);
%title(ax4, 'v_m^{lin}(t)');

