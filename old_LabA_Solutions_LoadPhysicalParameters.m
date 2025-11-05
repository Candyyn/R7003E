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


% undre
a = ((I_w/l_w)+ l_w*m_b + l_w*m_w);
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

A = inv(A1)*A2;

B1 = [
    0;
    a4;
    0;
    -b5
    ];

B = inv(A1)*B1;

C = [
    0, 0, 1, 0;
    ];

D = 0;



SYS = ss(A,B,C,D);
ss2zp(A, B, C, D);

s = tf('s');
G = tf(SYS);

Zeros = zero(G);
Poles = pole(G);
%ss2zp(A, B, C, D);

% pzplot(G);    
% pzplot(G);    

%%

[z, p, k] = ss2zp(A, B, C, D);
ss2zp(A, B, C, D);
z = [ 0 ];
p = p(2:end, 1);
% Plant TF is thus
[num, den] = zp2tf(z, p, k);
G = tf(num, den);

Zeros = zero(G);
Poles = pole(G)

%pzplot(G)
%impulse(feedback(G, 1))

Df = zeros(4, 2);
%% Task 3.5 Design a PID controller stabilizing the TF
% Of our poles, two are in LHP and are thus stable.
% Our single real root on the RHP is unstable and must be moved.
% Our desired poles are thus (where the unstable pole has been moved):
dp = abs([p(p < 0, :); -90]);
dp
%dp = [475.0690, 5.6571, 70];
den = den(1, 2:end); % we are not interested in the coeff. of s^3'

% We find our PID parameters after equating them on paper
%kP = (dp(1)*dp(2) + dp(2)*dp(3) + dp(1)*dp(3) - den(2)) / k;
%kI = (dp(1)*dp(2)*dp(3) - den(3)) / k;
%kD = (sum(dp) - den(1)) / k;

p1 = dp(1);
p2 = dp(2);
p3 = dp(3);

kI = (-15390 - p3*p2*p1 ) / 90.03;
kP = (-62.08 - (p3*p2 + p3*p1 + p2*p1) ) / 90.03;
kD = ( 475 - (p3 + p2 + p1) ) / 90.03;


% We verify
controller = pid(kP, kI, kD);
system = feedback(G, controller); % closed-loop system

[num, den] = tfdata(system);  % Get numerator and denominator
%num = cell2mat(num);  % Convert cell array to matrix
%den = cell2mat(den);  % Convert cell array to matrix


%[zc, pc, kc] = tf2zp(num, den);
[zc, pc, kc] = zpkdata(system, 'v');
% Compare the above with [ z, p, k ]
% Visually inspect system response via:
%impulse(feedback(system, controller))
feedback(system, controller);

gh = G * controller; 
bandwidth(gh) + bandwidth(1 + gh)

push = inv(gamma) * [ l_w; l_b ];
inv(A1)
b_f = [ 0 0; B(2) push(1); 0 0; B(4) push(2) ];

b_f

%B_for_poking = b_f;
B_for_poking = [ B , [0; l_w/gamma; 0; l_b/gamma] ];

Cf = eye(4);
C_for_full_observability = Cf;

Df = zeros(4, 2);
D_for_full_observability_and_poking = Df;
sampleTime = 0.0219; 
fSamplingPeriod = sampleTime

%t = tiledlayout(2,2);
%ax1 = nexttile;
%plot(ax1, x_w.time,x_w.signals.values);
%title(ax1, 'x_w');
%ax2 = nexttile; 
%plot(ax2, theta_b.time,theta_b.signals.values);
%title(ax2,'\theta_b');
%ax3 = nexttile;
%plot(ax3, v_m.time,v_m.signals.values);
%title(ax3, 'v_m');
%ax4 = nexttile;
%plot(ax4, d.time,d.signals.values);
%title(ax4, 'd');

%% Task 3.8 Convert the controller to the discrete domain
% Find the bandwidth of the system.
sys_bw = bandwidth(controller*G) + bandwidth(1 + controller * G);
%sys_bw
% NOTE: the below assumes the while feedback system as the controller,
% which is probably incorrect.
% system = zpk(zc, pc, kc);
% system = minreal(system);
% sys_bw = bandwidth(system);
% In the above, we redeclare our system in zpk-foR_m so that minreal works.
% minreal cancels the zero/pole at s = 0 that comes from adding system
% feedback.

% We find our sampling frequency as according to 8.3.6 in FPE7.
% See also example 8.1, page 623.
sampling_freq = sys_bw * 25;
% We convert to Hz
sampling_freq = sampling_freq / (2 * pi);
T = 1 / sampling_freq;

% We compute the discrete controller
controllerd = c2d(pid(kP, kI, kD, T), T, 'zoh')
t = tiledlayout(2,2);
ax1 = nexttile;
plot(ax1, ScopeData8.time,ScopeData8.signals.values);
title(ax1, '\theta_b(t)');
ax2 = nexttile; 
plot(ax2, ScopeData11.time,ScopeData11.signals.values);
title(ax2,'v_m(t)');
ax3 = nexttile;
plot(ax3, ScopeData7.time,ScopeData7.signals.values);
title(ax3, '\theta_b^{lin}(t)');
ax4 = nexttile;
plot(ax4, ScopeData10.time,ScopeData10.signals.values);
title(ax4, 'v_m^{lin}(t)');

