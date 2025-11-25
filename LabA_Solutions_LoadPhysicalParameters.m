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

A = inv(A1)*A2; % Compare with 3.3.1 Troubleshoot

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



%% 4.5 -- Check the controllability and observability properties of the linearized system
disp("Check the controllability and observability properties of the linearized system")

O = obsv(A, C) % At this state, our obserablivity is not full, due to rankO = 3 where n = 4
rankO = rank(O)

% This will make it so there is a state/mode in the system that cant be
% seen or detected from the output 

% ctrb - controllability.
ctr = ctrb(A, B) % Why A and B? prob due to First and Second part of the entire system
rankC = rank(ctr) % Our RankC and Ctrl n is equal. Which makes our system "controllable?"

%% 4.6.1
oldPC = pc

etha = 0.7
p_slow = pc(4,1)

w_n = (abs(p_slow) / etha) % Our Real del

w_d = w_n * sqrt(1 - etha^2) % Imaginar
p_dom1 = -etha * w_n + 1j * w_d;
p_dom2 = -etha * w_n - 1j * w_d;

p_fast = [pc(2,1); pc(3,1)]

p_cl = [p_fast; p_dom1; p_dom2]
%p_cl = [pc(2,1); p_dom1; p_dom2; pc(3,1)]

%Ts = 4 / (etha * w_n) % Settlings time (2% criteria) 

% Choose our -5.6576 from oldPC  

% outcommended old code thats not relevant
%Tr = 0.5; %We want a fast rise time <= 0.5
%Mp = 0; % Overshoot -> etha = 1

%ess = 0.01; % Error (want it less than 1%)
%Ts  <2s Settlingstime

%risetime = 1.8 % 1.8 % book  3.4.1 Rise Time
%w_n = risetime / Tr;

%Ts = (-(ess)) / (etha * w_n) 

%pc_des = [s + (w_d*i); s - (w_d*i); -50; -100]
%pc_des

K = place(A, B, p_cl) % First way
% -574.8968 -296.9783 -413.5632  -68.4583

%% 4.7.1

% a) Choose C

C_bar = [5 1 10 2]%[5 1 10 2];
D_bar = 0;


% b) Show the locus plot

%C_bar = [20,1,25,1];

s = tf('s');
sys_pos_ss = ss(A, B, C_bar, D_bar);
G_pos = tf(sys_pos_ss);
G_pos_min = minreal(G_pos);

[num_pos, den_pos] = tfdata(G_pos_min, 'v');

A_neg      = -A;
sys_neg_ss = ss(A_neg, B, C_bar, 0);
G_neg      = tf(sys_neg_ss);
G_neg_min  = minreal(G_neg);



%G_pos = (-90.03*s)/((s+475)*(s+5.65)*(s-5.72));
%G_neg = (-90.03*-s)/((-s+475)*(-s+5.65)*(-s-5.72));
        
sysGG = G_neg*G_pos;

rlocus(sysGG);

rho = 1; % 0.1, 1, 10, 100
   
all_roots = rlocus(sysGG, rho);
neg_roots = all_roots(all_roots<=0);

    
Q = rho*transpose(C_bar)*C_bar;
     
K2 = lqr(A,B,Q, 1) % Secound awy to get K value
%-20.0000  -47.2074  -72.3938  -10.9849

%% 4.8.1

C_luen = [1 0 0 0;
          0 0 1 0];

A;
%   1.0e+03 *

%         0    0.0010         0         0
%         0   -0.4350   -0.0061    0.0091
%         0         0         0    0.0010
%         0    1.9034    0.0620   -0.0400
C_luen';
pc';  % from 3.5

format bank;

Acl = A - B*K;
Acl2 = A - B*K2;
p_cl1 = eig(Acl);
p_cl2 = eig(Acl2);

p_cl 
poles_cl = 4 * p_cl;

L= place(A', C_luen', (4*p_cl)').'
L_K = place(A', C_luen', p_cl1')'
L_K2 = place(A', C_luen', p_cl2')'

eig(A - L*C_luen)
rank(obsv(A, C_luen))

% Reducer

% Partition matrices
Aww = A(1,1);
Awr = A(1,2:4);
Arw = A(2:4,1);
Arr = A(2:4,2:4);

Bw  = B(1,:);
Br  = B(2:4,:);

C2w = C_luen(2,1);
C2r = C_luen(2,2:4);

% Pole choice (2–6x faster than -5.66)
p_slow    = -5.66;
p_obs_red = [2 4 6] * p_slow;
Lr        = place(Arr.', C2r.', p_obs_red).';

% Reduced-order observer gains:
M1 = Arr - Lr*C2r      % 3x3
M2 = Arw - Lr*C2w      % 3x1
M3 = Lr                % 3x1
M4 = Br                % 3xm

M5 = [0 0 0;
      1 0 0;
      0 1 0;
      0 0 1]           % 4x3

M6 = [1; 0; 0; 0]      % 4x1
M7 = [0; 0; 0; 0]      % 4x1
M7 = [
      0     0     0
      1     0     0
      0     1     0
      0     0     1
 ]


C_acc=[1 0 0 0];
C_nacc=[0 0 1 0];
T_inv=[C_acc;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
T=inv(T_inv);
A_tilde=T_inv*A*T
B_tilde=T_inv*B
C_acc_tilde = C_acc*T
C_nacc_tilde = C_nacc * T

A_yy=A_tilde(1,1)
A_yx=A_tilde(1,2:4)
A_xy=A_tilde(2:4,1)
A_xx=A_tilde(2:4,2:4)


C_tilde_y = C_nacc_tilde(1,1)
C_tilde_chi = C_nacc_tilde(1,2:4)

B_tilde_y = B_tilde(1,1);
B_tilde_chi = B_tilde(2:4,1);
CC = [A_yx; C_tilde_chi]
L = (place(A_xx', CC',3*[-5.6, -5.65, -4*5.65]))'

L_acc = L(1:3,1)
L_nacc= L(1:3,2)

%3x3 - 1x1 * 1x3  - 2x1 * 1x3  
M1 = A_xx - L_acc*A_yx-L_nacc*C_tilde_chi
M2 = B_tilde_chi - L_acc *B_tilde_y
M3 = (A_xy - L_acc*A_yy - L_nacc*C_tilde_y)
M4 = L_nacc
M5 = L_acc
M6 = T(1:4,1)
M7 = T(1:4,2:4)
