LabA_solutions;

% ---- If x and u are structures (Structure with time) ----
% t  = x.time;              % time vector
% X  = x.signals.values;    % states   (N × nx)
% U  = u.signals.values;    % control  (N × nu)
% 
% figure;
% subplot(2,1,1);
% plot(t, X);
% grid on;
% xlabel('Time [s]');
% ylabel('States x');
% legend('x_1','x_2','x_3','x_4');   % adjust if you have more/fewer states
% title('State trajectories');
% 
% subplot(2,1,2);
% plot(t, U);
% grid on;
% xlabel('Time [s]');
% ylabel('u');
% title('Control input');

%close all;
%clear all;
%clc;

%LabA_solutions;

%K = place(A, B, pc_des);
% K = [-449.2475 -112.4387 -178.4821  -22.2137] %<- poles placement%
%K = [-20      -47.207      -72.394      -10.985]%[ -5      -43.831      -64.573      -10.432]
%K = [-449.2475 -112.4387 -178.4821  -22.2137]
%K = [ -10.0000  -57.4908 -105.0371  -19.5009 ]
% K = [-3841.49 -737.34 -1425.32 -169.65]

zeta = 0.8;
wn = 7;
p1 = -zeta*wn + 1i*wn*sqrt(1 - zeta^2);
p2 = -zeta*wn - 1i*wn*sqrt(1 - zeta^2);
p_des = [p1, p2, pc(3, 1), pc(1,1)]
K = place(A,B,p_des)


%C_bar = [5 1 3 1];%[5 1 10 2];

D_bar = 0;


% b) Show the locus plot

C_bar = [10 2 10 1];

s = tf('s');

[num, den] = ss2tf(A,B,C_bar, D, 1)

sys_pos_ss = ss(A, B, C_bar, D);
G_pos = tf(sys_pos_ss);
G_pos_min = minreal(G_pos);

A_neg      = -A;
sys_neg_ss = ss(A_neg, B, C_bar, D);
G_neg      = tf(sys_neg_ss);
G_neg_min  = minreal(G_neg);

sysGG = G_neg*G_pos;

rho = 10; % 0.1, 1, 10, 100
   
all_roots = rlocus(sysGG, rho);
neg_roots = all_roots(all_roots<=0);

figure;
rlocus(sysGG);
axis([-20 20 -20 20])
    
Q = rho*transpose(C_bar)*C_bar;
     
%K = lqr(A,B,Q, 1)
%eig(A - B*K)



%sys_cl = ss(A-B*K, B, C, D);
%step(sys_cl)
