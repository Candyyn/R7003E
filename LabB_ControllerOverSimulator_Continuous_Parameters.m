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

close all;
clear all;
clc;

%K = place(A, B, pc_des);
% K = [-449.2475 -112.4387 -178.4821  -22.2137] %<- poles placement%
%K = [-20      -47.207      -72.394      -10.985]%[ -5      -43.831      -64.573      -10.432]
%K = [-449.2475 -112.4387 -178.4821  -22.2137]
%K = [ -10.0000  -57.4908 -105.0371  -19.5009 ]
K = [-3841.49 -737.34 -1425.32 -169.65]


