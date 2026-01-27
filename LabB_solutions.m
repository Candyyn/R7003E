LabA_solutions;

%% 4.5 -- Check the controllability and observability properties of the linearized system
disp("Check the controllability and observability properties of the linearized system")
format bank;
O = obsv(A, C) % At this state, our obserablivity is not full, due to rankO = 3 where n = 4
rankO = rank(O)
C

% This will make it so there is a state/mode in the system that cant be
% seen or detected from the output 

% ctrb - controllability.
ctr = ctrb(A, B) % Why A and B? prob due to First and Second part of the entire system
rankC = rank(ctr) % Our RankC and Ctrl n is equal. Which makes our system "controllable?"



%% test 2 4.6.1
currPoles = pole(G);
dis_poles = [currPoles(1), currPoles(3), currPoles(3)-0.01, currPoles(3)-0.02]
%K = acker(A, B, pc)
K = place(A, B, dis_poles)


zeta = 0.8;
wn = 7;
p1 = -zeta*wn + 1i*wn*sqrt(1 - zeta^2);
p2 = -zeta*wn - 1i*wn*sqrt(1 - zeta^2);
dis_poles = [p1, p2, pc(3, 1), pc(1,1)]
K = acker(A,B,dis_poles)



%% test 4.6.1

%risetime = 0.55;
%w = 1.8 / risetime;
%e = 0.707;
%overshoot = exp(-pi*e/(sqrt(1-e^2)));
%dom_sys = w^2/(s^2+2*e*w*s+w^2);
%step(dom_sys);
%dom_sys_pole = pole(dom_sys);

%currPoles = pole(G);
%dis_poles = [currPoles(1), currPoles(3), currPoles(3)-0.01, currPoles(3)-0.02]

%% 4.6.1
oldPC = pc;

etha = 0.7;
p_slow = pc(3,1);

w_n = (abs(p_slow) / etha); % Our Real del

w_d = w_n * sqrt(1 - etha^2); % Imaginar
p_dom1 = -etha * w_n + 1j * w_d;
p_dom2 = -etha * w_n - 1j * w_d;

p_fast = [pc(1,1); pc(3,1)];

p_cl = [p_dom1; p_fast; p_dom2];
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
%p_cl;
%p_cl = [-4, -475, -4, -4];
%K = acker(A, B, dis_poles)
%K = place(A, B, p_cl) % First way
% -574.8968 -296.9783 -413.5632  -68.4583


%% 4.7.1

% a) Choose C

C_bar = [5 1 10 2]%[5 1 10 2];
D_bar = 0;


% b) Show the locus plot

C_bar = [10 2 10 1];

s = tf('s');
sys_pos_ss = ss(A, B, C, D_bar);
G_pos = tf(sys_pos_ss);
G_pos_min = minreal(G_pos);

[num_pos, den_pos] = tfdata(G_pos_min, 'v');

A_neg      = -A;
sys_neg_ss = ss(A_neg, B, C, 0);
G_neg      = tf(sys_neg_ss);
G_neg_min  = minreal(G_neg);

sysGG = G_neg*G_pos;

rho = 7; % 0.1, 1, 10, 100
   
all_roots = rlocus(sysGG, rho);
neg_roots = all_roots(all_roots<=0);

    
Q = rho*transpose(C_bar)*C_bar;
     
K = lqr(A,B,Q, 1)
dis_poles = eig(A - B*K)

%% 4.8.1 Observer

% Full order Luenberger observer 

% C_luen = [1 0 0 0;
%           0 0 1 0];
% %O=[C_luen; C_luen*A; C_luen*A^2; C_luen*A^3]; ran = rank(O)
% 
% obs_poles = dis_poles * 4
% L = (place(A', C_luen', obs_poles))'
% O=[C; C*A; C*A^2; C*A^3];
% rank(O)
% 
% % Reduced order state Luenberger observer
% 
% C_acc=[1 0 0 0];
% C_nacc=[0 0 1 0];
% 
% 
% %C_acc = [
% %    1 0 0 0   % theta_b 0 0 1 0   % x_w
% %];
% 
% V = null(C_acc);  
% T_inv = [C_acc; V'];
% T = inv(T_inv);
% 
% %T_inv=[C_acc;
% %     0 1 0 0; 0 0 1 0; 0 0 0 1];
% %T=inv(T_inv);
% 
% A_tilde=T_inv*A*T;
% B_tilde=T_inv*B;
% 
% C_acc_tilde = C_acc*T;
% C_nacc_tilde = C_nacc * T;
% 
% A_yy=A_tilde(1,1);
% A_yx=A_tilde(1,2:4);
% A_xy=A_tilde(2:4,1);
% A_xx=A_tilde(2:4,2:4);
% 
% C_tilde_y = C_nacc_tilde(1,1);
% C_tilde_chi = C_nacc_tilde(1,2:4);
% 
% B_tilde_y = B_tilde(1,1);
% B_tilde_chi = B_tilde(2:4,1);
% CC = [A_yx; C_tilde_chi];
% 
% 
% L_redu = (place(A_xx', CC',obs_poles(2:4, 1)))'
% 
% 
% L_acc = L_redu(1:3,1);
% L_nacc= L_redu(1:3,2);
% 
% 
% M1 = A_xx - L_acc*A_yx-L_nacc*C_tilde_chi
% M2 = B_tilde_chi - L_acc *B_tilde_y
% M3 = (A_xy - L_acc*A_yy - L_nacc*C_tilde_y)
% M4 = L_nacc
% M5 = L_acc
% M6 = T(1:4,1)
% M7 = T(1:4,2:4)

% Reduced order state Luenberger observer
scaling = 3;

C = [1 0 0 0;
     0 0 1 0];

poles = dis_poles .* scaling;

O=[C; C*A; C*A^2; C*A^3];
ran = rank(O)


C_acc=[1 0 0 0];
C_nacc=[0 0 1 0];

T_inv=[C_acc;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
T=inv(T_inv);

A_tilde=T_inv*A*T;
B_tilde=T_inv*B;

C_acc_tilde = C_acc*T;
C_nacc_tilde = C_nacc * T;

A_yy=A_tilde(1,1);
A_yx=A_tilde(1,2:4);
A_xy=A_tilde(2:4,1);
A_xx=A_tilde(2:4,2:4);

C_tilde_y = C_nacc_tilde(1,1);
C_tilde_chi = C_nacc_tilde(1,2:4);

B_tilde_y = B_tilde(1,1);
B_tilde_chi = B_tilde(2:4,1);
CC = [A_yx; C_tilde_chi];
L = (place(A_xx', CC',3*dis_poles(2:4, 1)))'


L_acc = L(1:3,1);
L_nacc= L(1:3,2);


M1 = A_xx - L_acc*A_yx-L_nacc*C_tilde_chi
M2 = B_tilde_chi - L_acc *B_tilde_y
M3 = (A_xy - L_acc*A_yy - L_nacc*C_tilde_y)
M4 = L_nacc
M5 = L_acc
M6 = T(1:4,1)
M7 = T(1:4,2:4)


% Full order Luenberger observer 
C_est = [1 0 0 0 ; 
         0 0 1 0];

obs_poles = dis_poles * scaling
L = (place(A', C', obs_poles))'

%% 4.9.1 Descrete

freq = 200;
fSamplingPeriod = 1/freq;
SamplingPeriod = 0.0219;
% Convert our system from continues to descrete
system_d = c2d(ss(A,B,C,D), fSamplingPeriod, 'zoh'); 

Ad = system_d.A
Bd = system_d.B
Cd = system_d.C
Dd = system_d.D

% Map to z 
p_controller = dis_poles
z_controller = exp(p_controller * fSamplingPeriod)

abs(z_controller) % Should be < 1


% Discrete Full luenberger

C_luen = [1 0 0 0;
          0 0 1 0];
%O=[C_luen; C_luen*Ad; C_luen*Ad^2; C_luen*Ad^3];
%ran = rank(O); % Checks

C_bar = [10 2 10 1];
Q=rho*(C_bar')*C_bar
[Kd, P_dlqr, e_dlqr]=dlqr(Ad,Bd,Q,1);
Dd = [
     0
     0
 ]

Cd_acc=[1 0 0 0];
Cd_nacc=[0 0 1 0];

Td_inv=[Cd_acc;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
Td=inv(Td_inv);

Ad_tilde=Td_inv*Ad*Td;
Bd_tilde=Td_inv*Bd;

Cd_acc_tilde = Cd_acc*Td;
Cd_nacc_tilde = Cd_nacc * Td;

Ad_yy=Ad_tilde(1,1);
Ad_yx=Ad_tilde(1,2:4);
Ad_xy=Ad_tilde(2:4,1);
Ad_xx=Ad_tilde(2:4,2:4);

Cd_tilde_y = Cd_nacc_tilde(1,1);
Cd_tilde_chi = Cd_nacc_tilde(1,2:4);

Bd_tilde_y = Bd_tilde(1,1);
Bd_tilde_chi = Bd_tilde(2:4,1);
CCd = [Ad_yx; Cd_tilde_chi];
Ld = (place(Ad_xx', CCd',scaling*dis_poles(2:4, 1)))'
Ld = (place(Ad', Cd',exp(dis_poles*fSamplingPeriod))')



Ld_acc = Ld(1:3,1);
Ld_nacc= Ld(1:3,2);


Md1 = Ad_xx - Ld_acc*Ad_yx-Ld_nacc*Cd_tilde_chi
Md2 = Bd_tilde_chi - Ld_acc *Bd_tilde_y
Md3 = (Ad_xy - Ld_acc*Ad_yy - Ld_nacc*Cd_tilde_y)
Md4 = Ld_nacc
Md5 = Ld_acc
Md6 = Td(1:4,1)
Md7 = Td(1:4,2:4)


% Full
Ld = (place(Ad', Cd',exp(dis_poles*fSamplingPeriod))')


% obs_poles = dis_poles * 2
% Ld = (place(Ad', C_luen', obs_poles))'
% 
% 
% Cd_acc=[1 0 0 0];
% Cd_nacc=[0 0 1 0];
% 
% Td_inv=[Cd_acc;
%      0 1 0 0;
%      0 0 1 0;
%      0 0 0 1];
% Td=inv(Td_inv);
% 
% Ad_tilde=Td_inv*Ad*Td;
% Bd_tilde=Td_inv*Bd;
% 
% Cd_acc_tilde = Cd_acc*Td;
% Cd_nacc_tilde = Cd_nacc * Td;
% 
% Ad_yy=Ad_tilde(1,1);
% Ad_yx=Ad_tilde(1,2:4);
% Ad_xy=Ad_tilde(2:4,1);
% Ad_xx=Ad_tilde(2:4,2:4);
% 
% Cd_tilde_y = Cd_nacc_tilde(1,1);
% Cd_tilde_chi = Cd_nacc_tilde(1,2:4);
% 
% Bd_tilde_y = Bd_tilde(1,1);
% Bd_tilde_chi = Bd_tilde(2:4,1);
% CCd = [Ad_yx; Cd_tilde_chi];
% Ld_redu = (place(Ad_xx', CCd',3*dis_poles(2:4, 1)))'
% 
% 
% Ld_acc = L_redu(1:3,1);
% Ld_nacc= L_redu(1:3,2);
% 
% 
% Md1 = Ad_xx - Ld_acc*Ad_yx-Ld_nacc*Cd_tilde_chi
% Md2 = Bd_tilde_chi - Ld_acc *Bd_tilde_y
% Md3 = (Ad_xy - Ld_acc*Ad_yy - Ld_nacc*Cd_tilde_y)
% Md4 = Ld_nacc
% Md5 = Ld_acc
% Md6 = Td(1:4,1)
% Md7 = Td(1:4,2:4)



%% 4.8.1 Full order 
% 
% C_luen = [1 0 0 0;
%           0 0 1 0];
% 
% p_test = oldPC(oldPC < 0); % only negative real poles
% P_o = 6 * real(p_cl);
% P_o = [P_o(1,1), P_o(2,1), P_o(3,1), -30] % add a random extra 
% %P_o
% 
% 
% %speed_factor = 5;
% %P_o = speed_factor * real(lambda_cl) ...
% %      + 1i*speed_factor * imag(lambda_cl);    % scale both real and imag parts
% %P_o = P_o(:); 
% 
% 
% 
% %Our P_o are WAY TO FAST we need to change that.
% %p_neg  = oldPC(oldPC < 0);         % [-475.08; -90.00; -5.66]
% 
% % Dominant (slowest) closed-loop pole
% %p_slow = p_neg(end);               % -5.66
% 
% % Choose observer poles as 3–6 times faster than p_slow
% %scale = [3; 4; 5; 6];              % multipliers
% 
% %P_o = scale * p_slow;
% 
% %P = [oldPC(1, 1), oldPC(3, 1), oldPC(3, 1), oldPC(3, 1)];
% 
% % 
% % P_o = [
% %         P(1) * 4; 
% %         P(2) * 4; 
% %         P(3) * 4; 
% %         P(4) * 4]
% 
% L = (place(A', C_luen', dis_poles * 5)).'
% 
% %% 4.8.1  redued
% 
% V = [0 1 0 0; 0 0 1 0; 0, 0, 0, 1]
% C_notacc = C_luen(2,:);
% C_acc = C_luen(1,:);
% T_inv = [C_luen(1,:) ; V]
% T = inv(T_inv)
% 
% A_tilde = T_inv * A * T;
% B_tilde = T_inv * B;
% 
% 
%  C_acc_tilde = C_acc * T;
%  C_notacc_tilde = C_notacc * T;
% 
%  n = 4;
%  m = 1;
% 
%  A_tilde_yy = A_tilde(1:m, 1:m);
%  A_tilde_yx = A_tilde(1:m, 1+m:n);
%  A_tilde_xy = A_tilde(1+m:n, 1:m);
%  A_tilde_xx = A_tilde(1+m:n, 1+m:n)
% 
%  B_tilde_y = B_tilde(1:m);
%  B_tilde_x = B_tilde(1+m:n);
% 
%  C_tilde_y = C_notacc_tilde(1:m);
%  C_tilde_x = C_notacc_tilde(1+m:n);
% % y_acc = C(1,;) * x
% % x_hat = T(:,1) * y_acc + T(:,2:)
% 
% AA = A_tilde_xx
% CC = [A_tilde_yx; C_tilde_x]
% dis_poles(2:4)
% 
% %L_r = (place(AA', CC', P_o(1+m:n)))';
% L_r = (place(AA', CC', dis_poles(2:4)))';
% L_acc = [L_r(:, 1:m)]
% L_notacc = L_r(:, 1+m:size(L_r, 2));
% 
% %Axx - L_red_acc * Ayx - L_red_not_acc * Cx
% M1 = A_tilde_xx - L_acc * A_tilde_yx - L_notacc * C_tilde_x %
% M2 = B_tilde_x - L_acc * B_tilde_y;
% M3 = A_tilde_xy - L_acc * A_tilde_yy - L_notacc * C_tilde_y;
% M4 = L_notacc; %
% 
% M5 = L_acc; %
% 
% M6 = T(: , 1:m);
% M7 = T(: , 1+m:n);



%% 4.9.1

% freq = 200;
% fSamplingPeriod = 1/freq;
% %fSamplingPeriod = 0.01;
% 
% D = 0;
% 
% system_d = c2d(ss(A,B,C,D), fSamplingPeriod, 'zoh');
% 
% Ad = system_d.A
% Bd = system_d.B
% Cd = system_d.C
% Dd = system_d.D
% 
% 
% 
% 
% % compute the gains Kd, Ld, Md1, . . . , Md7 
% Kd = place(Ad, Bd, exp(dis_poles * fSamplingPeriod))
% %Kd = lqrd(A,B, Q, rho, fSamplingPeriod);
% %Kd = 0.75 * Kd
% 
% % L = (place(A', C', P_o))'
% Cd = [1 0 0 0; 0 0 1 0];
% Ld = place(Ad', Cd', 5*exp(dis_poles * fSamplingPeriod)).'
% 
% V = [Cd(2,:);0, 1, 0, 0; 0, 0, 0, 1]
% C_notacc = Cd(2,:);
% C_acc = Cd(1,:);
% T_inv = [Cd(1,:) ; V];
% T = inv(T_inv);
% 
% A_tilde = T_inv * Ad * T;
% B_tilde = T_inv * Bd;
% 
% 
%  C_acc_tilde = C_acc * T;
%  C_notacc_tilde = C_notacc * T;
% 
%  n = 4;
%  m = 1;
% 
%  A_tilde_yy = A_tilde(1:m, 1:m);
%  A_tilde_yx = A_tilde(1:m, 1+m:n);
%  A_tilde_xy = A_tilde(1+m:n, 1:m);
%  A_tilde_xx = A_tilde(1+m:n, 1+m:n);
% 
%  B_tilde_y = B_tilde(1:m);
%  B_tilde_x = B_tilde(1+m:n);
% 
%  C_tilde_y = C_notacc_tilde(1:m);
%  C_tilde_x = C_notacc_tilde(1+m:n);
% % y_acc = C(1,;) * x
% % x_hat = T(:,1) * y_acc + T(:,2:)
% 
% AA = A_tilde_xx;
% CC = [A_tilde_yx; C_tilde_x];
% 
% %P_o; %TODO: CHECK IF WE NEED TO REPLACE P_o with diff
% 
% %L_r = (place(AA', CC', P_o(1+m:n)))';
% L_r = (place(AA', CC', exp(P_o(1:n-1)*fSamplingPeriod)))';
% L_acc = [L_r(:, 1:m)];
% L_notacc = L_r(:, 1+m:size(L_r, 2));
% 
% Md1 = A_tilde_xx - L_acc * A_tilde_yx - L_notacc * C_tilde_x
% Md2 = B_tilde_x - L_acc * B_tilde_y
% Md3 = A_tilde_xy - L_acc * A_tilde_yy - L_notacc * C_tilde_y
% Md4 = L_notacc
% 
% Md5 = L_acc
% 
% Md6 = T(: , 1:m)
% Md7 = T(: , 1+m:n)