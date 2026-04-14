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
display("4.6.1")


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

rho = 10; % 0.1, 1, 10, 100
   
all_roots = rlocus(sysGG, rho);
neg_roots = all_roots(all_roots<=0);

    
Q = rho*transpose(C_bar)*C_bar;
     
K = lqr(A,B,Q, 1)
dis_poles = eig(A - B*K)

%% 4.8.1 Observer

% Reduced order state Luenberger observer
scaling = 4;

C = [1 0 0 0;
     0 0 1 0];

poles = dis_poles .* scaling


display("HEREO");
display(poles)


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
Ld = (place(Ad', Cd',exp(scaling*dis_poles*fSamplingPeriod))')