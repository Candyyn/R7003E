LabB_solutions;


%% 5.2.1


freq = 5;

fSamplingPeriod = 1/freq;

system_d = c2d(ss(A,B,C,D), fSamplingPeriod, 'zoh'); 

Ad = system_d.A
Bd = system_d.B
Cd = system_d.C
Dd = system_d.D

p_controller = dis_poles
z_controller = exp(p_controller * fSamplingPeriod)

abs(z_controller) 


C_luen = [1 0 0 0;
          0 0 1 0];
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


%% 5.3.1

freq = 80;

fSamplingPeriod = 1/freq;

sysc = ss(A, B, C, 0);
sysd = c2d(sysc, fSamplingPeriod);

Ad = sysd.A
Bd = sysd.B
Cd = sysd.C
Dd

% Discrete controller gain
z_poles = exp(dis_poles * fSamplingPeriod);
[Kd, P_dlqr, e_dlqr]=dlqr(Ad,Bd,Q,1);
Kd

% Full-order discrete observer
scaling = 4;
obs_poles = dis_poles * scaling;
z_obs_poles = exp(obs_poles * fSamplingPeriod);
Ld = place(Ad', Cd', z_obs_poles)'

% Coordinate transform
C_acc = [1 0 0 0];
C_nacc = [0 0 1 0];

T_inv = [C_acc;
         0 1 0 0;
         C_nacc;
         0 0 0 1];
T = inv(T_inv);

% Transformed discrete matrices
A_tilde = T_inv * Ad * T;
B_tilde = T_inv * Bd;

C_acc_tilde  = C_acc  * T;
C_nacc_tilde = C_nacc * T;

% Partition the system
A_yy = A_tilde(1,1);
A_yx = A_tilde(1,2:4);
A_xy = A_tilde(2:4,1);
A_xx = A_tilde(2:4,2:4);

C_tilde_y   = C_nacc_tilde(1,1);
C_tilde_chi= C_nacc_tilde(1,2:4);

B_tilde_y   = B_tilde(1,1);
B_tilde_chi= B_tilde(2:4,1);

% Reduced observer gain
CC = [A_yx;
      C_tilde_chi];

z_red_obs_poles = exp(3 * dis_poles(2:4) * fSamplingPeriod);

L = place(A_xx', CC', z_red_obs_poles)';

L_acc  = L(:,1);
L_nacc= L(:,2);

Md1 = A_xx - L_acc*A_yx - L_nacc*C_tilde_chi
Md2 = B_tilde_chi - L_acc*B_tilde_y
Md3 = A_xy - L_acc*A_yy - L_nacc*C_tilde_y
Md4 = L_nacc
Md5 = L_acc
Md6 = T(:,1)
Md7 = T(:,2:4)