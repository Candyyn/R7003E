%LabB_ObserverAndControllerOverSimulator_Continous_Parameters:

close all;
clear all;
clc;

kP = -404.3247;
kI = -2.2604e+03;
kD = -0.8411;

% Same parameter block as in Lab A
g   = 9.8;
m_b = 0.381;  l_b = 0.112;  I_b = 0.00616;
m_w = 0.036;  l_w = 0.021;  I_w = 0.00000746;
R_m = 4.4;    K_e = 0.444;  K_t = 0.470;

gamma_11 = l_w*(m_w + m_b) + (I_w/l_w);
gamma_12 = l_w*l_b*m_b;
gamma_21 = m_b*l_b;
gamma_22 = I_b+m_b*l_b^2;

alpha_11 = 0;
alpha_12 = -((K_t*K_e)/(R_m*l_w));
alpha_13 = 0;
alpha_14 = ((K_t*K_e)/R_m);
alpha_21 = 0;
alpha_22 = (K_t*K_e)/(R_m*l_w);
alpha_23 = m_b*l_b*g;
alpha_24 = -((K_t*K_e)/R_m);

beta_11 = K_t/R_m;
beta_21 = -K_t/R_m;

gamma = [gamma_11, gamma_12; gamma_21, gamma_22];
alpha = [alpha_11, alpha_12, alpha_13, alpha_14;
         alpha_21, alpha_22, alpha_23, alpha_24];
beta  = [beta_11; beta_21];

A_small = gamma\alpha;
B_small = gamma\beta;

A = [0 1 0 0;
     A_small(1,:);
     0 0 0 1;
     A_small(2,:)];
B = [0; B_small(1); 0; B_small(2)];

C = [0 0 1 0];
D = zeros(2,1);

%task4.5.1
O = obsv(A,C);
rankO = rank(O);

Ctl = ctrb(A, B);
rankC = rank(Ctl);

%task4.6.1
pc = [-475.0690; -5.6571; -3.6; -43.2];
K2 = place(A,B,pc)

%4.7.1
C_bar = [20,1,25,1];

s = tf('s');

G_pos = (-90.03*s)/((s+475)*(s+5.65)*(s-5.72));
G_neg = (-90.03*-s)/((-s+475)*(-s+5.65)*(-s-5.72));
        
sysGG = G_neg*G_pos;
        
rlocus(sysGG)
     
rho = 1; % 0.1, 1, 10, 100
   
all_roots = rlocus(sysGG, rho);
neg_roots = all_roots(all_roots<=0);

    
Q = rho*transpose(C_bar)*C_bar;
     
K = lqr(A,B,Q, 1)




%task4.8.1
% Luenburger estimator 
C_481 = [1 0 0 0;
         0 0 1 0];

pe = [-500, -10, -7, -80];

A

L = place(A',C_481',pe)'

% L =

%         11.23         -3.11
%       -570.94       2123.10
%         -5.26        110.76
%       2498.95      -8612.23

% Reduced Luenburger estimator
C =[1 0 0 0;
    0 0 1 0];
T= eye(4);
T_inv = inv(T);
A_tilde = T*A*T_inv;
B_tilde = T_inv*B;
C_tilde = C * T_inv;

A_yy = A_tilde(1,1);
A_yx = A_tilde(1,2:4);
A_xy = A_tilde(2:4,1);
A_xx = A_tilde(2:4,2:4);

B_y = B_tilde(1);
B_x = B_tilde(2:4);

C_tilde_y = C_tilde(2,1);
C_tilde_x = C_tilde(2,2:4);

CC = [A_yx; C_tilde_x];

p_red = [-5.6571; -3.6; -43.2];
L_red = (place(A_xx', CC', p_red))';

M1 = A_xx-L_red(:,1)*A_yx-L_red(:,2)*C_tilde_x;
M2 = B_x-L_red(:,1)*B_y;
M3 = A_xy-L_red(:,1)*A_yy-L_red(:,2)*C_tilde_y;
M4 = L_red(:,2);
M5 = L_red(:,1);
M6 = T(:,1);
M7 = T(:,2:4);



% 4.9.1
% Decretize system
Ts = 0.183;
sysc = ss(A,B,C_481,D);

sysd = c2d(sysc,Ts);

A_d = sysd.A;
B_d = sysd.B;
C_d = sysd.C;
D_d = sysd.D;

% Find K_d, L_d, M_d1-M_d7
% convert poles to Z domain using Z_p = exp(p * Ts)
poles_d = exp(pc * Ts);
pe_d = exp(pe * Ts);
pe_red_d = exp(p_red * Ts);

% find new controller and estimation values.
K_d = place(A_d,B_d,poles_d)
L_d = place(A_d',C_d',pe_d)'

% reduced order 
A_tilde_d = T*A_d*T_inv;
B_tilde_d = T_inv*B_d;
C_tilde_d = C_d * T_inv;

A_yy_d = A_tilde_d(1,1);
A_yx_d = A_tilde_d(1,2:4);
A_xy_d = A_tilde_d(2:4,1);
A_xx_d = A_tilde_d(2:4,2:4);

B_y_d = B_tilde_d(1);
B_x_d = B_tilde_d(2:4);

C_tilde_y_d = C_tilde_d(2,1);
C_tilde_x_d = C_tilde_d(2,2:4);

CC_d = [A_yx_d; C_tilde_x_d];

L_red_d = (place(A_xx_d', CC_d', pe_red_d))';

M1_d = A_xx_d-L_red_d(:,1)*A_yx_d-L_red_d(:,2)*C_tilde_x_d;
M2_d = B_x_d-L_red_d(:,1)*B_y_d;
M3_d = A_xy_d-L_red_d(:,1)*A_yy_d-L_red_d(:,2)*C_tilde_y_d;
M4_d = L_red_d(:,2);
M5_d = L_red_d(:,1);
M6_d = T(:,1);
M7_d = T(:,2:4);

% test for stability
test1 = eig(A_d-B_d*K_d);
test2 = eig(A_d-L_d*C_d);
