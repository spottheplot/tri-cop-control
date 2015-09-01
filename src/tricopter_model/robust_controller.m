clear, clc
digits(4)
format short

syms Ixx_b Iyy_b Izz_b Ixy_b Ixz_b Iyz_b % Moments and products of inertia calculated about the BAC
syms Ixx Iyy Izz Ixy Ixz Iyz             % Moments and products of inertia calculated about the CG
syms P Q R Pd Qd Rd U V W Ud Vd Wd
syms m cx cy cz
syms X Y Z L M N
syms Gx Gy Gz Gl Gm Gn
syms mass

%% Tricopter specific variables
syms tau % Throttle for each input
syms eta gamma % eta: arm roll angle || gamma: arm pitch angle

% Rotation matrixes
syms ang
Rx = [1,    0,         0;
      0, cos(ang),   sin(ang);
      0, -sin(ang),  cos(ang)];

Ry = [cos(ang), 0, -sin(ang);
        0,      1,    0;
      sin(ang), 0, cos(ang)];
  
Rz = [cos(ang),  sin(ang), 0;
      -sin(ang), cos(ang), 0;
      0,         0,        1];
% Body to Earth
% syms phi theta psi
% R_E2B = subs(Rx, ang, phi) * subs (Ry, ang, theta) * subs (Rz, ang, psi)
% Body to arm matrixes Varm = R_B2Arm * Vbody
% Euler Rate Body to earth
% syms phi_dot theta_dot psi_dot
% L_E2B = [phi_dot, 0, 0].' +...
%          subs(Rx, ang, phi) * [0, theta_dot, 0].' +...
%          subs(Rx, ang, phi) * subs (Ry, ang, theta) * [0, 0, psi_dot].';
R_BtoArm1 = subs (Ry.', ang, gamma) * subs(Rx.', ang, eta); % Rotation matrix Body2Arm1
R_BtoArm23 = subs (Rx.', ang, gamma) * subs(Ry.', ang, eta); % Rotation matrix Body2Arm1

%% Tricopter Forces/Moments(x) 9DOF
% To apply NDI to nonlinear systems
% x_dot = f(x) + g(x,u)
% Calculate g(x,u) --> gx

syms Kt Kq
% Kt = 25.9; % Thrust = Kt * tau || Previous used value: 15
% Kq = 0.3; % Moment = Kq * tau || Previous used value: 0.07

Tarm = [0 0 -Kt * tau].'; % Local arm thrust vector
Marm = [0 0 -Kq * tau].'; % Local arm moment vector for CW blade rotation

% Force and moment contributions from arms in body axis
syms tau1 tau2 tau3 % Throttle for each input
syms eta1 eta2 eta3 gamma1 gamma2 gamma3 % eta: arm roll angle || gamma: arm pitch angle
rot_sign1 = 1; rot_sign2 = -1; rot_sign3 = -1; % CW = 1 || CCW = -1
% TODO rot_sign should be defined in a header or init file

% Tbody_ext = R_Arm2Body * Tarm
Tarm1_body = subs(R_BtoArm1.',  [gamma, eta], [gamma1, eta1]) * subs(Tarm, tau, tau1);
Tarm2_body = subs(R_BtoArm23.', [gamma, eta], [gamma2, eta2]) * subs(Tarm, tau, tau2);
Tarm3_body = subs(R_BtoArm23.', [gamma, eta], [gamma3, eta3]) * subs(Tarm, tau, tau3);
Tbody_ext = Tarm1_body + Tarm2_body + Tarm3_body;
% External moments about BAC -> Mbody_ext = R_Arm2Body * Marm + r_arm x Tbody_ext
l = 0.3450; % Distance from BAC to arm
% Vector going from BAC to each of the arms.
r_arm1 = l*[1;0;0]; 
r_arm2 = l*[-1/2;sqrt(3)/2;0];
r_arm3 = l*[-1/2;-sqrt(3)/2;0];
Mbody_ext = subs(R_BtoArm1.',  [gamma, eta], [gamma1, eta1]) * rot_sign1 * subs(Marm, tau, tau1) +...
            subs(R_BtoArm23.', [gamma, eta], [gamma2, eta2]) * rot_sign2 * subs(Marm, tau, tau2) +...
            subs(R_BtoArm23.', [gamma, eta], [gamma3, eta3]) * rot_sign3 * subs(Marm, tau, tau3) +...
            cross(r_arm1, Tarm1_body)+...
            cross(r_arm2, Tarm2_body)+...
            cross(r_arm3, Tarm3_body);

gx = [Tbody_ext; Mbody_ext];

% Approximate gx = g(x,u) as
% g(x,u) ~= g(x,u0) + jac(g(x,u), u) * (u - u0)
% g(x,u) ~= h(x,u0) + jac(g(x,u), u) * u
J_gx = jacobian(gx, [tau1, eta1, gamma1, tau2, eta2, gamma2, tau3, eta3, gamma3]);

% Previus step / initial inputs - Needed for first order approximation
%    [tau1, tau2, tau3, gamma1, gamma2, gamma3, eta1, eta2, eta3]
syms tau1_0 tau2_0 tau3_0 gamma1_0 gamma2_0 gamma3_0 eta1_0 eta2_0 eta3_0
u0 = [tau1_0, eta1_0, gamma1_0, tau2_0, eta2_0, gamma2_0, tau3_0, eta3_0, gamma3_0].';
J_gx0 = subs(J_gx, [tau1, eta1, gamma1, tau2, eta2, gamma2, tau3, eta3, gamma3], u0.');
gx_u0 = subs(gx, [tau1, eta1, gamma1, tau2, eta2, gamma2, tau3, eta3, gamma3], u0.');

hx_u0 = gx_u0 - J_gx0 * u0;

%% Eq of motion (General)
% I_BAC * omega_dot + omega x (I_BAC * omega) = Ma + (Mc)u * U
%       Where I_BAC - Inertia moment about BAC
%             omega, omega_dot - Angular speed, acc
%             Ma - External moments due to the system state and perturbations ie. Gravity, Wind, etc...
%             (Mc)u * U - External moments created by actuators
% omega_dot  = I_BAC \ (Ma - omega x (I_BAC * omega)) + (I_BAC \ (Mc)u) * U
%
% Dynamics about BAC with Mass Properties (Moments and Product of Inertia) defined about the Centre of Gravity.
%             Udot Vdot Wdot   Pdot     Qdot     Rdot
mass_matrix= [mass   0    0      0     mass*cz -mass*cy ; % X
                0  mass   0  -mass*cz     0     mass*cx ; % Y
                0    0  mass  mass*cy -mass*cx     0    ; % Z
                0    0    0     Ixx     -Ixy     -Ixz   ; % L
                0    0    0    -Ixy      Iyy     -Iyz   ; % M
                0    0    0    -Ixz     -Iyz      Izz   ];% N       
% *******************************************************                      
excitation_matrix= [Gx + mass*R*V - mass*Q*W + mass*cx*(Q^2+R^2) - mass*cy*P*Q - mass*cz*P*R ; 
                    Gy - mass*R*U + mass*P*W - mass*cx*P*Q + mass*cy*(P^2+R^2) - mass*cz*Q*R ;
                    Gz + mass*Q*U - mass*P*V - mass*cx*P*R - mass*cy*Q*R + mass*cz*(P^2+Q^2) ;
                    - Q*R*(Izz-Iyy) - P*R*Ixy + P*Q*Ixz - (R^2-Q^2)*Iyz;
                    - P*R*(Ixx-Izz) + Q*R*Ixy - P*Q*Iyz - (P^2-R^2)*Ixz;
                    - P*Q*(Iyy-Ixx) - Q*R*Ixz + P*R*Iyz - (Q^2-P^2)*Ixy];
%                 Gravity term is being calculated twice and it cancels
%                 itself.
%                     Gl - Q*R*(Izz-Iyy) - P*R*Ixy + P*Q*Ixz - (R^2-Q^2)*Iyz + cz*Gy - cy*Gz;
%                     Gm - P*R*(Ixx-Izz) + Q*R*Ixy - P*Q*Iyz - (P^2-R^2)*Ixz + cx*Gz - cz*Gx;
%                     Gn - P*Q*(Iyy-Ixx) - Q*R*Ixz + P*R*Iyz - (Q^2-P^2)*Ixy + cy*Gx - cx*Gy];

% % Mass Properties (Moments and Product of Inertia) are defined about the Body Axis Centre.
% %             Udot      Vdot      Wdot       Pdot        Qdot        Rdot
% mass_matrix= [mass      0         0          0           mass*cz    -mass*cy ; % X
%               0         mass      0         -mass*cz     0           mass*cx ; % Y
%               0         0         mass       mass*cy    -mass*cx     0       ; % Z
%               0        -mass*cz   mass*cy    Ixx        -Ixy        -Ixz     ; % L
%               mass*cz   0        -mass*cx   -Ixy         Iyy        -Iyz     ; % M
%              -mass*cy   mass*cx   0         -Ixz        -Iyz         Izz    ]; % N
% % *******************************************************                      
% excitation_matrix= [Gx + mass*R*V - mass*Q*W + mass*cx*(Q^2+R^2) - mass*cy*P*Q       - mass*cz*P*R ; 
%                     Gy - mass*R*U + mass*P*W - mass*cx*P*Q       + mass*cy*(P^2+R^2) - mass*cz*Q*R ;
%                     Gz + mass*Q*U - mass*P*V - mass*cx*P*R       - mass*cy*Q*R       + mass*cz*(P^2+Q^2) ;
%                     -cz*Gy + cy*Gz - Q*R*(Izz-Iyy) - P*R*Ixy + P*Q*Ixz - (R^2-Q^2)*Iyz                     - (P*V-Q*U)*mass*cy + (R*U-P*W)*mass*cz;
%                     -cx*Gz + cz*Gx - P*R*(Ixx-Izz) + Q*R*Ixy - P*Q*Iyz - (P^2-R^2)*Ixz + (P*V-Q*U)*mass*cx                     - (Q*W-R*V)*mass*cy;
%                     -cy*Gx + cx*Gy - P*Q*(Iyy-Ixx) - Q*R*Ixz + P*R*Iyz - (Q^2-P^2)*Ixy - (R*U-P*W)*mass*cx + (Q*W-R*V)*mass*cy                   ];
                                                                    
% *******************************************************
 % Applies "cost" for moving forces applied on CG to BAC by generating the
 % associated moment.
Mc = [1,	0,      0,      0,  0,  0;
      0,	1,      0,      0,  0,  0;
      0,	0,      1,      0,  0,  0;
      0,	cz,     -cy,    1,  0,  0;
      -cz,	0,      cx,     0,  1,  0;
      cy,	-cx,	0,      0,  0,  1];

% Actual Eq of motion (Not needed for NDI)
% TODO 
% [Ud, Vd, Wd, Pd, Qd, Rd] = mass_matrix\(excitation_matrix + Mc * [Tbody_ext; Mbody_ext])
% vpa(eval(mass_matrix\(excitation_matrix + Mc * [Tbody_ext; Mbody_ext])), 5);
  
% %% NDI Components from Eqs of motion
% fx = mass_matrix\(excitation_matrix + Mc * hx_u0);
% J_fx = jacobian(fx, [U,V,W,P,Q,R]);
% 
% % g(x) = Bv*B
% Bv =  mass_matrix\Mc;
% B = J_gx0;
% gx_hat = Bv*B;

%% Linear model dot(x) = A(x-x_0) + B(u-u_0) = Ax + Bu
FX = mass_matrix\(excitation_matrix + Mc * gx);
% A = d(FX)/F(X)|x_0
J_FX = jacobian(FX, [U,V,W,P,Q,R]);
A = subs(J_FX, [U,V,W,P,Q,R], [0,0,0,0,0,0]);
A = subs(J_FX, [U,V,W,P,Q,R], [0.1,0.1,0.1,0.1*pi/180,0.1*pi/180,0.1*pi/180]);
% B = d(FX)/F(U)|U_0
J_FU = jacobian(FX, [tau1, eta1, gamma1, tau2, eta2, gamma2, tau3, eta3, gamma3]);

% Calculated /simulink_functions/mass_properties with parameters
% commented above
I_BAC_init =...
   [ 0.1602  0      -0.0080 ;
     0       0.2042  0      ;
    -0.0080  0       0.3441];
CG_init = [0         0   -0.0015];
mass = 3.51;
I_CG_init = I_BAC_init - mass * (CG_init * CG_init' * eye(3) - CG_init' * CG_init);
Kt = 19.5; % Thrust = Kt * tau || Previous used value: 15
Kq = 0.26; % Moment = Kq * tau || Previous used value: 0.07

%% 4DOF
A_4DOF = A([3,4,5,6],[3,4,5,6]);
% Trim input for 4 DOF [tau1, eta1, tau2, tau3] =
% [0.59,-2.298*pi/180,0.5889,0.5896]
B = subs(J_FU, [tau1, eta1, gamma1, tau2, eta2, gamma2, tau3, eta3, gamma3], [0.59 ,-2.298*pi/180 ,0, 0.5889,0,0,0.5896 ,0,0]);
B = subs(B, [U,V,W,P,Q,R], [0,0,0,0,0,0]);
B_4DOF = B([3,4,5,6],[1,2,4,7]);

% 4DOF no massic update
gamma1 = 0;
gamma1_0 = 0;
gamma2 = 0;
gamma2_0 = 0;
gamma3 = 0;
gamma3_0 = 0;
eta2 = 0;
eta2_0 = 0;
eta3 = 0;
eta3_0 = 0;
Ixx = I_CG_init(1,1);
Iyy = I_CG_init(2,2);
Izz = I_CG_init(3,3);
Ixy = I_CG_init(1,2);
Ixz = I_CG_init(1,3);
Iyz = I_CG_init(2,3);
cx = CG_init(1);
cy = CG_init(2);
cz = CG_init(3);
% Check for controllability
A_4DOF = double(eval(A_4DOF));
B_4DOF = double(eval(vpa(B_4DOF,5)));

ctrb(double(A_4DOF),double(B_4DOF));

% C and D matrices definition
C_4DOF = eye(4);
D_4DOF = zeros(4,4);

LM_4DOF = ss(A_4DOF,B_4DOF,C_4DOF,D_4DOF);

% %% Inner LQR w p q r
% % Define R and Q matrices for LQR
% % R = diag(1./[1,2*pi*325/360,1,1]);
% % Initial guess
% % R = eye(4);
% % Q = 10*R;
% % K=lqr(A_4DOF, B_4DOF, Q, R);
% % sys_cl = ss(A_4DOF-B_4DOF*K, B_4DOF, C_4DOF, D_4DOF);
% % step(0.2*sys_cl)

% LQI
% R = eye(4);
% Q = blkdiag(10*eye(4),eye(4));
% sys = ss(A_4DOF, B_4DOF, C, D);
% Ki = lqi(sys, Q,R); % Ki = [K, Ki]

% Augmented model with x = [w,p,q,r,z,phi,theta,psi]
A_aug = [A_4DOF, zeros(size(A_4DOF));
    eye(size(A_4DOF)),zeros(size(A_4DOF))];
B_aug = [B_4DOF; zeros(size(B_4DOF))];
C_aug = [zeros(size(A_4DOF)), eye(size(A_4DOF))];
D_aug = zeros(4);
R = eye(4);
Q = blkdiag(10*eye(4),eye(4),eye(4));
LM_4DOF_aug = ss(A_aug, B_aug, C_aug, D_aug);
Ki = lqi(LM_4DOF_aug, Q,R);