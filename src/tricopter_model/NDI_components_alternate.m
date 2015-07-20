%% In this file, I'm going to try to do the same as in NDI_components but using only sym variables

clear, clc

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
  
% Body to arm matrixes Varm = R_B2Arm * Vbody
R_BtoArm1 = subs (Ry, ang, gamma) * subs(Rx, ang, eta); % Rotation matrix Body2Arm1
R_BtoArm23 = subs (Rx, ang, gamma) * subs(Ry, ang, eta); % Rotation matrix Body2Arm1

%% Tricopter Forces/Moments(x) 9DOF
% To apply NDI to nonlinear systems
% x_dot = f(x) + g(x,u)
% Calculate g(x,u) --> gx

syms Kt Kq
% Kt = 15; % Thrust = Kt * tau || TODO: NOT REAL VALUE UPDATE
% Kq = 0.07; % Moment = Kq * tau || TODO: NOT REAL VALUE UPDATE

Tarm = [0 0 -Kt * tau].'; % Local arm thrust vector
Marm = [0 0 -Kq * tau].'; % Local arm moment vector for CW blade rotation

% Force and moment contributions from arms in body axis
syms tau1 tau2 tau3 % Throttle for each input
syms eta1 eta2 eta3 gamma1 gamma2 gamma3 % eta: arm roll angle || gamma: arm pitch angle
rot_sign1 = 1; rot_sign2 = -1; rot_sign3 = -1; % CW = 1 || CCW = -1
% TODO rot_sign should be defined in a header or init file

Tbody_ext = subs(R_BtoArm1.',  [gamma, eta], [gamma1, eta1]) * subs(Tarm, tau, tau1) +... 
            subs(R_BtoArm23.', [gamma, eta], [gamma2, eta2]) * subs(Tarm, tau, tau2) +...
            subs(R_BtoArm23.', [gamma, eta], [gamma3, eta3]) * subs(Tarm, tau, tau3);
Mbody_ext = subs(R_BtoArm1.',  [gamma, eta], [gamma1, eta1]) * rot_sign1 * subs(Marm, tau, tau1) +...
            subs(R_BtoArm23.', [gamma, eta], [gamma2, eta2]) * rot_sign2 * subs(Marm, tau, tau2) +...
            subs(R_BtoArm23.', [gamma, eta], [gamma3, eta3]) * rot_sign3 * subs(Marm, tau, tau3);

gx = [Tbody_ext; Mbody_ext];

% Approximate gx = g(x,u) as
% g(x,u) ~= g(x,u0) + jac(g(x,u0), u) * (u - u0)
% g(x,u) ~= h(x,u0) + jac(g(x,u0), u) * u
J_gx = jacobian(gx, [tau1, tau2, tau3, gamma1, gamma2, gamma3, eta1, eta2, eta3]);

% Previus step / initial inputs - Needed for first order approximation
%    [tau1, tau2, tau3, gamma1, gamma2, gamma3, eta1, eta2, eta3]
syms tau1_0 tau2_0 tau3_0 gamma1_0 gamma2_0 gamma3_0 eta1_0 eta2_0 eta3_0
u0 = [tau1_0, tau2_0, tau3_0, gamma1_0, gamma2_0, gamma3_0, eta1_0, eta2_0, eta3_0].';
J_gx0 = subs(J_gx, [tau1, tau2, tau3, gamma1, gamma2, gamma3, eta1, eta2, eta3], u0.');
gx_u0 = subs(gx, [tau1, tau2, tau3, gamma1, gamma2, gamma3, eta1, eta2, eta3], u0.');

hx_u0 = gx_u0 - J_gx0 * u0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Eq of motion (General)
accBody = [Ud Vd Wd].';
vBody = [U V W].';
omega = [P Q R].';
omega_dot = [Pd Qd Rd].';
Fg = [Gx Gy Gz].'; % Gravity force in body axis
CG = [cx cy cz].'; % Coordinates of CG from BAC
%% Forces
% Mass * (AccBody) = Fa + Fg - omega_dot x CG - omega x omega x CG)
% AccBody = (Fg + [Fa]x + cross(omega_dot, CG) + cross(omega,cross(omega,CG))) / mass + [Fa]u * U / mass
% Fa = Fa_x + Fa_u * U
Fa_x = hx_u0(1:3);
F_x = Fg + Fa_x + cross(omega_dot, CG) + cross(omega,cross(omega,CG)) / mass;

% force_eq = accBody == (Fg + cross(omega_dot, CG) + cross(omega,cross(omega,CG))) / mass + Fa / mass;


%% Moments
% I_BAC * omega_dot + omega x (I_BAC * omega) = Ma + (Mc)u * U
%       Where I_BAC - Inertia moment about BAC
%             omega, omega_dot - Angular speed, acc
%             Ma - External moments due to the system state and perturbations ie. Gravity, Wind, etc...
%             (Mc)u * U - External moments created by actuators
% omegaB_dot  = I_BAC \ (Ma - omegaB x (I_BAC * omegaB)) + (I_BAC \ (Mc)u) * U
I_BAC = [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];
Ma_gravity = cross([Gx Gy Gz], [cx cy cz]).';
Ma_control = Mbody_ext;
Ma = Ma_gravity + Ma_control;
% moment_equation = omega_dot == I_BAC \ (Ma - cross(omega, (I_BAC * omega)));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



