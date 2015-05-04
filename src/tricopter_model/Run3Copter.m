%Run the model of the 3copter
clear all
close all
clc
global throtle_1 throtle_23 eta_23
addpath(genpath('Used Functions'))
T_sim=0.01;
%% Load data
SetupComplete;
Throttle_Hover
Initial_Omegas = [1;1;1]*Omega_hover;
Initial_Torques = [1;1;1]*Torque_hover;

close all
% return
%% Find trim point
% Cost evaluation: 1 time
throtle_1= 0.512; % Those are suposed to be the trimed state ones
eta_23 = deg2rad(-1.42);
throtle_23 = throtle_1/cos(eta_23);
% % Trim_Cost = TrimCostFunction(100*[throtle_1,throtle_23,eta_23])

return
% % % % % % Optimization
% % % % % options=optimset('Display','iter','LargeScale','off','TolFun',10e-9,'TolX',10e-11,'MaxFunEvals',6000,'MaxIter',6000);
% % % % % 
% % % % % [X,fval]= fminunc( @(X) TrimCostFunction(X), 100*[throtle_1,throtle_23,eta_23] )
% % % % % 
% % % % % throtle_1=X(1)/100;
% % % % % throtle_23=X(2)/100;
% % % % % eta_23=X(3)/100;
% % % % % 
% % % % % Cost = TrimCostFunction(100*[throtle_1,throtle_23,eta_23])
