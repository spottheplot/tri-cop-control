%% This script makes the set-up of the Neural networks

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%  Crosssed Chanels Neural networks paramrters %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 4DOF
% All chanles are mixed here. Only a single NN
%   inputs: 30
%   Hidden layer neurons=50. Activation fun: sigmoidal
%   Output: 4:
V_4DOF_last = 0.05*(2*rand(50,30)-1);
W_4DOF_last = 0.05*(2*rand(4,51)-1);

% 9DOF
% All chanles are mixed here. Only a single NN
%   inputs: 38
%   Hidden layer neurons=50. Activation fun: sigmoidal
%   Output: 6:
V_9DOF_last = 0.005*(2*rand(50,38)-1);
W_9DOF_last = 0.005*(2*rand(6,51)-1);
