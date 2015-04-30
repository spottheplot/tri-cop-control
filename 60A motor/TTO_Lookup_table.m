%% 
clear all 
clc
close all

load('TTO_raw_data.mat')
% % 
% % plot3(throtle,Mom,Omega,'*')
%% Interpolate
fitt=TTO_data_fitV2(throtle(throtle<=18), Mom(throtle<=18), Omega(throtle<=18))

% % % % Manual interpolation
% % % a=0;
% % % b=-0;
% % % c=0.0;
% % % Cost_fit(throtle(throtle<=16),Mom(throtle<=16),Omega(throtle<=16),a,b,c) % Throle less than 16 (Saturation)
% % % 
% % % XX = fminunc( @(XX) Cost_fit(throtle(throtle<=16),Mom(throtle<=16),Omega(throtle<=16),XX(1),XX(2),XX(3)) , [a,b,c])
% % % a=XX(1);
% % % b=XX(2);
% % % c=XX(3);
% return
%% Extrapolate
Throtle_TTO=repmat(0:0.5:20,size(0:0.05:1,2),1);
Torque_TTO=repmat(0:0.05:1,size(0:0.5:20,2),1)';
for i=1:size(Throtle_TTO,1)
    for j=1:size(Throtle_TTO,2)
        Omega_TTO(i,j) = max(0, min( fitt(Throtle_TTO(i,j),Torque_TTO(i,j)),fitt(16,Torque_TTO(i,j))  ) );
    end
end

% max(0, min((a*log((x)*atan2(x,y))+b)/(y+c),(a*log((x)*atan2(x,y))+b)/(y+c)   ) )
% 
% C(:,:,1)=ones(size(Z));
% C(:,:,2)=zeros(size(Z));
% C(:,:,3)=zeros(size(Z));

h=surf(Throtle_TTO,Torque_TTO,Omega_TTO)
hold on
plot3(throtle,Mom,Omega,'o','MarkerEdgeColor','w','MarkerFaceColor',[0,0,1],...
                'MarkerSize',5)
            xlabel(' Equivalent Throtle (V)')
ylabel(' Torque (N.m)')
zlabel('Omega (rad/s)')
return
%% Ideal surface
X=repmat(min(throtle):0.1:max(throtle),size(min(Mom):0.01:max(Mom),2),1);
Y=repmat(min(Mom):0.01:max(Mom),size(min(throtle):0.1:max(throtle),2),1)';
Z=550*2*pi/60.*X;


C(:,:,1)=ones(size(Z));
C(:,:,2)=zeros(size(Z));
C(:,:,3)=zeros(size(Z));

h=surf(X,Y,Z,C)
set(h,'FaceAlpha',0.5)

legend(' Real curve', 'Test data','Ideal curve')


%% 