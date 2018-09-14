clear variables; 
close all;
clc;
%% Extended Kalman Filter Matlab

load('Trial_1','-mat')
%% Set up time step and parameters

del_t = 1/256;            %time step
runT = 5;              %run time
nT = runT/del_t;    %number of samples
t = 0:del_t:runT;

n = 2;                  %number of states
nz = 2;                 %number of measurements
g = 9.81;
u = 0.506*10^-3;
%% Constructing State Vectors
x = zeros(n,nT);        %x_0 = [0;0]
%x(1,1) = 20;
%x(2,1) = 2;
P = zeros(n,n,nT);      %covariance matrix
z = zeros(nz,nT);    %measurement vector

for i = 1:nT
    P(:,:,i) = eye(n);
end 

%% System Model
syms theta theta_dot

Q = diag([0.4 0.2]);
R = diag([8.502*10^-5, 0.0095]);
G = [1 del_t;
     0 1];
B = [-del_t;0];
%% Sensor Model
H = zeros(nz,n,nT);
h = zeros(nz,nT);

h_model = [g*sin(theta);theta_dot];
H_model = [g*theta_dot*cos(theta), 0;0 1];

%% Setting up Matrices
xpred = x;
Ppred = P;
z(1,:) = Accel(nT,1)';
z(2,:) = Gyro(nT,1)';

%% Begin simulation
for k = 2:nT
    
    %Predict
    %xpred(:,k) = double(subs(g,t,timevec(k-1))); 
    %Ppred(:,:,k) = double(subs(G,t,timevec(k-1)))*P(:,:,k-1)*double(subs(G,t,timevec(k-1))) + Q;
    xpred(:,k) = G*x(:,k-1) + B*u;
    Ppred(:,:,k) = G*P(:,:,k-1)*G' + Q;
    
    %Calculate H and h   
    h(1,k) = double(subs(h_model(1),theta,xpred(1,k)));
    h(2,k) = double(subs(h_model(2),theta_dot,xpred(2,k)));
    
    H(1,1,k) = double(subs(H_model(1,1),[theta,theta_dot],[xpred(1,k),xpred(2,k)]));
    H(2,2,k) = 1;
    
    %Update - assume z has been set
    K = Ppred(:,:,k)*H(:,:,k)'*inv((H(:,:,k)*Ppred(:,:,k)*H(:,:,k)' + R));
    x(:,k) = xpred(:,k) + K*(z(:,k) - h(:,k)); %z(:,k) is either actual measurements or simulated measurements
    P(:,:,k) = (eye(n) - K*H(:,:,k))*Ppred(:,:,k);
    
end

%% Extract CoM Position
l = 1.5;
posn = zeros(length(t)-1,1);
for i = 1:length(t)-1
    posn(i,1) = l*sin(x(1,i));
end

t_fil = t(1,1:length(t)-1);

%% Plots
figure(1)
title('COM vs Time')
xlabel('Time(s)')
ylabel('Distance(m)')
plot(t_fil',posn(:,1))


% figure(2)
% title('Velocity vs Time with Object Motion Data - Poor Sensing Conditions')
% xlabel('Time(s)')
% ylabel('Velocity(cm/s)')
% hold on
% %plot(t,xsim(2,:),'DisplayName','Simulated Model')
% plot(t_fil,x(2,:),'DisplayName','Extended Kalman Filter')
% legend('show')





