%clear variables;
close all;
clc;

load('Trial_4','-mat')

%% Set up time step and parameters

del_t = 1/256;            %time step
runT = 5;              %run time
nT = runT/del_t + 1;    %number of samples
t = 0:del_t:runT;

n = 2;                  %number of states
nz = 2;                 %number of measurements
%% Constructing State Vectors
x = zeros(n,nT);        %x_0 = [0;0]
P = zeros(n,n,nT);      %covariance matrix
z = zeros(nz,nT);    %measurement vector

for i = 1:nT
    P(:,:,i) = eye(n);
end 

%% Setting up Matrices
xpred = x;
Ppred = P;

% Constant Velocity Model
A = [1 -del_t; 
    0 1];
B = [del_t;
     0];
 
C = [1 0];

Q = diag([0.4 0.2]);   % What do you know about system errors?

R = 8.50200000000000e-05;

g = -9.81;

%% Import Sensor Data 

for i = 1:nT
    z(1,i) = Gyro(i,1);
end

%% Just Accel
for i = 1:nT
    z(2,i) = asin(Accel(i,1)/g);
end

%% Just Gyro

t_g = cumtrapz(t,Gyro(1:length(t)));

%% Begin simulation
for k = 2:nT
    
    %Predict
    xpred(:,k) = A*x(:,k-1) + B*z(1,k); 
    Ppred(:,:,k) = A*P(:,:,k-1)*A' + Q;
    
    %Update - assume z has been set
    K = Ppred(:,:,k)*C'*inv(C*Ppred(:,:,k)*C' + R);
    x(:,k) = xpred(:,k) + K*(z(2,k) - C*xpred(:,k)); %z(:,k) is either actual measurements or simulated measurements
    P(:,:,k) = (eye(n) - K*C)*Ppred(:,:,k);
    
end

%% Plot
l = 1;
posn = zeros(length(t),1);

for i = 1:length(t)
    posn(i) = l*sin(x(1,i));
end


figure(1)
plot(t',posn(:,1));
ylabel('Position (m)')
xlabel('Time (s)')
title('Position of Center of Mass vs Time')

figure
plot(t,x(1,:));
hold on 
plot(t,z(2,:));
% hold on
% plot(t,t_g(:,1));
ylabel('Angle (rad)')
xlabel('Time (s)')
legend('KF','Accelerometer')
title('Tilt Angle vs Time')


