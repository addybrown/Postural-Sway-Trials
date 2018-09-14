clear variables;
close all;
clc;

load('Trial_Ex','-mat')

%% Plot Raw Data
sampletime = 1/256;
t = 0:sampletime:5;

accel_data(1:length(t),1) = Accel(1:length(t),1);
gyro_data(1:length(t),1) = Gyro(1:length(t),1);

thetaint = cumtrapz(t,gyro_data);
theta_ac = zeros(length(t),1);

for i = 1:length(t)
    theta_ac(i,1) = asind(accel_data(i,1)/-9.81);
end

% for i = 1:length(t)
%     theta_ac(i,1) = theta_ac(i,1) - mean(theta_ac);
% end

posn = zeros(length(t),1);

for i = 1:length(t)
    posn(i,1) = 1.5*sind(theta_ac(i,1));
end


figure(1)
plot(t',accel_data(:,1))
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Accelerometer Raw Data');

figure(2)
plot(t',gyro_data(:,1))
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Gyroscope Raw Data');

figure(3)
plot(t,thetaint(:,1))
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Angle from Integration');

figure(4)
plot(t,theta_ac(:,1))
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Angle from Accelerometer');

figure(5)
plot(t,posn(:,1))
xlabel('Time (s)');
ylabel('CoM (m)');
title('Center of Mass');