clear variables;
close all;
clc;

load('posn_mat','-mat')

%% Calculate Metrics

mean_mat = zeros(1281,15);
zero_posn_mat = zeros(1281,15);
stdev_vec = zeros(1,15);
max_vec = zeros(1,15);
z = zeros(1281,1);

for i = 1:15
    mean_mat(:,i) = mean(posn_mat(:,i));
    zero_posn_mat(:,i) = posn_mat(:,i) - mean_mat(:,i);
    stdev_vec(i) = std(zero_posn_mat(:,i));
    max_vec(i) = max(abs(zero_posn_mat(:,i)));
end

% figure(1)
% scatter(posn_mat(:,1),z);
% for i = 2:10
%     hold on
%     scatter(posn_mat(:,i),z)
% end
% xlabel('Position X (cm)')
% ylabel('Position Y (cm)')
% title('Postural Sway Map of Quiet standing')

stdev = mean(stdev_vec(1:10));
max = mean(max_vec(1:10));
stdev_int = mean(stdev_vec(11:15));
max_int = mean(max_vec(11:15));
