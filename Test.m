
%% Collect Large amount of KF data from multiple trials 

A9=zeros(size(z));

[Y1,X1,Af]=PosturalSway(z(2,:),z(1,:));

plot(t,Y1)

hold on
plot(t,posn)