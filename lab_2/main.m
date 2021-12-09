close all
clear
clc

finaltime = 2;
StepSize = 0.01;

initial_step = 1;
M = 1; %kg
G = 9.8; %m/s^2
Kt = 3.575e-5; %N/(rad/s)^2
Z0 = 2; %m

omega_0 = sqrt(G*M/Kt); %rad/s
u_0 = omega_0;
u_rpm = [100, 1000, 5000];
u = u_rpm.*(2*pi/60); %rad/s

simout = sim('non_linear','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

legendcell = {};
for i = 1:length(u)
    legendcell = [legendcell, cellstr(strcat('u = ', num2str(u_rpm(i))))];
end

fig1 = figure(1);
plot(simout.omega.time, simout.omega.signals.values);
legend(legendcell,'Location','southwest');

fig2 = figure(2);
plot(simout.z.time, simout.z.signals.values);
legend(legendcell,'Location','southwest');

    

