%% Simplified model for altitude control of a drone
% The title kinda says it all
%% Initializing this bad boy
close all
clear
clc

%% Setting the system parameters
% Parameters for running the simulation
finaltime = 2;
StepSize = 0.01;

% Problem parameters
initial_step = 1; %s
M = 1; %kg
G = 9.8; %m/s^2
Kt = 3.575e-5; %N/(rad/s)^2
Z0 = 2; %m

omega_0 = sqrt(G*M/Kt); %rad/s
u_0 = omega_0;
u_rpm = [100,1000,5000];
u = u_rpm.*(2*pi/60); %rad/s
delta_u = u-u_0;

%% Gettin'em legends on point
legendcell = {};
for i = 1:length(u)
    legendcell = [legendcell, cellstr(strcat('u = ', num2str(u_rpm(i))))];
end

% %% Running the non_linear simulation
% % We cycle through all the step sizes specified in the StepSizes vector
% simout = sim('non_linear','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
% 
% 
% %% Plotting some dope-ass graphs
% % Velocidade angular em função do tempo em resposta a alteração de u
% figure(1)
% plot(simout.get('omega').time, simout.get('omega').signals.values/2/pi*60);
% xlabel('time (s)')
% ylabel('w (rpm)')
% legend(legendcell,'Location','southwest');
% title("U with non-linear")
% 
% % Altitude em função do tempo para deiferentes inputs de u
% figure(2)
% plot(simout.get('z').time, simout.get('z').signals.values);
% xlabel('time (s)')
% ylabel('z (m)')
% legend(legendcell,'Location','southwest');
% title("Z with non-linear")
% 
% %% Running the linear simulation
% % We cycle through all the step sizes specified in the StepSizes vector
% for i = 1:length(u)
%     simout_lin = sim('laplace_transform','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
%     %% Plotting some dope-ass graphs
% 
%     figure(3)
%     plot(simout_lin.get('omega_lin').time, simout_lin.get('omega_lin').signals.values/2/pi*60);
%     xlabel('time (s)')
%     ylabel('w (rpm)')
%     hold on
%     
%     figure(4)
%     plot(simout_lin.get('z_lin').time, simout_lin.get('z_lin').signals.values);
%     xlabel('time (s)')
%     ylabel('z (m)')
%     hold on
% end
% %% Gettin'em legends on point
% 
% 
% figure(3)
% title("U with Laplace transform")
% legend(legendcell,'Location','southwest');
% 
% figure(4)
% title("Z with Laplace transform")
% legend(legendcell,'Location','southwest');
% 

%% Running the complete simulation
% We cycle through all the step sizes specified in the StepSizes vector
for i = 1:length(u)
    simout_tot = sim('total','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    %% Plotting some dope-ass graphs

    figure(5)
    plot(simout_tot.get('omega_lin').time, simout_tot.get('omega_lin').signals.values/2/pi*60);
    xlabel('time (s)')
    ylabel('w (rpm)')
    hold on
    
    figure(6)
    plot(simout_tot.get('z_lin').time, simout_tot.get('z_lin').signals.values);
    xlabel('time (s)')
    ylabel('z (m)')
    hold on
    
    figure(7)
    plot(simout_tot.get('omega').time, simout_tot.get('omega').signals.values/2/pi*60);
    xlabel('time (s)')
    ylabel('w (rpm)')
    hold on

    % Altitude em função do tempo para diferentes inputs de u
    figure(8)
    plot(simout_tot.get('z').time, simout_tot.get('z').signals.values);
    xlabel('time (s)')
    ylabel('z (m)')
    hold on
    
    
end
%% Gettin'em legends on point

figure(5)
title("U with linear")
legend(legendcell,'Location','southwest');

figure(6)
title("Z with linear")
legend(legendcell,'Location','southwest');

figure(7)
title("U with non-linear")
legend(legendcell,'Location','southwest');

figure(8)
title("Z with non-linear")
legend(legendcell,'Location','southwest');

