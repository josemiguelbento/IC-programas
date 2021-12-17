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

%% Running the complete simulation
% We cycle through all the step sizes specified in the StepSizes vector
for i = 1:length(u)
    simout_tot = sim('total','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    %% Plotting some dope-ass graphs

    figure(4*(i-1)+1)
    plot(simout_tot.get('omega_lin').time, simout_tot.get('omega_lin').signals.values/2/pi*60);
    hold on
    plot(simout_tot.get('omega').time, simout_tot.get('omega').signals.values/2/pi*60);
    xlabel('time (s)')
    ylabel('w (rpm)')
    title(strcat("Angular velocity ", "u = ", num2str(u_rpm(i)), " rpm"))
    legend('lin','non lin','Location','southwest');
    
    
    figure(4*(i-1)+2)
    plot(simout_tot.get('z_lin').time, simout_tot.get('z_lin').signals.values);
    hold on
    plot(simout_tot.get('z').time, simout_tot.get('z').signals.values);
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "u = ", num2str(u_rpm(i)), " rpm"))
    legend('lin','non lin','Location','southwest');
    
    figure(4*(i-1)+3)
    plot(simout_tot.get('omega_lin').time, simout_tot.get('z_pt_lin').signals.values);
    hold on
    plot(simout_tot.get('omega').time, simout_tot.get('z_pt').signals.values);
    xlabel('time (s)')
    ylabel('Velocidade (m/s)')
    title(strcat("Velocity ", "u = ", num2str(u_rpm(i)), " rpm"))
    legend('lin','non lin','Location','southwest');
    
    figure(4*(i-1)+4)
    plot(simout_tot.get('omega_lin').time, simout_tot.get('z_2pt_lin').signals.values);
    hold on
    plot(simout_tot.get('omega').time, simout_tot.get('z_2pt').signals.values);
    xlabel('time (s)')
    ylabel('Aceleração (m/s^2)')
    title(strcat("Acceleration ", "u = ", num2str(u_rpm(i)), " rpm"))
    legend('lin','non lin','Location','southwest');
    
end
