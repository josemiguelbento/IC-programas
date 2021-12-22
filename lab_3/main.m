%% Simplified model for altitude control of a drone
% The title kinda says it all
%% Initializing this bad boy
close all
clear
clc

%% Setting the system parameters
% Parameters for running the simulation
finaltime = 200;
StepSize = 0.01;

% Problem parameters
initial_step = 1; %s
M = 1; %kg
G = 9.8; %m/s^2
Kt = 3.575e-5; %N/(rad/s)^2
Z0 = 2; %m

Kp = 0.5;
Kd = 1;

omega_0 = sqrt(G*M/Kt); %rad/s
u_0 = omega_0;

dZr = 2;

%% Defining Transfer Function for root locus
% Proportional
s = tf('s');
K_prop = 600*Kp*Kt*omega_0/M;
g_prop = 1/(s^2*(s+300));

figure(5)
rlocus(g_prop);

% Proportional + Derivative
K_prop_der = 600*Kd*Kt*omega_0/M;
z = Kp/Kd;
g_prop_der = (s+z)/(s^2*(s+300));
figure(6)
rlocus(g_prop_der);

%% Running the complete simulation
% We cycle through all the step sizes specified in the StepSizes vector
for i = 1:length(dZr)
    simout_tot = sim('total','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    %% Plotting some dope-ass graphs

    figure(4*(i-1)+1)
    plot(simout_tot.get('omega_p').time, simout_tot.get('omega_p').signals.values/2/pi*60);
    hold on
    plot(simout_tot.get('omega_pd').time, simout_tot.get('omega_pd').signals.values/2/pi*60);
    xlabel('time (s)')
    ylabel('w (rpm)')
    title(strcat("Angular velocity ", "dZr = ", num2str(dZr), " m"))
    legend('prop','dif prop','Location','southwest');
    
    
    figure(4*(i-1)+2)
    plot(simout_tot.get('z_p').time, simout_tot.get('z_p').signals.values);
    hold on
    plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m"))
    legend('prop','dif prop','Location','southwest');
    
    figure(4*(i-1)+3)
    plot(simout_tot.get('omega_p').time, simout_tot.get('z_pt_p').signals.values);
    hold on
    plot(simout_tot.get('omega_pd').time, simout_tot.get('z_pt_pd').signals.values);
    xlabel('time (s)')
    ylabel('Velocidade (m/s)')
    title(strcat("Velocity ", "dZr = ", num2str(dZr), " m"))
    legend('prop','dif prop','Location','southwest');
    
    figure(4*(i-1)+4)
    plot(simout_tot.get('omega_p').time, simout_tot.get('z_2pt_p').signals.values);
    hold on
    plot(simout_tot.get('omega_pd').time, simout_tot.get('z_2pt_pd').signals.values);
    xlabel('time (s)')
    ylabel('Aceleração (m/s^2)')
    title(strcat("Acceleration ", "dZr = ", num2str(dZr), " m"))
    legend('prop','dif prop','Location','southwest');
    
end
