%% Simplified model for altitude control of a drone
% The title kinda says it all
%% Initializing workspace
close all
clear
clc

%% Setting the system parameters
% Parameters for running the simulation
finaltime = 5;
StepSize = 0.01;

% Problem parameters
initial_step = 1; %s
M = 1; %kg
G = 9.8; %m/s^2
Kt = 3.575e-5; %N/(rad/s)^2
Z0 = 2; %m


omega_0 = sqrt(G*M/Kt); %rad/s
u_0 = omega_0;

dZr = 10; %m

fig_count = 1;
%% 4.2 Root locus for PID open loop different pairs of zeros
% Transfer Function for proportional
s = tf('s');

z1_test = [0.5, 1,2];
z2_test = [1,10,20];



% For K > 0 the root locus is
for i = 1:length(z1_test)
    figure(fig_count)
    fig_count = fig_count+1;
    g_pid_ol_test = (s+z1_test(i))*(s+z2_test(i))/(s^3*(s+300));
    rlocus(g_pid_ol_test);
    title(strcat("Root locus PID", " z_1 = ", num2str(z1_test(i)), "  z_2 = ", num2str(z2_test(i))))
end


%% 4.3 z_1=1 z_2=10 finding K so poles are -1, -21 x 2, -224
z1 = 1;
z2 = 10;
g_pid_ol = (s+z1)*(s+z2)/(s^3*(s+300));
figure(fig_count)
fig_count = fig_count+1;
rlocus(g_pid_ol);
[r_procura,k_procura] = rlocus(g_pid_ol);
title('Root locus for PID z_1 = 1 z_2 = 10')
for j = 1:length(k_procura)
    if(abs(r_procura(1,j) - (-1))< 0.01 && ...
            imag(r_procura(2,j)) < 0.0001 && ...
            abs(r_procura(3,j) - r_procura(2,j)) < 0.0001 && ...
            imag(r_procura(4,j)) < 0.0001)
        k_alvo = k_procura(j);
        polo_alvo = r_procura(1,j);
        break
    end
end

% Calculating coefficients from the K found
kd_procura = k_alvo * M/600/Kt/omega_0;
kp_procura = kd_procura*(z1+z2);
ki_procura = kd_procura*z1*z2;

%% Running the complete simulation for the data from question 4.3
Kd = kd_procura;
Kp = kp_procura;
Ki = ki_procura;

simout_tot = sim('total_lab4','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

% Plotting the step response of the closed-loop linear system with the
% pid controller.
figure(fig_count)
fig_count = fig_count+1;
plot(simout_tot.get('z_pid').time, simout_tot.get('z_pid').signals.values);
xlabel('time (s)')
ylabel('z (m)')
title({strcat("Altitude ", "dZr = ", num2str(dZr), " m"),...
    strcat('z_1 = ', num2str(z1), 'z_2 = ', num2str(z2), '   K_p = ',...
    num2str(Kp), '   K_i = ', num2str(Ki), '   K_d = ', num2str(Kd))})
legend('PID','Location','southeast');


%% Effect of a varying z1 and z2 for contant K 4.4a
finaltime = 10;

K_44a = 11740;
z1_44a = [0.5, 1,2];
z2_44a = [1, 10,20];

legendcella = {};

for i = 1:length(z1_44a)
    Kd = K_44a * M/600/Kt/omega_0;
    Kp = Kd*(z1_44a(i)+z2_44a(i));
    Ki = Kd*z1_44a(i)*z2_44a(i);
    simout_tot = sim('total_lab4','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(fig_count)

    plot(simout_tot.get('z_pid').time, simout_tot.get('z_pid').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m   For constant K = 11740"))
    legendcella = [legendcella, cellstr(strcat( "z_1 = ", num2str(z1_44a(i)), "  z_2 = ", num2str(z2_44a(i))))];
end
fig_count = fig_count+1;
legend(legendcella,'Location','southeast');


%% Effect of a varying K for contant z1 and z2 4.4b
finaltime = 10;

K_44b = [150,500, 11740, 20000];
z1_44b = z1;
z2_44b = z2;

legendcellb = {};

for i = 1:length(K_44b)
    Kd = K_44b(i) * M/600/Kt/omega_0;
    Kp = Kd*(z1_44b+z2_44b);
    Ki = Kd*z1_44b*z2_44b;
    simout_tot = sim('total_lab4','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(fig_count)

    plot(simout_tot.get('z_pid').time, simout_tot.get('z_pid').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m   For constant z_1 = 1, z_2 = 10"))
    legendcellb = [legendcellb, cellstr(strcat('K = ', num2str(K_44b(i))))];
end
fig_count = fig_count+1;
legend(legendcellb,'Location','southeast');
