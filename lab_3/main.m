%% Simplified model for altitude control of a drone
% The title kinda says it all
%% Initializing this bad boy
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

%% Root locus for proportional loop
% Transfer Function for proportional
s = tf('s');

g_prop = 1/(s^2*(s+300));

% For K > 0 the root locus is
figure(1)
rlocus(g_prop);
title('Root locus for K > 0, proportional loop')

% For K < 0 we do rlocus for the symetric of the transfer function
figure(2)
rlocus(-g_prop);
title('Root locus for K < 0, proportional loop')


%% Root locus for derivative loop
% Transfer Function for Proportional and Derivative
z_test = [-20, -10, 10, 20];

% For K > 0 we do rlocus for several values of z
for i = 1:length(z_test)
    figure(i+2)
    g_prop_der = (s+z_test(i))/(s^2*(s+300));
    rlocus(g_prop_der);
    title(strcat("Root locus ", "z = ", num2str(z_test(i))))
end
%% Quastion 3.8
% Procurar o K que satisfaz a condição de haver um polo duplo
z_procura = 1;
polo_procurado = -2.01;
g_procura = (s+z_procura)/(s^2*(s+300));
[r_procura,k_procura] = rlocus(g_procura);
k_alvo = 0;
polo_alvo = 0+0i;
for j = 2:length(k_procura)
    if(abs(r_procura(1,j)-r_procura(2,j))<0.0001)
        k_alvo = k_procura(j);
        polo_alvo = r_procura(1,j);
        break
    end
end
Kd_procura = k_alvo/(600*Kt*omega_0/M);
Kp_procura = z_procura * Kd_procura;



%% Running the complete simulation for the data from question 3.8
% For this set of data, we exemplify the difference in response of the
% systems analysed

Kp = Kd_procura;
Kd = Kp_procura;


K_prop = 600*Kp*Kt*omega_0/M;
K_prop_der = 600*Kd*Kt*omega_0/M;
z = Kp/Kd;

% Plotting the root locus for z = 1 and displaying the double pole
figure(3+length(z_test))
g_prop_der = (s+z)/(s^2*(s+300));
rlocus(g_prop_der);
hold on
plot(real(r_procura(1,j)), imag(r_procura(1,j)), 'r.','MarkerSize', 15);
title('Root locus for z = 1, displaying the double pole');

% Running the simulation for the previous data
simout_tot = sim('total_lab3','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

figure(4+length(z_test))
plot(simout_tot.get('z_p').time, simout_tot.get('z_p').signals.values);
hold on
plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
plot(simout_tot.get('z_pdtf').time, simout_tot.get('z_pdtf').signals.values);
xlabel('time (s)')
ylabel('z (m)')
title({strcat("Altitude ", "dZr = ", num2str(dZr), " m"),...
    strcat('z = ', num2str(z), '   K_p = ', num2str(Kp), '   K_d = ', num2str(Kd))})
legend('prop','dif prop','dif prop with tf','Location','southeast');


%% Effect of a varying z for contant K 3.9a
K_39a = 1192;
z_39a = [0, 1, 10, 50];

legendcell = {};

for i = 1:length(z_39a)
    z = z_39a(i);
    Kd = K_39a/(600*Kt*omega_0/M);
    Kp = z_39a * Kd;
    K_prop = 600*Kp*Kt*omega_0/M;
    K_prop_der = 600*Kd*Kt*omega_0/M;
    simout_tot = sim('total_lab3','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(5+length(z_test))
    %plot(simout_tot.get('z_p').time, simout_tot.get('z_p').signals.values);
    %plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
    plot(simout_tot.get('z_pdtf').time, simout_tot.get('z_pdtf').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m    For constant K = 1192"))
    legendcell = [legendcell, cellstr(strcat('z = ', num2str(z)))];
end
legend(legendcell,'Location','southeast');


%% Effect of a varying K for contant z 3.9b
finaltime = 10;

K_39b = [150,500, 1192, 3000];
z_39b = 1;

legendcellb = {};

for i = 1:length(K_39b)
    z = z_39b;
    Kd = K_39b(i)/(600*Kt*omega_0/M);
    Kp = z_39b * Kd;
    K_prop = 600*Kp*Kt*omega_0/M;
    K_prop_der = 600*Kd*Kt*omega_0/M;
    simout_tot = sim('total_lab3','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(6+length(z_test))
    %plot(simout_tot.get('z_p').time, simout_tot.get('z_p').signals.values);
    %plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
    plot(simout_tot.get('z_pdtf').time, simout_tot.get('z_pdtf').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m   For constant z = 1"))
    legendcellb = [legendcellb, cellstr(strcat('K = ', num2str(K_39b(i))))];
end
legend(legendcellb,'Location','southeast');


