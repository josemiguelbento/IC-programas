%% Simplified model for altitude control of a drone
%% Initializing workspace
close all
clear
clc

%% Setting the system parameters


% Parameters for running the simulation
finaltime = 5;
StepSize = 0.01;

% Problem parameters
initial_step = 1; % s
M = 1; % kg
G = 9.8; % m/s^2
Kt = 3.575e-5; % N/(rad/s)^2
Z0 = 2; % m

omega_0 = sqrt(G*M/Kt); % rad/s
u_0 = omega_0;

dZr = 10; % m

%% Question 3.4


% Transfer Function for the proportional controller
s = tf('s');

g_prop = 1/(s^2*(s+300));

%%
% For K > 0, the root locus is
figure(1)
rlocus(g_prop);
title('Root locus for K > 0, proportional loop')

%%
% As can be seen from the root locus, the proportional system always has
% either two poles with positive real part or a pole with zero real part
% but multiplicity 2. Thus, by the Hurwitz criterion, the system is
% unstable.

%% Question 3.5
% For K < 0, we obtain the root locus of the symmetric of the transfer
% function
figure(2)
rlocus(-g_prop);
title('Root locus for K < 0, proportional loop')


%% Question 3.7
% Now we use the transfer function for a proportional-derivative
% controller and plot the root locus for different values of z and constant
% K > 0
z_test = [-500, -20, -10, 10, 20, 500];

for i = 1:length(z_test)
    figure(i+2)
    g_prop_der = (s+z_test(i))/(s^2*(s+300));
    rlocus(g_prop_der);
    title(strcat("Root locus ", "z = ", num2str(z_test(i))))
end

%%
% Here are 6 root loci for different values of z. For z < 0, the system is
% unstable as there is always a pole with positive real part or a double
% pole with zero real part. For small enough z > 0, the system is stable
% except for a specific value of K that causes a double pole with zero real
% part. For large z > 0, the system is again unstable.
%% Question 3.8 - searching the value for K

% Searches the value for K such that the closed-loop system has a double
% pole at s = -2.01

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

% Calculating Kp and Kd from the K found in the search for the double pole
% (-2.01). The K for the pole mentioned is K = 1192.

Kd_procura = k_alvo/(600*Kt*omega_0/M);
Kp_procura = z_procura * Kd_procura;

%% Question 3.8 - running the complete simulation
% For this set of data, we exemplify the difference in response of the
% systems analysed.

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

% Plotting the step response of the closed-loop linear system with the
% proportional-derivative controller.
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

%%
% Here we plotted 3 different graphs to compare different controllers.
% 
% As indicated in the legend, the blue one originates from a simple
% proportional controller. As seen, it does not stabilize in the reference
% value.
% 
% The red graph is the step response of a proportional-derivative
% controller, the simulation for which is done step by step using the
% script from lab class 2. In the simulink model it is the "proportional
% derivative" block. This is the answer to question 3.8.
% 
% The final yellow graph is the step response of a proportional derivative
% controller, the simulation for which is done with a single transfer
% function. In the simulink model it is the "tf proportional derivative" block.
% Note that this one differs from the second graph although theoretically
% they should look alike.
%% Question 3.9 - effect of a varying z for constant K
% We now plot the step response keeping K constant but varying z.
K_39a = 1192;
z_39a = [1, 10, 50, 300];

legendcella = {};

for i = 1:length(z_39a)
    z = z_39a(i);
    Kd = K_39a/(600*Kt*omega_0/M);
    Kp = z_39a(i) * Kd;
    K_prop = 600*Kp*Kt*omega_0/M;
    K_prop_der = 600*Kd*Kt*omega_0/M;
    simout_tot = sim('total_lab3','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(5+length(z_test))
    plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m    For constant K = 1192"))
end
legendcella = "z = " + string(cast(z_39a,'int32'));
legend(legendcella,'Location','Southeast');

%%
% From the obtained plots we conclude that the response is faster as we
% increased z, however, at the cost of also increasing its oscillations and
% overshoot.

%% Question 3.9 - effect of a varying K for constant z
% We now plot the step response keeping z constant but varying K.
finaltime = 10;

K_39b = [150, 500, 1192, 3000];
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
    plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m   For constant z = 1"))
    legendcellb = [legendcellb, cellstr(strcat('K = ', num2str(K_39b(i))))];
end
legend(legendcellb,'Location','southeast');

%%
% From these plots we conclude that increasing K not only makes the response
% faster but also reduces the overshoot.
