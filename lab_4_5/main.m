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

dZr = 1; % m

fig_count = 1;
%% Question 4.2 - root locus of PID open loop for different pairs of zeros

% Transfer Function for proportional
s = tf('s');

% Because these values enter the gain expression as (s+z), for a positive
% value here, we have a zero on the left complex semiplane.

z1_test = [0.5, 1, 2];
z2_test = [1, 10, 20];



% For K > 0 the root locus is
for i = 1:length(z1_test)
    figure(fig_count)
    fig_count = fig_count+1;
    g_pid_ol_test = (s+z1_test(i))*(s+z2_test(i))/(s^3*(s+300));
    rlocus(g_pid_ol_test);
    title(strcat("Root locus PID", " z_1 = ", num2str(z1_test(i)), "  z_2 = ", num2str(z2_test(i))))
end

%%
% In all the previous plots we can see the movement of 4 poles. For K=0, 3
% of them sit at the origin (Re = 0, Im = 0), and the other one is equal 
% to -300. If we zoom in near the origin we can see that at first, 2 of the
% poles that sit at the origin will go to the right complex semiplane. For
% example, for the zeros at $z_1$ = 1 and $z_2$ = 10, we see that the two poles
% in question are in the right complex semiplane when K is lower than 275.
% Therefore, the system is unstable for K lower than 275 (for these zeros).
%
% For K = 275 (or close to it), the system is marginally stable,
% since we have 2 poles with 0 real part and 2 poles on the left complex
% semiplane.
%
% For K greater than 275, we get 4 poles in the left complex
% semiplane, thus concluding that the system is stable. This is illustrated
% in the step response in section 4.4a.

%% 
% We will now analyse the root-locus for at least one positive zero (on the right
% complex semiplane).


% Because these values enter the gain expression as (s+z), for a negative
% value here, we have a zero on the right complex semiplane.

z1_test = [-1, 1];
z2_test = [-2, -2];



% For K > 0 the root locus is
for i = 1:length(z1_test)
    figure(fig_count)
    fig_count = fig_count+1;
    g_pid_ol_test = (s+z1_test(i))*(s+z2_test(i))/(s^3*(s+300));
    rlocus(g_pid_ol_test);
    title(strcat("Root locus PID", " z_1 = ", num2str(z1_test(i)), "  z_2 = ", num2str(z2_test(i))))
end

%%
% In both the previous plots we can see the movement of 4 poles. For K=0, 3
% of them sit at the origin (Re = 0, Im = 0), and the other one is equal 
% to -300.

% We will first look at the root locus for zeros at 1 and 2 (figure 4). In
% this root locus, we can see that whatever the value of K, we always have
% 2 poles on the right complex semiplane, which means the system is
% unstable.

% For the second pair, we have zeros at -1 and 2. Again, we can see in the
% root locus that for every value of K, we always have 1 pole on the right
% complex semiplane, so the system is again unstable.

% The step response for both these situations is illustrated in section
% 4.4b.

% From these analysis, we conclude that a system that has zeros in the 
% left complex semiplane is only stable if K is greater than a certain
% value that depends on the values of z_1 and z_2. A system that has
% zeros on the right complex semiplane is always unstable, regardless of the
% value of K.
%% Question 4.3 - finding K so poles are -1, -21 x 2, -224 for z_1=1, z_2=10
% In this section, we plot the root locus for Z_1 = 1 and z_2 = 10, in
% other words, for the system with zeros in -1 and -10. We then search for
% the gain K for which the poles are -1, -21 (double pole) and -224.

z1 = 1;
z2 = 10;

% open loop transfer function of PID
g_pid_ol = (s+z1)*(s+z2)/(s^3*(s+300));

figure(fig_count)
fig_count = fig_count+1;
rlocus(g_pid_ol);
[r_procura,k_procura] = rlocus(g_pid_ol);
title('Root locus for PID z_1 = 1 z_2 = 10')

% Search algorithm for the K to get the desired poles.

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

% Computing coefficients from the K found
kd_procura = k_alvo * M/600/Kt/omega_0;
kp_procura = kd_procura*(z1+z2);
ki_procura = kd_procura*z1*z2;

%% Running the complete simulation for the data from question 4.3
Kd = kd_procura;
Kp = kp_procura;
Ki = ki_procura;

simout_tot = sim('total_lab4','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));

% Plotting the step response of the closed-loop linear system with the
% PID controller.
figure(fig_count)
fig_count = fig_count+1;
plot(simout_tot.get('z_pid').time, simout_tot.get('z_pid').signals.values);
xlabel('time (s)')
ylabel('z (m)')
title({strcat("Altitude ", "dZr = ", num2str(dZr), " m"),...
    strcat('z_1 = ', num2str(z1), 'z_2 = ', num2str(z2), '   K_p = ',...
    num2str(Kp), '   K_i = ', num2str(Ki), '   K_d = ', num2str(Kd))})
legend('PID','Location','southeast');

% In this figure, we have the representation of the step response for a
% system with 2 negative zeros (-1 and -10), with a gain that is greater
% than 275 (K = 11740). Here we confirm what was expected by looking at the
% root-locus for this kind of parameters in section 4.2. As expected , for
% both the zeros negative and a gain greater than 275 (for zeros at -1 and
% -10), the system is stable, thus converging to the reference altitude.

%% Question 4.4 - effect of a varying z1 and z2 with constant K and negative zeros
finaltime = 8;

K_44a = 11740;
z1_44a = [0.5, 1, 2];
z2_44a = [1, 10, 20];

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

% In this figure, we can see the step response for a constant K for
% negative zeros. Here we can see that the smaller (the real part of) the zero is (the greater
% it's absolute value, since it is negative), the smaller is the time
% the system takes to converge.
%
% Looking at the root locus from section 4.2 we can understand why. A lower
% value of zero "pulls" the poles to a more negative (real) value (for the
% same K).
%
% For example, for the zeros at -0.5 and -1, with a gain of 23000, we get a
% pole at -1.03. In contrast, for zeros at -2 and -20, with the same gain,
% the pole in question would take the value of -34.7. This second, more
% negative pole will result in a faster convergence of the system, thus
% illustrating our point.
%
% Note: In this explanation we refer to negative and positive poles and zeros, as
% well as poles and zeros greater than each other since all the values are
% real numbers.

%% Question 4.4 - effect of a varying z1 and z2 with constant K 4.4 and at least one positive zero
finaltime = 8;

K_44a = 11740;
z1_44a = [-1 1];
z2_44a = [-2 -2];

for i = 1:length(z1_44a)
    Kd = K_44a * M/600/Kt/omega_0;
    Kp = Kd*(z1_44a(i)+z2_44a(i));
    Ki = Kd*z1_44a(i)*z2_44a(i);
    simout_tot = sim('total_lab4','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(fig_count)
    fig_count = fig_count+1;
    
    plot(simout_tot.get('z_pid').time, simout_tot.get('z_pid').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("Altitude ", "dZr = ", num2str(dZr), " m   For constant K = 11740"))
    legend(strcat( "z_1 = ", num2str(z1_44a(i)), "  z_2 = ", num2str(z2_44a(i))))
end

% In these two figures, we present the step response for at least one zero
% in the right complex semiplane. 

%% Question 4.4 - effect of a varying K with constant z1 and z2
finaltime = 5;

K_44b = [150, 275, 1000, 11740];
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
legend(legendcellb,'Location','northwest');


%% Lab 5 - Tracking error and disturbance rejection

%% Question 5.4 - effect of disturbances with PD controller
% First we study the effect of varying the proportional gain Kp.

% Parameters for running the simulation
finaltime = 5;
StepSize = 0.01;

% For this set of data, we exemplify the difference in response of the
% analysed systems.
dZr = 0; % m

Kd_54a = 106.14;
Kp_54a = [106.14 89.04 1060 5300 21200];

z_pd = 1;
w_d_rpm = 50;
w_d = w_d_rpm/60*2*pi;


legendcell54a = {};

for i = 1:length(Kp_54a)
    
    Kd = Kd_54a;
    Kp = Kp_54a(i);

    % Defining the parameters for the simulation.
    K_prop_der = 600*Kd*Kt*omega_0/M; % proportional-derivative controller gain

    z = Kp/Kd;

    simout_tot = sim('total_lab5','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(fig_count)

    plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("PD with disturbance - Altitude ", "dZr = ", num2str(dZr), " m  z = 1", " K_d = ", num2str(Kd_54a)))
    legendcell54a = [legendcell54a, cellstr(strcat('K_p = ', num2str(Kp_54a(i))))];
end

for i = 1:length(Kp_54a)
    Kd = Kd_54a;
    Kp = Kp_54a(i);
    figure(fig_count)
    hold on
    yline(Z0+dZr + w_d/Kp, '--');
end

legendcell54a = [legendcell54a, cellstr('Theo. value ss out')];


set(gca, 'YLimSpec', 'stretch');

fig_count = fig_count+1;
legend(legendcell54a,'Location','Southeast');

%%
% Next we study the effect of varying the derivative gain Kd.

% Parameters for running the simulation
finaltime = 5;
StepSize = 0.01;

% For this set of data, we exemplify the difference in response of the
% systems analysed.
dZr = 0; % m

Kd_54b = [106.14 89.04 500 1500 20];
Kp_54b = 106.14; % experimentar 106 ?

z_pd = 1;
w_d_rpm = 50;
w_d = w_d_rpm/60*2*pi;


legendcell54b = {};

for i = 1:length(Kd_54b)
    
    Kd = Kd_54b(i);
    Kp = Kp_54b;

    % Defining the parameters for the simulation.
    K_prop_der = 600*Kd*Kt*omega_0/M; % proportional-derivative controller gain

    z = Kp/Kd;
    
    simout_tot = sim('total_lab5','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(fig_count)

    plot(simout_tot.get('z_pd').time, simout_tot.get('z_pd').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("PD with disturbance - Altitude ", "dZr = ", num2str(dZr), " m  z = 1", " K_p = ", num2str(Kp_54b)))
    legendcell54b = [legendcell54b, cellstr(strcat('K_d = ', num2str(Kd_54b(i))))];
end

for i = 1:length(Kd_54b)
    Kd = Kd_54b(i);
    Kp = Kp_54b;
    figure(fig_count)
    hold on
    yline(Z0+dZr + w_d/Kp, '--');
end

legendcell54b = [legendcell54b, cellstr('Theoretical value for steady state output')];


set(gca, 'YLimSpec', 'stretch');

fig_count = fig_count+1;
legend(legendcell54b,'Location','Northeast');

%%

% discuss the results 

%% Question 5.6 - effect of disturbances with PID controller

dZr = 0; % m
K_56 = [150, 275, 1000];
z1 = 1;
z2 = 10;
w_d_rpm = 50;
w_d = w_d_rpm/60*2*pi;

legendcellb = {};

for i = 1:length(K_56)
    Kd = K_56(i) * M/600/Kt/omega_0;
    Kp = Kd*(z1+z2);
    Ki = Kd*z1*z2;
    simout_tot = sim('total_lab5','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(fig_count)

    plot(simout_tot.get('z_pid').time, simout_tot.get('z_pid').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("PID with disturbance - Altitude ", "dZr = ", num2str(dZr), " m  z_1 = -1, z_2 = -10"))
    legendcellb = [legendcellb, cellstr(strcat('K = ', num2str(K_56(i))))];
end

yline(Z0+dZr, '--');
legendcellb = [legendcellb, cellstr('Theoretical value for steady state output')];
    
fig_count = fig_count+1;
legend(legendcellb,'Location','Northeast');

%% Effect of disturbances with PID controller (going stable). 5.6

dZr = 0; % m
K_56 = [10000, 11740];
z1 = 1;
z2 = 10;
w_d_rpm = 50;
w_d = w_d_rpm/60*2*pi;

legendcellb = {};

for i = 1:length(K_56)
    Kd = K_56(i) * M/600/Kt/omega_0;
    Kp = Kd*(z1+z2);
    Ki = Kd*z1*z2;
    simout_tot = sim('total_lab5','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize));
    
    figure(fig_count)

    plot(simout_tot.get('z_pid').time, simout_tot.get('z_pid').signals.values);
    hold on
    xlabel('time (s)')
    ylabel('z (m)')
    title(strcat("PID with disturbance - Altitude ", "dZr = ", num2str(dZr), " m  z_1 = -1, z_2 = -10"))
    legendcellb = [legendcellb, cellstr(strcat('K = ', num2str(K_56(i))))];
end

yline(Z0+dZr, '--');
legendcellb = [legendcellb, cellstr('Theoretical value for steady state output')];
    
ylim([Z0-0.00001 inf]);

fig_count = fig_count+1;
legend(legendcellb,'Location','Northeast');

%%

% discuss the results