%% Simulation of a Linear System
% The title kinda says it all

%% Initializing this bad boy

close all
clear
clc

%% Setting the second-order LTI system parameters
% We can change the parameters of the simulation here instead of 
% having to do it on simulink.
xi = 0.4;
omega_n = 2*pi/5; %rad/s

%% Running the simulation
simout = sim('dif_eq'); %Question: does it need to be in the same folder?

%% Plotting some dope-ass graphs
plot(simout.y.time, simout.y.signals.values);
xlabel('t (s)');
ylabel('y');
grid on;
box on;
title('Example step response of a second-order LTI system');


%% Author notes
% Apparentely this is used to model solar panels deflection and shit.

