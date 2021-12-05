%% Simulation of a Linear System
% The title kinda says it all

%% Initializing this bad boy

close all
clear
clc

%% Setting the second-order LTI system parameters
% We can change the parameters of the simulation here instead of 
% having to do it on simulink.
xis = [0.1, 0.2, 0.5, 0.9, 1, 2, 4];
omega_n = 2*pi/5; %rad/s
StepSize = 0.01;
finaltime = 20;
%% Running the simulation
% We cycle through all the dampings specified in the xis vector
figure(1)
for i = 1:length(xis)
    xi = xis(i);
    simout = sim('dif_eq','StopTime',num2str(finaltime),'FixedStep',num2str(StepSize)); %Question: does it need to be in the same folder?

%% Plotting some dope-ass graphs
plot(simout.y.time, simout.y.signals.values);
hold on
end
%% Gettin'em legends on point
legend(strcat('xi = ', num2str(xis(1))),'Location','southeast')

for j = 2:length(xis)
%first get the legend handles
old_legend=findobj(gcf, 'Type', 'Legend');
%then append the legend with new entry
legend([old_legend.String,strcat('xi = ', num2str(xis(j)))])
end

xlabel('t (s)');
ylabel('y');
grid on;
box on;
title('Example step response of a second-order LTI system');

%% Author notes
% Apparentely this is used to model solar panels deflection and shit.

