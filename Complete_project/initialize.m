clear all
clc
close all
%% Initialization
initial_condition = [12 0 pi/2];
obstacle_position = [0 10];
parking_spot = [14 0 -pi/2]; % or [12, -2, pi/2] to show the backwards maneuver
use_posture_regulator = 1;

if(use_posture_regulator)
    sim_time = 40; %15%40
else
    sim_time = 12.2;
end    

%% Simulation
sim('unicycle_singularity_check.slx', sim_time);

%% Animation
set(groot, 'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(gca,'TickLabelInterpreter','latex');
close all;

animation(state,trajectory,state.time, obstacle_position, parking_spot);

%% State evolution
close all

nonzero = nonzeros(trajectory.signals.values(:,1));
to_plot = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(1)];
plot(state.time, state.signals.values(:,1),'Color','b','LineWidth', 0.6); hold on;
plot(state.time, to_plot,'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, x", 'interpreter','latex');
legend({"$x(t)$","$x_d(t)$"}, 'Fontsize',12, 'Location','southeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("x coordinate [m]", 'interpreter','latex');
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/state_x.pdf','ContentType','vector');
close all

nonzero = nonzeros(trajectory.signals.values(:,2));
to_plot = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(2)];
plot(state.time, state.signals.values(:,2),'Color','b','LineWidth', 0.6); hold on;
plot(state.time, to_plot,'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, y", 'interpreter','latex');
legend({"$y(t)$","$y_d(t)$"}, 'Fontsize',12, 'Location','northeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("y coordinate [m]", 'interpreter','latex');
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/state_y.pdf','ContentType','vector');
close all

nonzero = nonzeros(wrapToPi(flatness.signals.values(:,5)));
to_plot = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(3)];
plot(state.time, wrapTo2Pi(state.signals.values(:,3)),'Color','b','LineWidth', 0.6); hold on;
plot(state.time, wrapTo2Pi(to_plot),'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, $\theta$", 'interpreter','latex');
legend({"$\theta(t)$","$\theta_d(t)$"}, 'Fontsize',10, 'Location','northwest');
xlabel("time [s]", 'interpreter','latex'),
ylabel("$\theta$ [rad]", 'interpreter','latex');
ylim([-0.5,7]);
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/state_theta.pdf','ContentType','vector');
close all
%% Polar coordinates

plot(polar.time, polar.signals.values(1,:),'Color','#77AC30','LineWidth', 0.6); hold on;
plot(polar.time, polar.signals.values(2,:),'Color','b','LineWidth', 0.6); grid on;
plot(polar.time, polar.signals.values(3,:),'Color','m','LineWidth', 0.6); hold off;
title("Polar coordinates", 'interpreter','latex');
legend({"$\rho(t)$","$\gamma(t)$", "$\delta(t)$"}, 'Fontsize',10, 'Location','northeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("angles [rad]", 'interpreter','latex');
ylim([-3,3]);
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/polar.pdf','ContentType','vector');
close all

%% Control inputs d = 0.4 a=8
figure()
plot(inputs.time(2:3015), inputs.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); hold on; grid on;
plot(inputs.time(2:3015), inputs.signals.values(2:3015,2),'Color','b','LineWidth', 0.6); hold off;
title("Control inputs $\xi$=0.4 $a$=8", 'interpreter','latex');
legend({"$v(t)$ [m/s]","$\omega(t)$ [rad/s]"}, 'Fontsize',10, 'Location','northeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("u(t)", 'interpreter','latex');
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/inputs_a8.pdf','ContentType','vector');
%% %% Control inputs d = 0.2 a=2
figure()
plot(inputs.time(2:3015), inputs.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); hold on; grid on;
plot(inputs.time(2:3015), inputs.signals.values(2:3015,2),'Color','b','LineWidth', 0.6); hold off;
title("Control inputs $\xi$=0.2 $a$=2", 'interpreter','latex');
legend({"$v(t)$ [m/s]","$\omega(t)$ [rad/s]"}, 'Fontsize',10, 'Location','northeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("u(t)", 'interpreter','latex');
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/inputs_a2.pdf','ContentType','vector');
%% error  d = 0.4 a=8
figure()
plot(error.time(2:3015), error.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); hold on; grid on;
plot(error.time(2:3015), error.signals.values(2:3015,2),'Color','b','LineWidth', 0.6);hold on; grid on;
plot(error.time(2:3015), error.signals.values(2:3015,3),'Color','r','LineWidth', 0.6);hold on; grid on;
title("Error $\xi$=0.4 $a$=8", 'interpreter','latex');
legend({"$e_{x}(t)$ [m]","$e_{y}(t)$ [m]","$e_{\theta}(t)$ [rad]"}, 'Fontsize',7, 'Location','southeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("e(t)", 'interpreter','latex');
set(gcf,'Position',[500 100 300 200]);
ylim([-2 0.5])
fig = gcf;
exportgraphics(fig,'plots/error_a8.pdf','ContentType','vector');
%% error  d = 0.2 a=2
figure()
plot(error.time(2:3015), error.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); hold on; grid on;
plot(error.time(2:3015), error.signals.values(2:3015,2),'Color','b','LineWidth', 0.6);hold on; grid on;
plot(error.time(2:3015), error.signals.values(2:3015,3),'Color','r','LineWidth', 0.6);hold on; grid on;
title("Error $\xi$=0.2 $a$=2", 'interpreter','latex');
legend({"$e_{x}(t)$ [m]","$e_{y}(t)$ [m]","$e_{\theta}(t)$ [rad]"}, 'Fontsize',7, 'Location','southeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("e(t)", 'interpreter','latex');
set(gcf,'Position',[500 100 300 200]);
ylim([-2 1.5])
fig = gcf;
exportgraphics(fig,'plots/error_a2.pdf','ContentType','vector');
%% 
% %% ex d = 0.4 a=8
% figure()
% plot(ex.time(2:3015), ex.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); grid on;
% title("$e_{x}$ $\xi$=0.4 $a$=8", 'interpreter','latex');
% xlabel("time [s]", 'interpreter','latex'),
% ylabel("e(t)", 'interpreter','latex');
% set(gcf,'Position',[500 100 300 200]);
% ylim([-2 2])
% fig = gcf;
% exportgraphics(fig,'plots/ex_a8.pdf','ContentType','vector');
% %% ey d = 0.4 a=8
% figure()
% plot(ey.time(2:3015), ey.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); grid on;
% title("$e_{y}$ $\xi$=0.4 $a$=8", 'interpreter','latex');
% xlabel("time [s]", 'interpreter','latex'),
% ylabel("e(t)", 'interpreter','latex');
% set(gcf,'Position',[500 100 300 200]);
% ylim([-2 2])
% fig = gcf;
% exportgraphics(fig,'plots/ey_a8.pdf','ContentType','vector');
% %% etheta d = 0.4 a=8
% figure()
% plot(etheta.time(2:3015), etheta.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); grid on;
% title("$e_{\theta}$ $\xi$=0.4 $a$=8", 'interpreter','latex');
% xlabel("time [s]", 'interpreter','latex'),
% ylabel("e(t)", 'interpreter','latex');
% set(gcf,'Position',[500 100 300 200]);
% ylim([-2 2])
% fig = gcf;
% exportgraphics(fig,'plots/etheta_a8.pdf','ContentType','vector');
% %% %% ex d = 0.2 a=2
% figure()
% plot(ex.time(2:3015), ex.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); grid on;
% title("$e_{x}$ $\xi$=0.2 $a$=2", 'interpreter','latex');
% xlabel("time [s]", 'interpreter','latex'),
% ylabel("e(t)", 'interpreter','latex');
% set(gcf,'Position',[500 100 300 100]);
% ylim([-2 2])
% fig = gcf;
% exportgraphics(fig,'plots/ex_a2.pdf','ContentType','vector');
% %% ey d = 0.2 a=2
% figure()
% plot(ey.time(2:3015), ey.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); grid on;
% title("$e_{y}$ $\xi$=0.2 $a$=2", 'interpreter','latex');
% xlabel("time [s]", 'interpreter','latex'),
% ylabel("e(t)", 'interpreter','latex');
% set(gcf,'Position',[500 100 300 200]);
% ylim([-2 2])
% fig = gcf;
% exportgraphics(fig,'plots/ey_a2.pdf','ContentType','vector');
% %% etheta d = 0.2 a=2
% figure()
% plot(etheta.time(2:3015), etheta.signals.values(2:3015,1),'Color','#77AC30','LineWidth', 0.6); grid on;
% title("$e_{\theta}$ $\xi$=0.2 $a$=2", 'interpreter','latex');
% xlabel("time [s]", 'interpreter','latex'),
% ylabel("e(t)", 'interpreter','latex');
% set(gcf,'Position',[500 100 300 200]);
% ylim([-2 2])
% fig = gcf;
% exportgraphics(fig,'plots/etheta_a2.pdf','ContentType','vector');
%% xd_dd
figure()
plot(xd_dd.time(1:827), xd_dd.signals.values(1:827,1),'Color','#77AC30','LineWidth', 0.6); hold on; grid on;
plot(xd_dd.time(1:827), xd_dd.signals.values(1:827,2),'Color','b','LineWidth', 0.6); hold off;
ylim([-500 500]);
title("$\ddot{x_{d}(t)}$", 'interpreter','latex');
legend({"$\ddot{x_{d}(t)}$ no sat","$\ddot{x_{d}(t)}$ sat"}, 'Fontsize',10, 'Location','northeast');
xlabel("time [s]", 'interpreter','latex'),
ylabel("$\ddot{x_{d}(t)}$", 'interpreter','latex');
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/xd_dd.pdf','ContentType','vector');