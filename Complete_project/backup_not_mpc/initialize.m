clear all
clc
close all
%% Initialization
initial_condition = [12 0 pi/2];
obstacle_position = [0 10];
parking_spot = [12, -2, pi/2]; % or [12, -2, pi/2] to show the backwards maneuver [14 0 -pi/2]
use_posture_regulator = 1;

if(use_posture_regulator)
    sim_time = 15;
else
    sim_time = 12.2;
end    

%% Simulation
sim('unicycle_linearization_approach.slx', sim_time);

%% Animation
set(groot, 'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(gca,'TickLabelInterpreter','latex');
close all

% animation(state,trajectory,state.time, obstacle_position, parking_spot);

%% State evolution
close all
figure()
nonzero = nonzeros(trajectory.signals.values(:,1));
to_plot = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(1)];
plot(state.time, state.signals.values(:,1),'Color','b','LineWidth', 0.6); hold on;
plot(state.time, to_plot,'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, x", 'interpreter','latex');
legend({"$x(t)$","$x_d(t)$"}, 'Fontsize',7, 'Location','northwest');
xlabel("time [s]", 'interpreter','latex'),
xlim([0, sim_time]);
ylabel("x coordinate [m]", 'interpreter','latex');
set(gcf,'Position',[500 100 300 100]);
fig = gcf;
exportgraphics(fig,'plots/state_x_posture.pdf','ContentType','vector');

figure()
nonzero = nonzeros(trajectory.signals.values(:,2));
to_plot = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(2)];
plot(state.time, state.signals.values(:,2),'Color','b','LineWidth', 0.6); hold on;
plot(state.time, to_plot,'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, y", 'interpreter','latex');
legend({"$y(t)$","$y_d(t)$"}, 'Fontsize',7, 'Location','best');
xlabel("time [s]", 'interpreter','latex'),
xlim([0, sim_time]);
ylabel("y coordinate [m]", 'interpreter','latex');
set(gcf,'Position',[500 100 300 100]);
fig = gcf;
exportgraphics(fig,'plots/state_y_posture.pdf','ContentType','vector');

%%
close all
figure()
nonzero = nonzeros(wrapToPi(flatness.signals.values(:,5)));
to_plot = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(3)];
plot(state.time, wrapTo2Pi(state.signals.values(:,3)),'Color','b','LineWidth', 0.6); hold on;
plot(state.time, wrapTo2Pi(to_plot),'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, $\theta$", 'interpreter','latex');
legend({"$\theta(t)$","$\theta_d(t)$"}, 'Fontsize',7, 'Location','best');
xlabel("time [s]", 'interpreter','latex'),
ylabel("$\theta$ [rad]", 'interpreter','latex');
ylim([-0.5,7]);
xlim([0, sim_time]);
set(gcf,'Position',[500 100 300 100]);
fig = gcf;
exportgraphics(fig,'plots/state_theta_posture.pdf','ContentType','vector');

%% Polar coordinates
close all
plot(polar.time, polar.signals.values(1,:),'Color','#77AC30','LineWidth', 0.6); hold on;
plot(polar.time, polar.signals.values(2,:),'Color','b','LineWidth', 0.6); grid on;
plot(polar.time, polar.signals.values(3,:),'Color','m','LineWidth', 0.6); hold off;
title("Polar angles", 'interpreter','latex');
l = legend({"$\rho(t)$","$\gamma(t)$", "$\delta(t)$"}, 'Fontsize',7, 'Location','best');
set(l,'position',[0.8 0.8 0.07 0.07]);
xlabel("time [s]", 'interpreter','latex'),
ylabel("angles [rad]", 'interpreter','latex');
ylim([-3,3]);
set(gcf,'Position',[500 100 300 100]);
fig = gcf;
exportgraphics(fig,'plots/polar_traj.pdf','ContentType','vector');

%% Control inputs
figure()
plot(inputs.time(2:end), inputs.signals.values(2:end,1),'Color','#77AC30','LineWidth', 0.6); hold on; grid on;
plot(inputs.time(2:end), inputs.signals.values(2:end,2),'Color','b','LineWidth', 0.6); hold off;
title("Control inputs", 'interpreter','latex');
legend({"$v(t)$ [m/s]","$\omega(t)$ [rad/s]"}, 'Fontsize',7, 'Location','north');
xlabel("time [s]", 'interpreter','latex'),
ylabel("u(t)", 'interpreter','latex');
set(gcf,'Position',[500 100 300 100]);
xlim([0, sim_time]);
fig = gcf;
exportgraphics(fig,'plots/inputs_posture.pdf','ContentType','vector');

%% Trajectory
figure()
nonzero = nonzeros(trajectory.signals.values(:,1));
to_plot_x = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(1)];

nonzero = nonzeros(trajectory.signals.values(:,2));
to_plot_y = [nonzero; zeros(length(state.time)-length(nonzero),1)+parking_spot(2)];

plot(parking_spot(1), parking_spot(2), 'g*', 'LineWidth', 10); hold on;
plot(state.signals.values(:,1), state.signals.values(:,2), 'b', 'LineWidth', 1); hold on;
plot(trajectory.signals.values(:,1), trajectory.signals.values(:,2), 'r--','LineWidth', 1); grid on;
plot(obstacle_position(1), obstacle_position(2), 'm*', 'LineWidth', 10)
xlabel('x (m)', 'interpreter','latex')
ylabel('y (m)', 'interpreter','latex')
xlim([9.5,14.5]);
ylim([-1,2.5]);
title('Cartesian trajectory', 'interpreter','latex')
legend({"parking spot","actual","reference"}, 'Fontsize',10, 'Location','south');
set(gcf,'Position',[500 100 300 200]);
fig = gcf;
exportgraphics(fig,'plots/maneuver_traj_part.pdf','ContentType','vector');    