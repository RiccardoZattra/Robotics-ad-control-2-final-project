%% Plot results

%% Animation
set(groot, 'defaulttextinterpreter','latex');
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
set(gca,'TickLabelInterpreter','latex');
close all

% animation(state,trajectory,tout, obstacle_position, parking_spot);

%% State evolution
close all
figure()
plot(tout, state_sim(:,1),'Color','b','LineWidth', 0.6); hold on;
plot(tout, ref(:,1),'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, x", 'interpreter','latex');
legend({"$x(t)$","$x_d(t)$"}, 'Fontsize',7, 'Location','northwest');
xlabel("time [s]", 'interpreter','latex'),
xlim([0, data.Tf]);
ylabel("x coordinate [m]", 'interpreter','latex');
set(gcf,'Position',[500 100 300 100]);
fig = gcf;
exportgraphics(fig,'plots/state_x_mpcn5_n100good.pdf','ContentType','vector');

figure()
plot(tout, state_sim(:,2),'Color','b','LineWidth', 0.6); hold on;
plot(tout, ref(:,2),'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, y", 'interpreter','latex');
legend({"$y(t)$","$y_d(t)$"}, 'Fontsize',7, 'Location','best');
xlabel("time [s]", 'interpreter','latex'),
xlim([0, data.Tf]);
ylabel("y coordinate [m]", 'interpreter','latex');
set(gcf,'Position',[500 100 300 100]);
fig = gcf;
exportgraphics(fig,'plots/state_y_mpcn5_n100good.pdf','ContentType','vector');

%%
close all
figure()
plot(tout, wrapTo2Pi(state_sim(:,3)),'Color','b','LineWidth', 0.6); hold on;
plot(tout, wrapTo2Pi(ref(:,3)),'r--','LineWidth', 0.6); hold off; grid on;
title("State evolution, $\theta$", 'interpreter','latex');
legend({"$\theta(t)$","$\theta_d(t)$"}, 'Fontsize',7, 'Location','best');
xlabel("time [s]", 'interpreter','latex'),
ylabel("$\theta$ [rad]", 'interpreter','latex');
ylim([-0.5,7]);
xlim([0, data.Tf]);
set(gcf,'Position',[500 100 300 100]);
fig = gcf;
exportgraphics(fig,'plots/state_thetan5_n100good.pdf','ContentType','vector');

%% Control inputs
figure()
stairs(tout, controls_MPC(:,1),'Color','#77AC30','LineWidth', 0.6); hold on; grid on;
stairs(tout, controls_MPC(:,2),'Color','b','LineWidth', 0.6); hold off;
title("Control inputs", 'interpreter','latex');
legend({"$v(t)$ [m/s]","$\omega(t)$ [rad/s]"}, 'Fontsize',7, 'Location','north');
xlabel("time [s]", 'interpreter','latex'),
ylabel("u(t)", 'interpreter','latex');
set(gcf,'Position',[500 100 300 100]);
xlim([0, data.Tf]);
fig = gcf;
exportgraphics(fig,'plots/inputs_mpcn5_n100good.pdf','ContentType','vector');

%% Trajectory
figure()

plot(state_sim(1:13.5/0.001,1), state_sim(1:13.5/0.001,2), 'b', 'LineWidth', 1); hold on;
plot(ref(1:12.5/0.001,1), ref(1:12.5/0.001,2), 'r--','LineWidth', 1); grid on;
plot(0,10, 'm*', 'LineWidth', 5);
xlabel('x (m)', 'interpreter','latex')
ylabel('y (m)', 'interpreter','latex')
xlim([-12.5,12.5]);
ylim([-12,12]);
title('Cartesian trajectory', 'interpreter','latex')
l=legend({"actual","reference","obstacle"}, 'Fontsize',10, 'Location','best');
set(l,'position',[0.5 0.5 0.07 0.07]);
set(gcf,'Position',[500 100 300 300]);
fig = gcf;
exportgraphics(fig,'plots/maneuver_mpcn5_n100good.pdf','ContentType','vector');    