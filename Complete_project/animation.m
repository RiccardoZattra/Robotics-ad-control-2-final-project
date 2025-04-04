%% Plot animated trajectory 
%Video on how to do it
%https://www.youtube.com/watch?v=3I1_5M7Okqo

function animation(state,desired_trajectory, t, obstacle_position, parking_position)
close all

x1 = state.signals.values(:,1);
y1 = state.signals.values(:,2);

x_traj = desired_trajectory.signals.values(1:947,1);
y_traj = desired_trajectory.signals.values(1:947,2);

unicycleTranslations1 = [state.signals.values(:,1:2) zeros(length(state.signals.values(:,1:2)),1)];
unicycleRot1 = axang2quat([repmat([0 0 1],length(state.signals.values(:,3)),1) state.signals.values(:,3)]);

figure 
for k=1:10:length(t)
    clf
    hold all
    plot(x_traj, y_traj, 'Color','#d3d3d3', 'LineWidth', 5);
    plot(x1(1:k), y1(1:k), 'b--', 'LineWidth', 1)
    plot(obstacle_position(1), obstacle_position(2), 'r*', 'LineWidth', 10)
    plot(parking_position(1), parking_position(2), 'g*', 'LineWidth', 10)

    plotTransforms(unicycleTranslations1(k,:), ...
        unicycleRot1(k,:), ...
        MeshFilePath="groundvehicle.stl", ...
        MeshColor="blue", ...
        View="2D", ...
        FrameSize=0.8);
    
    grid on
    xlabel('x (m)', 'interpreter','latex')
    ylabel('y (m)', 'interpreter','latex')
    xlim([-12,15]);
    ylim([-12,12]);
    title(['\textbf{Unicycle trajectory}, $t = ', num2str(t(k), '%.1f'), '$ [s]'], 'interpreter','latex')
    %legend({"reference","actual","obstacle","parking spot"}, 'Fontsize',12, 'Location','southeast');
    drawnow
end  

end
