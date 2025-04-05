clear all
clc
close all
%% Initialization
initial_condition = [12 0 pi/2];
obstacle_position = [0 10];
%parking_spot = [14 0 -pi/2];
parking_spot = [12, -2, pi/2]; % to show the backwards maneuver
use_posture_regulator = 0;

if(use_posture_regulator)
    sim_time = 15;
else
    sim_time = 12.2;
end    