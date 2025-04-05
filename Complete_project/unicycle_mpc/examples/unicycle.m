%------------------------------------------%
% Inverted Pendulum 
  
% "Autogenerating microsecond solvers for nonlinear MPC: A tutorial
% using ACADO integrators", Quirynen, 2015

% typical configuration: 1) N=80, Ts=Ts_st=0.025, no shifting 2) N=40,
% Ts=Ts_st=0.05, shifting

%------------------------------------------%


%% Dimensions

nx=3;  % No. of differential states
nu=2;  % No. of controls
nz=0;  % No. of algebraic states
ny=5; % No. of outputs
nyN=3; % No. of outputs at the terminal point
np=0; % No. of model parameters
nc=1; % No. of general constraints
ncN=1; % No. of general constraints at the terminal point
nbx = 0; % No. of bounds on states
nbu = 1; % No. of bounds on controls

% state and control bounds
nbx_idx = []; % indexs of states which are bounded
nbu_idx = 1; % indexs of controls which are bounded

%% create variables

import casadi.*

states   = SX.sym('states',nx,1);   % differential states
controls = SX.sym('controls',nu,1); % control input
alg      = SX.sym('alg',nz,1);      % algebraic states
params   = SX.sym('paras',np,1);    % parameters
refs     = SX.sym('refs',ny,1);     % references of the first N stages
refN     = SX.sym('refs',nyN,1);    % reference of the last stage
Q        = SX.sym('Q',ny,1);        % weighting matrix of the first N stages
QN       = SX.sym('QN',nyN,1);      % weighting matrix of the last stage
aux      = SX.sym('aux',ny,1);      % auxilary variable
auxN     = SX.sym('auxN',nyN,1);    % auxilary variable

%% Dynamics

x=states(1);
y=states(2);
theta=states(3);
v=controls(1);
omega=controls(2);

% explicit ODE RHS
x_dot=[v*cos(theta);v*sin(theta);omega];  
 
% algebraic function
z_fun = [];                   

% implicit ODE: impl_f = 0
xdot = SX.sym('xdot',nx,1);
impl_f = xdot - x_dot;        
     
%% Objectives and constraints

% inner objectives
h = [x;y;theta;v;omega];
hN = h(1:nyN);

% outer objectives
obji = 0.5*(h-refs)'*diag(Q)*(h-refs);
objN = 0.5*(hN-refN)'*diag(QN)*(hN-refN);

obji_GGN = 0.5*(aux-refs)'*(aux-refs);
objN_GGN = 0.5*(auxN-refN)'*(auxN-refN);

% general inequality constraints
general_con = (y-10)^2+(x)^2;
general_con_N = (y-10)^2+(x)^2;

%% NMPC discretizing time length [s]

Ts_st = 0.05; % shooting interval time
