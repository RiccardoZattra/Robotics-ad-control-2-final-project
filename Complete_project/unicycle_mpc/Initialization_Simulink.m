
clear mex; close all; clear; clc;

addpath([pwd,'/nmpc']);
addpath([pwd,'/model_src']);
addpath([pwd,'/mex_core']);

%% Load settings from file
cd data;
if exist('settings','file')==2
    load('settings');
    cd ..
else 
    cd ..
    error('No setting data is detected!');
end

Ts  = settings.Ts_st;        % Sampling time

Ts_st   = settings.Ts_st;    % Shooting interval
nx      = settings.nx;       % No. of differential states
nu      = settings.nu;       % No. of controls
nz      = settings.nz;       % No. of algebraic states
ny      = settings.ny;       % No. of outputs (references)    
nyN     = settings.nyN;      % No. of outputs at terminal stage 
np      = settings.np;       % No. of parameters (on-line data)
nc      = settings.nc;       % No. of constraints
ncN     = settings.ncN;      % No. of constraints at terminal stage
nbu     = settings.nbu;      % No. of control bounds
nbx     = settings.nbx;      % No. of state bounds
nbu_idx = settings.nbu_idx;  % Index of control bounds
nbx_idx = settings.nbx_idx;  % Index of state bounds


%% Configure additional parameters for settings
N  = 100;
N2 = 5;
r  = 100;

settings.N = N;
settings.N2 = N2;
settings.r = r;

%% Specify options
opt.hessian='Gauss_Newton';  % 'Gauss_Newton', 'Generalized_Gauss_Newton'
opt.integrator='ERK4'; % 'ERK4','IRK3, 'IRK3-DAE'
opt.condensing='default_full';  %'default_full','no','blasfeo_full(require blasfeo installed)','partial_condensing'
opt.qpsolver='qpoases'; 
opt.hotstart='no'; %'yes','no' (only for qpoases)
opt.shifting='no'; % 'yes','no'
opt.ref_type=1; % 0-time invariant, 1-time varying(no preview), 2-time varying (preview)
opt.nonuniform_grid=0; % currently not supported 
opt.RTI = 'no'; % if use Real-time Iteration

% available qp solvers
%'qpoases' (for full condensing)
%'qpoases_mb' (for full condensing+moving block, please use ERK4 as the integrator)
%'hpipm_sparse' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
%'hpipm_pcond' (run mex_core/compile_hpipm.m first; set opt.condensing='no')
 
%% Initialization

    switch(settings.model)
        case 'InvertedPendulum'
            x0 = [0;pi;0;0];    
            u0 = zeros(nu,1); 
            z0 = zeros(nz,1);
            para0 = zeros(max(1,np),1);  
            
            W=repmat([10 10 0.1 0.1 0.01]',1,N);
            WN=W(1:nyN,1);
            
            % upper and lower bounds for states (=nbx)
            lb_x = -2;
            ub_x = 2;
            
            % upper and lower bounds for controls (=nbu)           
            lb_u = -20;
            ub_u = 20;
                                   
            % upper and lower bounds for general constraints (=nc)
            lb_g = [];
            ub_g = [];            
            lb_gN = [];
            ub_gN = [];

        case 'unicycle'
            x0 = [12;0;pi/2];    
            u0 = zeros(nu,1); 
            z0 = zeros(nz,1);
            para0 = zeros(max(1,np),1);  

           % W=repmat([0.1 0.1 0.05 0.001 0.01]',1,N); %stranger, ok for
            %n=5, dist = 0.5 
            W=repmat([0.1 0.1 0.01 0.1 0.1]',1,N); %stranger, ok for
            %n=5, dist = 0.5            
            %W=repmat([0.1 0.1 0.2 0.1 0.1]',1,N);
            WN=W(1:nyN,1);

            % upper and lower bounds for states (=nbx)
            lb_x = [];
            ub_x = [];

            % upper and lower bounds for controls (=nbu)           
            lb_u = -1000;
            ub_u = 1000;
                       
            % upper and lower bounds for general constraints (=nc)
            lb_g = [0.5];
            ub_g = [2000];            
            lb_gN = [0.5];
            ub_gN = [2000];               
    end

 
%% Setup variables according to specified intialization

lb = repmat(lb_g,N,1);
ub = repmat(ub_g,N,1);
lb = [lb;lb_gN];
ub = [ub;ub_gN];
if isempty(lb)
    lb=0;
    ub=0;
end
        
lbu = -inf(nu,1);
ubu = inf(nu,1);
for i=1:nbu
    lbu(nbu_idx(i)) = lb_u(i);
    ubu(nbu_idx(i)) = ub_u(i);
end
        
lbu = repmat(lbu,1,N);
ubu = repmat(ubu,1,N);

lbx = repmat(lb_x,1,N+1);
ubx = repmat(ub_x,1,N+1);
if isempty(lbx)
    lbx=0;
    ubx=0;
end

x = repmat(x0,1,N+1);   
u = repmat(u0,1,N);  
z = repmat(z0,1,N);
para = repmat(para0,1,N+1);  

if isempty(z)
    z0=0;
    z=0;
end

%% Reference specification
switch(settings.model)
    case 'unicycle'
        data.Tf  = 16;
        data.REF = zeros(data.Tf/Ts+1,ny);
        ref_t = 0:Ts:data.Tf;
        data.REF(:,1) = 10*cos(0.5*ref_t);
        data.REF(:,2) = 10*sin(0.5*ref_t);
        data.REF(:,3) = unwrap(atan2(10*0.5*cos(0.5*ref_t),-10*0.5*sin(0.5*ref_t)));
end