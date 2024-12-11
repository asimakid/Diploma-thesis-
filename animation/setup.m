% Setup file for vehicle_animation_sim_multi.slx which is a simple 2D
% kinematic steering vehicle model and animation for N_veh vehicles.
% See Readme.mdc.txt for more.
%
% This file should always be run before the simulation.
%
% Marc Compere
% comperem@erau.edu
% created : 30 July 2011
% modified: 22 Apr 2018

clear; clear all ; clear functions

% ------------------------------------------------
% -------        vehicle parameters        -------
% ------------------------------------------------
N_veh = 4; % number of vehicles in animation

vehicle_length = 2.7*ones(1,N_veh); % (m)
%vehicle_length = 0.1; % (m)
vehicle_width  = 1.5*ones(1,N_veh); % (m)

circleIC=1; % (0/1) start out in a big circle?
if circleIC==1
    dTheta=(2*pi/N_veh);
    theta=0:dTheta:(2*pi-dTheta);
    R=10; % (m)
    X_ic = R*cos(theta); % (m)
    Y_ic = R*sin(theta); % (m)
    yaw_ic = theta+(pi/2);
    delta_ss = atan2(vehicle_length,R); % (rad) note: delta_ackermann = L/R
else
    X_ic = 1*[50*(rand(1,N_veh)-0.5)+3]; % (m) random ICs on X position, note: rand() is on [0 1]
    Y_ic = 1*[50*(rand(1,N_veh)-0.5)+0]; % (m) random ICs on Y position
    %yaw_ic = 2*pi*(rand(nAgents,1)-0.5); % (rad) random ICs for yaw, or heading
    yaw_ic = 1*(-pi/2)*rand(1,N_veh); % (rad) random ICs for yaw, or heading
end

% ---------------------------------------------
% -----  solver and animation parameters  -----
% ---------------------------------------------
h_fixed = 0.05; % (s) fixed solver simulation stepsize

plotAxisLimitMode = 0; % 0->auto, 1->fixed, use Axes Limits in 'plotAxisLimits'
%plotAxisLimits = [-3 5 -3 6]*20; % [xmin xmax ymin ymax]
plotAxisLimits = [-3 5 -3 6]*3; % [xmin xmax ymin ymax]

%anim_fps=20; % (animation frames / second)
%anim_fps=10; % (animation frames / second)
anim_fps=2; % (animation frames / second)
enable_CG_trace=1;       % (0/1) plot animation trace from vehicle CG, or geometric center
enable_rearAxle_trace=1; % (0/1) enable animation trace from rear axle

save_anim_frames=0; % (0/1) save animation frames? this slows the simulation considerably
                    %       when writing a .jpeg image to file at each animation interval. 
                    %       see writeVideo() at this link for converting into .avi movies:
                    %       http://www.mathworks.com/help/matlab/examples/convert-between-image-sequences-and-video.html)

% animation update rate assuming Simulink clock advances very nearly the wall clock
C = round(1/(anim_fps*h_fixed)); % see notes below
animation_update_interval=C*h_fixed; % (s) This parameter controls three things:
                                              %     (1) The animation update to the screen is updated at this
                                              %         rate which needs to be an integer multiple of fixed stepsize integration. 
                                              %     (2) A frame is saved on this interval during movie makin (i.e. when movie_parm.save_frames=1)
                                              %     (3) how close to (or how much faster than) real-time the simulation runs.
                                              %         See timing_notes.txt for more information on run speeds.

                                              
% bring up the simulink model
%vehicle_animation_sim
vehicle_animation_sim_multi



