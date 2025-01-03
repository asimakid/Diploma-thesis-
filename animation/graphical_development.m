% this script is designed to be run *after* setup.m
% and prepares the workspace for a line-by-line walk through of 
% sanim_XY_vehicle_viz.m
%
% Marc Compere, comperem@gmail.com
% created : 28 Dec 2015
% modified: 07 Feb 2016

%anim_frame_name_str=char(datetime('now','Format','yyyy-MM-dd_HH_mm_ss'));
anim_frame_name_str=sprintf('%s_%s_%s_%s_%s_%s',string( fix(clock) ));
windowNum=200;

% for veh_object2.m
object_type=1;
if ( exist('vehicle_length') && exist('vehicle_width') ),
   obj_length=vehicle_length;
   obj_width=vehicle_width;
else,
   vehicle_length=L;
   vehicle_width=w;
end

%plotAxisLimits = plotAxisLimits;

% for mdlInitializeSizes()
Config.axisMode=plotAxisLimitMode;
Config.ax=plotAxisLimits;
Config.Ts=animation_update_interval;
Config.enable_CG_trace=enable_CG_trace;
Config.enable_rearAxle_trace=enable_rearAxle_trace;
Config.save_anim_frames=save_anim_frames;
Config.anim_frame_name_str=anim_frame_name_str;
Config.L=vehicle_length;
Config.W=vehicle_width;

Config.windowNum = windowNum;
Config.N_veh=N_veh;

% for mdlUpdate()

% since N=1, just put in scalars
X   = 2*rand(1,N_veh); % (m) object positions in global XY frame
Y   = 6*rand(1,N_veh); % (m) object positions in global XY frame
yaw = pi/10*rand(1,N_veh); % (rad) object yaw orientations about global Z axis
delta = (pi/6)*rand(1,N_veh);

u=[X,Y,yaw,delta];

% init:
%[sys,x0,str,ts] = sanim_XY_vehicle_viz(0,0,0,0,Config)
[sys,x0,str,ts] = sanim_XY_vehicle_viz_multi(0,0,0,0,Config)
x=x0;

% mdlUpdate()
%[sys,x,str,ts] = sanim_XY_vehicle_viz(0,x0,[X, Y, yaw, delta],2,Config)
[sys,x,str,ts] = sanim_XY_vehicle_viz_multi(0,x0,[X, Y, yaw, delta],2,Config)









