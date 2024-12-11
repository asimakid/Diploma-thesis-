function [sys,x0,str,ts] = sanim_XY_vehicle_viz(t,x,u,flag,Config)
% sanim_XY_vehicle_viz() - animate a 2D vehicle using SAE coordinates.
%
% This is a modified version of the Mathwork's sanim.m for animating a vehicle.
%
% Marc Compere, comperem@gmail.com
% created : 30 July 2011
% modified: 22 Apr 2018
%
%
% Edited from the original file:
% SANIM.m S-Function for displaying 6DoF trajectories
%
% See also:  Simulink library file 'aerospace.mdl' and sanim.m
%
% Copyright 1990-2002 The MathWorks, Inc.
% J.Hodgson  
% $Revision: 1.12 $  $Date: 2002/04/09 19:37:27 $

% for command line development:
%x=[0 10 0 10 0]
%u=[0 0 0]


switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
     [sys,x0,str,ts]=mdlInitializeSizes(Config);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case {1, 3, 9},
     sys=[];
     
  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
     sys = [];
     sys=mdlUpdate(t,x,u,Config);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u,Config.Ts);

otherwise
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%

   error(['Unhandled flag = ',num2str(flag)]);

end

end % sanim_XY_pairs()
%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(Config)

%
% Set Sizes Matrix
%
sizes = simsizes;

sizes.NumContStates  = 0;
sizes.NumDiscStates  = 6; % x(1:4) => [xmin xmax ymin ymax] when axisMode==0 (auto),
                          % x(5)=>initState for line trace setup is ZERO at t=0 and >0 for t>0
                          % x(6) counter for creating animation frame sequences
sizes.NumOutputs     = 0;
sizes.NumInputs      = 4; % [X_veh, Y_veh, psi_veh, delta_steer] in the global XY frame
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialise the initial conditions
%
%x0  = [];

%
% str is always an empty matrix
%
str = [];

%
% initialise the array of sample times
%
%ts  = [.1 0]; % Sample period of 0.1 seconds (10Hz)
ts = [Config.Ts 0]; % inherited


%
% Initialise Figure Window
%
   figStr=sprintf('XY anim fig%i',Config.windowNum);
   h_f=findobj('type','figure','Tag',figStr);
   
   if isempty(h_f)
     h_anim=figure(Config.windowNum);
   else
     h_anim=h_f;
   end

   % Figure 'position' args -> [left, bottom, width, height]
   % put the keyboard input figure right in the upper middle
   screen_size=get(0,'ScreenSize'); % [left, bottom, width, height] (in pixels)
   set_figure_window_size=0;
   if set_figure_window_size==1,
      figure_width=672; figure_height=504; % (pixels) Matlab defaults are 560x420(?) or... 672x504 on my laptop
      figure_left = screen_size(3) - figure_width - 10; % (pixels) almost all the way to the left side of the screen
      figure_bottom = screen_size(4) - figure_height - 100; % (pixels) almost all the way to the top of the screen
      set(h_anim,'name',figStr, ...
                 'renderer','OpenGL', ...
                 'clipping','off', ...
                 'position',[figure_left figure_bottom figure_width figure_height],...
                 'Tag',figStr);
   else
      set(h_anim,'name',figStr, ...
                 'renderer','OpenGL', ...
                 'clipping','off', ...
                 'Tag',figStr);
   end
%Painters
%Zbuffer
%OpenGL 

   if ~isempty(h_anim)                       % if there's a figure window..
      h_del = findobj(h_anim,'type','axes'); % find the axes..
      delete(h_del);                         % delete the axes..
      figure(h_anim);                        % bring figure to the front
   end

%
% Initialize graphics display axes
%
   handle.axes(1)=axes;
   set(handle.axes(1),...
           'visible','on','box','off', ...
           'units','normal', ...
           'position',[0.1 0.1 0.75 0.75], ...
           'Color',[.8 .8 .8], ...
           'clipping','off',...
           'XMinorTick','on',...
           'YMinorTick','on',...
           'Tag',figStr);
       
    % this reverses the direction of increasing Y values
    % to make Matlab figure window conform to SAE coordinates where:
    %    +X is to the right (as usual)
    %    +Y is down
    %    +Z is into the paper
    set(handle.axes(1),'YDir','reverse')
    set(handle.axes(1),'CameraUpvector',[0 -1 0])
    
    % set axes to initial [xmin xmax ymin ymax]
    axis(Config.ax);
    x0=axis; % assign iniital state to current axis limits
    x0(5)=0; % initState==0 at t=0 and >0 for t>0
    x0(6)=0; % animation frame counter (only writes .jpegs if save_anim_frames==1)
    grid on
    axis equal
   
    xlabel('SAE X_{gnd} (m)')
    ylabel('SAE Y_{gnd} (m)')
   
    
%
% Initialize snail trail objects (CG line trace and rearAxle trace)
%
   cmap_CG = colormap(cool(1)); % run 'colormapeditor' then choose in Tools | Standard Colormaps for examples
   if (Config.enable_CG_trace==1),
      line_width = 2;
      handle.line_CG = line(0,0); % N times with the client using each of the N lines for each server restart
      set(handle.line_CG,'linestyle','-','color',cmap_CG,'userdata',0,'clipping','off','LineWidth',line_width); % create line object for trace indicating where the Nth agent has been
   end
   
   cmap_rearAxle = colormap(spring(1)); % colormaps: hsv, gray, hot, cool, copper, pink, flag, jet, autumn, spring, summer, winter
   if (Config.enable_rearAxle_trace==1),
      line_width = 2;
      handle.line_rearAxle = line(0,0); % N times with the client using each of the N lines for each server restart
      set(handle.line_rearAxle,'linestyle','-.','color',cmap_rearAxle,'userdata',0,'clipping','off','LineWidth',line_width); % create line object for trace indicating where the Nth agent has been
   end
   
%
% Initialize vehicle object trajectories (chassis, front and rear tires)
%
   % make a generic set of vertices and faces for a 2D vehicle object
   veh=veh_object2(1,Config.L,Config.W); % see 'help patch' for how to make patch graphics objects
   cmap_veh = colormap(summer(1)); % colormaps: hsv, gray, hot, cool, copper, pink, flag, jet, autumn, spring, summer, winter
   handle.veh = patch('Vertices',veh.vertices','Faces',veh.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh,'EdgeColor',[0 0 0],'FaceAlpha',0.1);
   X_loc=sum(veh.vertices(1,:))/length(veh.vertices(1,:));
   Y_loc=sum(veh.vertices(2,:))/length(veh.vertices(2,:));
   handle.veh_text = text(X_loc,Y_loc,'veh','FontSize',10,'HorizontalAlignment','center','VerticalAlignment','middle'); % default FontSize is 10
   
   % create tire vertices, then make 4 different graphics patch objects to move around independently
   tire=veh_object2(2,Config.L/5,Config.W/5);
   handle.tire_rf = patch('Vertices',tire.vertices','Faces',tire.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh,'EdgeColor',[1 1 1],'FaceAlpha',0.1);
   handle.tire_lf = patch('Vertices',tire.vertices','Faces',tire.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh,'EdgeColor',[1 1 1],'FaceAlpha',0.1);
   handle.tire_rr = patch('Vertices',tire.vertices','Faces',tire.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh,'EdgeColor',[1 1 1],'FaceAlpha',0.1);
   handle.tire_lr = patch('Vertices',tire.vertices','Faces',tire.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh,'EdgeColor',[1 1 1],'FaceAlpha',0.1);
   
   
   % we must store all unmodified vertices in their local coordinate
   % frame so mdlUpdate() can orient and position in the global
   % then store them in the AXES UserData (not the figure window's UsersData)
   pass_these_verts{1} = get(handle.veh ,'Vertices');
   pass_these_verts{2} = get(handle.tire_rf,'Vertices');
   set(handle.axes(1),'userdata',pass_these_verts); % store veh object vertices for retrieval in mdlUpdate() below

%
% Set Handles of graphics in FIGURE UserData
%   
   set(h_anim,'UserData',handle); % store axes, line, and veh objects just created for retrieval in mdlUpdate()

end % mdlInitializeSizes
%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u,Config);

sys=x; % initialize outputs

X     = u(1); % (m) object positions in global XY frame
Y     = u(2); % (m) object positions in global XY frame
yaw   = u(3); % (rad) vehicle's yaw orientations about global Z axis
delta = u(4); % (rad) front tire's steered angle (w.r.t. vehicle yaw angle)

%
% Retrieve figure object handles
%
    figStr=sprintf('XY anim fig%i',Config.windowNum);
    handle = get(findobj('type','figure','Tag',figStr),'userdata'); % retrieve all graphics objects handles
    
    if isempty(findobj('type','figure','Tag',figStr))
     fprintf('\n\n\t\tfigure has been manually closed\n')
     return
    end

%
% Update all object positions
%

   if ~isempty(handle.axes(1))
      % retrieve the object vertices from the figure window's AXES UserData
      pass_these_verts=get(handle.axes(1),'UserData');
      if isempty(pass_these_verts)
         fprintf('axes userdata is missing - exiting at t=%f',t)
         return
      end
   else
      fprintf('axes handle is missing - exiting at t=%f',t)
      return
   end
   
   % pull out the veh vertices
   veh_verts    = pass_these_verts{1};
   tire_verts   = pass_these_verts{2};
         
   % now do all the same translation and rotation for the vehicle objects
   % ----------------------------------------------------
   % first: retrieve the i'th object's vertices (vehicle and rear two tires all have same)
   verts_veh_xy     = [veh_verts]; % pick off the i'th [X,Y] column pair
   verts_tire_rf_xy = [tire_verts]; % all 4 tires use identical vertices prior to XY positioning
   verts_tire_lf_xy = [tire_verts];
   verts_tire_rr_xy = [tire_verts];
   verts_tire_lr_xy = [tire_verts];
   [a_veh,b]  = size(verts_veh_xy);     % a_veh is number of vertices in vehicle object (10 for vehicle_object2(1,[]) )
   [a_tire,b] = size(verts_tire_rf_xy); % b_vehicle is number of vertices in tire object (12 for vehicle_object2(2,[]) )

   attitude         = [ cos(yaw)   -sin(yaw)   ;  sin(yaw)    cos(yaw)  ]; % transformation matrix from body-fixed to global or terrain frame
   attitude_steered = [ cos(delta) -sin(delta) ;  sin(delta)  cos(delta)]; % transformation matrix from body-fixed to global or terrain frame

   % do the schmack: translate in body-fixed xy, rotate about yaw(i) with 'attitude', then translate in XY to the terrain frame's [X(i),Y(i)] position
   verts_veh_XY     = attitude*[verts_veh_xy' ]                                                  + [ X ; Y]*ones(1,a_veh );
   verts_tire_rr_XY = attitude*[verts_tire_rr_xy' + [ -Config.L/2 ; -Config.W/2]*ones(1,a_tire)] + [ X ; Y]*ones(1,a_tire);
   verts_tire_lr_XY = attitude*[verts_tire_rr_xy' + [ -Config.L/2 ; +Config.W/2]*ones(1,a_tire)] + [ X ; Y]*ones(1,a_tire);

   % front tires require special consideration: rotate by steer angle first, then translate in xy, rotate by yaw, then translate in XY 
   verts_tire_rf_xy_steered = attitude_steered*[verts_tire_rf_xy'];
   verts_tire_rf_XY = attitude*[verts_tire_rf_xy_steered + [Config.L/2;-Config.W/2]*ones(1,a_tire)] + [X;Y]*ones(1,a_tire);

   verts_tire_lf_xy_steered = attitude_steered*[verts_tire_lf_xy'];
   verts_tire_lf_XY = attitude*[verts_tire_lf_xy_steered + [Config.L/2;+Config.W/2]*ones(1,a_tire)] + [X;Y]*ones(1,a_tire);


   % update the figure window object with the new position and orientation
   set(handle.veh,'Vertices',verts_veh_XY');
   set(handle.veh_text,'Position',[X;Y]);

   set(handle.tire_rf,'Vertices',verts_tire_rf_XY'); % set graphics handle vertices to vertices just rotated and translated
   set(handle.tire_lf,'Vertices',verts_tire_lf_XY');
   set(handle.tire_rr,'Vertices',verts_tire_rr_XY');
   set(handle.tire_lr,'Vertices',verts_tire_lr_XY');
      

   

%
% Update Line Objects
%
   if (Config.enable_CG_trace==1),
      initState = x(5); % 0 the first time through only
      %str=sprintf('sanim_tracked_vehicle.m: initState=%i',initState);disp(str)
      if initState>=1, % tack on the current vehicle positions to the vehicle line trace
         xLine = get(handle.line_CG,'XData');  
         yLine = get(handle.line_CG,'YData');
                        
         % use the graphics line objects XData and YData to store and display a growing set of line points
         set(handle.line_CG,'Xdata',[xLine X],'Ydata',[yLine Y]);
         
      else % init==0 so create first line point from vehicle IC's coming in from the UDP client s-function (not the x0 initialized with zeros in this s-function)
         
         sys(5)=1; % cause 'initState' to no longer be zero
         
         set(handle.line_CG,'Xdata',X,'Ydata',Y);
         
      end
   end % if Config.enable_CG_trace==1
   
   
   if (Config.enable_rearAxle_trace==1),
      initState = x(5); % 0 the first time through only
      %str=sprintf('sanim_tracked_vehicle.m: initState=%i',initState);disp(str)
      if initState>=1, % tack on the current vehicle positions to the vehicle line trace
          
            line_rearAxle_verts = attitude*[-Config.L/2 ; 0 ] + [X;Y]; % [X,Y] pair in i'th column
            
            xLine = get(handle.line_rearAxle,'XData');  
            yLine = get(handle.line_rearAxle,'YData');
                        
            % use the graphics line objects XData and YData to store and display a growing set of line points
            set(handle.line_rearAxle,'Xdata',[xLine line_rearAxle_verts(1)],'Ydata',[yLine line_rearAxle_verts(2)]);
            
      else % init==0 so create first line point from vehicle IC's coming in from the UDP client s-function (not the x0 initialized with zeros in this s-function)

         sys(5)=1; % cause 'initState' to no longer be zero
         
         line_rearAxle_verts = attitude*[-Config.L/2 ; 0 ] + [X;Y]; % [X,Y] pair in i'th column
         
         set(handle.line_rearAxle,'Xdata',line_rearAxle_verts(1),'Ydata',line_rearAxle_verts(2));
         
      end
   end % if Config.enable_rearAxle_trace==1
   
   
   % -------------------------------------------
   % --------  update the axis limits  --------
   % -------------------------------------------
   sys(1:4)=x(1:4); % init the first 4 discrete state (updated immmediately below)
   
   % this is where we grow the axis limits but never shrink - it captures
   % all objects and zooms out but does not pan or shrink limits (looks better)
   if (Config.axisMode==0), % -> GROW-TO-FIT from initial user-supplied axis limits
      axis(handle.axes(1),'tight'); % this resets the axes to capture all objects
      axisTight=axis(handle.axes(1)); % capture those new minimum limits
      sys(1)=min(X,axisTight(1)); % new Xmin as smaller of auto (tight axis) or X_ic
      sys(2)=max(X,axisTight(2)); % new Xmax as larger of auto (tight axis) or X_ic
      sys(3)=min(Y,axisTight(3)); % new Ymin as smaller of auto (tight axis) or Y_ic
      sys(4)=max(Y,axisTight(4)); % new Ymax as larger of auto (tight axis) or Y_ic
      
      % at this point the axis limits are captured but not assigned..
      axis(handle.axes(1),sys(1:4)); % so make the assignment to the graphics window
      
   else % -> FIXED-ONLY from user supplied initial axis limits
      axis(handle.axes(1),Config.ax);
   end
   
%
% Force MATLAB to Update Drawing
%
   %set(handle.axes(1),'visible','off')
   %drawnow

% make a sequence of animation frames for a movie?
if Config.save_anim_frames==1
   
   % increment state x(6) for current frame number
   sys(6) = x(6) + 1;
   
   % create jpg filename string using datetime() function in Simulink
   % dialogue box - this creates a single animation sequence using the date
   % and time from when the Simulink model was started
   imgFileStr=sprintf('%s_img_%0.6i.jpg',Config.anim_frame_name_str,sys(6)); % note: %0.6i pads with leading zeros just like writeVideo() demo
   
   % prepend a folder to contain all these animation sequence images
   myFile = fullfile('anim_sequences',imgFileStr);
   str=sprintf('saving image sequence [%s]',myFile); disp(str)
   
   % write this graphics frame to a file
   print('-opengl','-djpeg',myFile);
end

end % mdlUpdate

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  
%=============================================================================
%
%function sys=mdlGetTimeOfNextVarHit(t,x,u,Ts)
    
%    sys = ceil(t/Ts)*Ts+Ts;

% end mdlGetTimeOfNextVarHit



