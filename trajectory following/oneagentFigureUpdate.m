function oneagentFigureUpdate(x,N_veh,cntr,L,W,enable_CG_trace,enable_rearAxle_trace,save_anim_frames,axisMode,plotAxisLimits,maxa,enable_CU_trace,enable_CD_trace,fname)

%{
X     = u(      1      :  1*N_veh ); % (m) object positions in global XY frame
Y     = u( (1*N_veh+1) :  2*N_veh ); % (m) object positions in global XY frame
yaw   = u( (2*N_veh+1) :  3*N_veh ); % (rad) vehicle's yaw orientations about global Z axis
delta = u( (3*N_veh+1) :  4*N_veh ); % (rad) front tire's steered angle (w.r.t. vehicle yaw angle)
%}
i = 1 : N_veh;
X     = x(4*(i-1)+1); % (m) object positions in global XY frame
Y     = x(4*(i-1)+2); % (m) object positions in global XY frame
yaw   = x(4*(i-1)+3); % (rad) vehicle's yaw orientations about global Z axis
delta = x(4*(i-1)+4); % (rad) front tire's steered angle (w.r.t. vehicle yaw angle)

%
% Retrieve figure object handles
%
    figStr = "Figure for animation of cars";
    handle = get(findobj('type','figure','Tag',figStr),'userdata'); % retrieve all graphics objects handles
    aspects = pbaspect;
    %set(gca,'DataAspectRatio',aspects)
    rx = aspects(1);
    ry = aspects(2);
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
         fprintf('axes userdata is missing - exiting')
         return
      end
   else
      fprintf('axes handle is missing - exiting at')
      return
   end
   
   % pull out the veh vertices
   veh_verts    = pass_these_verts{1};
   tire_verts   = pass_these_verts{2};
         
   % now do all the same translation and rotation for all N_veh sets of vehicle objects
   % ----------------------------------------------------
   for i=1:N_veh
       % first: retrieve the i'th object's vertices (vehicle and rear two tires all have same)
       verts_veh_xy     = [veh_verts]; % pick off the i'th [X,Y] column pair
       verts_tire_rf_xy = [tire_verts]; % all 4 tires use identical vertices prior to XY positioning
       verts_tire_lf_xy = [tire_verts];
       verts_tire_rr_xy = [tire_verts];
       verts_tire_lr_xy = [tire_verts];
       [a_veh,b]  = size(verts_veh_xy);     % a_veh is number of vertices in vehicle object (10 for vehicle_object2(1,[]) )
       [a_tire,b] = size(verts_tire_rf_xy); % b_vehicle is number of vertices in tire object (12 for vehicle_object2(2,[]) )

       attitude         = [ cos(yaw(i))   -sin(yaw(i))  ;  sin(yaw(i))    cos(yaw(i))  ]; % transformation matrix from body-fixed to global or terrain frame
       attitude_steered = [ cos(delta(i)) -sin(delta(i)) ;  sin(delta(i))  cos(delta(i))]; % transformation matrix from body-fixed to global or terrain frame

       % do the schmack: translate in body-fixed xy, rotate about yaw(i) with 'attitude', then translate in XY to the terrain frame's [X(i),Y(i)] position
       verts_veh_XY     = attitude*[verts_veh_xy' ] + [ X(i) ; Y(i)]*ones(1,a_veh );
       verts_tire_rr_XY = attitude*[verts_tire_rr_xy' + [ -L/2.6; -W/2.6]*ones(1,a_tire)] + [ X(i) ; Y(i)]*ones(1,a_tire);
       verts_tire_lr_XY = attitude*[verts_tire_rr_xy' + [ -L/2.6; +W/2.6]*ones(1,a_tire)] + [ X(i) ; Y(i)]*ones(1,a_tire);
%2.6
       % front tires require special consideration: rotate by steer angle first, then translate in xy, rotate by yaw, then translate in XY 
       verts_tire_rf_xy_steered = attitude_steered*[verts_tire_rf_xy'];
       verts_tire_rf_XY = attitude*[verts_tire_rf_xy_steered + [L/2.6;-W/2.6]*ones(1,a_tire)] + [X(i);Y(i)]*ones(1,a_tire);

       verts_tire_lf_xy_steered = attitude_steered*[verts_tire_lf_xy'];
       verts_tire_lf_XY = attitude*[verts_tire_lf_xy_steered + [L/2.6;+W/2.6]*ones(1,a_tire)] + [X(i);Y(i)]*ones(1,a_tire);


       % update the figure window object with the new position and orientation
       set(handle.veh(i),'Vertices',verts_veh_XY');
       set(handle.veh_text(i),'Position',[X(i);Y(i)]);
       set(handle.tire_rf(i),'Vertices',verts_tire_rf_XY'); % set graphics handle vertices to vertices just rotated and translated
       set(handle.tire_lf(i),'Vertices',verts_tire_lf_XY');
       set(handle.tire_rr(i),'Vertices',verts_tire_rr_XY');
       set(handle.tire_lr(i),'Vertices',verts_tire_lr_XY');
   end  
   %
% Update Line Objects
%
   if (enable_CG_trace==1)
      initState = cntr; % 0 the first time through only
      %str=sprintf('sanim_tracked_vehicle.m: initState=%i',initState);disp(str)
      if initState>=1 % tack on the current vehicle positions to the vehicle line trace
          for i=1:N_veh
              xLine = get(handle.line_CG(i),'XData');  
              yLine = get(handle.line_CG(i),'YData');

              % use the graphics line objects XData and YData to store and display a growing set of line points
              set(handle.line_CG(i),'Xdata',[xLine X(i)],'Ydata',[yLine Y(i)]);
          end
      else % init==0 so create first line point from vehicle IC's coming in from the UDP client s-function (not the x0 initialized with zeros in this s-function)
         for i=1:N_veh
             set(handle.line_CG(i),'Xdata',X(i),'Ydata',Y(i)); % make first entry in all lines be IC's
         end
         
      end
   end % if Config.enable_CG_trace==1
   
   
   if (enable_rearAxle_trace==1)
      initState = cntr; % 0 the first time through only
      %str=sprintf('sanim_tracked_vehicle.m: initState=%i',initState);disp(str)
      if initState >= 1  % tack on the current vehicle positions to the vehicle line trace
          for i=1:N_veh
              attitude            = [ cos(yaw(i))   -sin(yaw(i))   ;  sin(yaw(i))    cos(yaw(i))  ]; % transformation matrix from body-fixed to global or terrain frame
              line_rearAxle_verts = attitude*[-L/2 ; 0 ] + [X(i);Y(i)]; % [X,Y] pair in i'th column
            
              xLine = get(handle.line_rearAxle(i),'XData');  
              yLine = get(handle.line_rearAxle(i),'YData');
                        
              % use the graphics line objects XData and YData to store and display a growing set of line points
              set(handle.line_rearAxle(i),'Xdata',[xLine line_rearAxle_verts(1)],'Ydata',[yLine line_rearAxle_verts(2)]);
          end
            
      else % init==0 so create first line point from vehicle IC's coming in from the UDP client s-function (not the x0 initialized with zeros in this s-function)
     
         for i=1:N_veh
             attitude            = [ cos(yaw(i))   -sin(yaw(i))   ;  sin(yaw(i))    cos(yaw(i))  ]; % transformation matrix from body-fixed to global or terrain frame
             line_rearAxle_verts = attitude*[-L/2 ; 0 ] + [X(i);Y(i)]; % [X,Y] pair in i'th column
             set(handle.line_rearAxle(i),'Xdata',line_rearAxle_verts(1),'Ydata',line_rearAxle_verts(2));
         end
         
      end
   end
   %{
%Update the limits of the road 
   if (enable_CU_trace==1)
      axis(handle.axes(1),'tight'); % this resets the axes to capture all objects
      axisTight=axis(handle.axes(1)); % capture those new minimum limits
      sys(1)=min(min(X),axisTight(1)); % new Xmin as smaller of auto (tight axis) or X_ic
      sys(2)=max(max(X),axisTight(2));
      %str=sprintf('sanim_tracked_vehicle.m: initState=%i',initState);disp(str)      
      xLine = get(handle.line_uppery,'XData');  
      yLine = get(handle.line_uppery,'YData');
      % use the graphics line objects XData and YData to store and display a growing set of line points
      xs = sys(1):1:sys(2);
      ys = maxa*ones(size(xs));
      set(handle.line_uppery,'Xdata',xs,'Ydata',ys);    
   end 
   if (enable_CD_trace==1)
      axis(handle.axes(1),'tight'); % this resets the axes to capture all objects
      axisTight=axis(handle.axes(1)); % capture those new minimum limits
      sys(1)=min(min(X),axisTight(1)); % new Xmin as smaller of auto (tight axis) or X_ic
      sys(2)=max(max(X),axisTight(2));
      %str=sprintf('sanim_tracked_vehicle.m: initState=%i',initState);disp(str)      
      xLine = get(handle.line_downy,'XData');  
      yLine = get(handle.line_downy,'YData');
      % use the graphics line objects XData and YData to store and display a growing set of line points
      xs = sys(1):1:sys(2);
      ys = -maxa*ones(size(xs));
      set(handle.line_downy,'Xdata',xs,'Ydata',ys);
   end 
   %}
 % if Config.enable_rearAxle_trace==1
   
   
   % -------------------------------------------
   % --------  update the axis limits  --------
   % -------------------------------------------
   
   % this is where we grow the axis limits but never shrink - it captures
   % all objects and zooms out but does not pan or shrink limits (looks better)
   if (axisMode==0) % -> GROW-TO-FIT from initial user-supplied axis limits
      axis(handle.axes(1),'tight'); % this resets the axes to capture all objects
      axisTight=axis(handle.axes(1)); % capture those new minimum limits
      sys(1)=min(min(X),axisTight(1)); % new Xmin as smaller of auto (tight axis) or X_ic
      sys(2)=max(max(X),axisTight(2)); % new Xmax as larger of auto (tight axis) or X_ic
      sys(3)=min(min(Y),axisTight(3)); % new Ymin as smaller of auto (tight axis) or Y_ic
      sys(4)=max(max(Y),axisTight(4)); % new Ymax as larger of auto (tight axis) or Y_ic
      
      % at this point the axis limits are captured but not assigned.      
      axis(handle.axes(1),sys(1:4)); % so make the assignment to the graphics window
      
   else % -> FIXED-ONLY from user supplied initial axis limits
      axis(handle.axes(1),plotAxisLimits);
   end
   
%
% Force MATLAB to Update Drawing
%
   %set(handle.axes(1),'visible','off')
   %drawnow

% make a sequence of animation frames for a movie?
if save_anim_frames==1      
   % create jpg filename string using datetime() function in Simulink
   % dialogue box - this creates a single animation sequence using the date
   % and time from when the Simulink model was started
   anim_frame_name_str = "frame";
   imgFileStr=sprintf('%s_img_%0.6i.jpg',anim_frame_name_str,cntr); % note: %0.6i pads with leading zeros just like writeVideo() demo
   % prepend a folder to contain all these animation sequence images
   myFile = fullfile(fname,imgFileStr);
   str=sprintf('saving image sequence [%s]',myFile);
   disp(str)   
   % write this graphics frame to a file
   print('-opengl','-djpeg',myFile);
end

end