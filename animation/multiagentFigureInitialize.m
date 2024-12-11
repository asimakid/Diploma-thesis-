function fname = multiagentFigureInitialize(N_veh,L,W,enable_CG_trace,enable_rearAxle_trace,enable_CU_trace,enable_CD_trace,save_anim_frames)
%
% Initialise Figure Window
%
     h_anim=figure('units','normalized','outerposition',[0 0 1 1]);
     figStr = "Figure for animation of cars";
   % Figure 'position' args -> [left, bottom, width, height]
   % put the keyboard input figure right in the upper middle
   screen_size=get(0,'ScreenSize'); % [left, bottom, width, height] (in pixels)
      set(h_anim,'name',figStr, ...
                 'renderer','OpenGL', ...
                 'clipping','off', ...
                 'Tag',figStr);
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
    set(handle.axes(1),'YDir','normal')
    set(handle.axes(1),'CameraUpvector',[0 -1 0])
    % set axes to initial [xmin xmax ymin ymax]
    
    grid on
    axis equal
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset ;
left = -outerpos(1) + ti(1)-0.01;
bottom = -outerpos(2) + ti(2)+0.1;
ax_width = outerpos(3) - ti(1) - ti(3)+0.03;
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];
axis on
%xlabel('Legth (m)')
%ylabel('Width (m)')   
%
% Initialize snail trail objects (CG line trace and rearAxle trace)
%
  cmap_uppery = [0,0,0]; % run 'colormapeditor' then choose in Tools | Standard Colormaps for examples
   if (enable_CU_trace==1)
       line_width = 1;
       handle.line_uppery = line(0,0); % N times with the client using each of the N lines for each server restart
       set(handle.line_uppery,'linestyle','-','color',cmap_uppery,'userdata',0,'clipping','off','LineWidth',line_width); % create line object for trace indicating where the Nth agent has been
   end
   cmap_downy = [0,0,0]; % run 'colormapeditor' then choose in Tools | Standard Colormaps for examples
   if (enable_CD_trace==1)
       line_width = 1;
       handle.line_downy = line(0,0); % N times with the client using each of the N lines for each server restart
       set(handle.line_downy,'linestyle','-','color',cmap_downy,'userdata',0,'clipping','off','LineWidth',line_width); % create line object for trace indicating where the Nth agent has been
   end   
   cmap_CG = colormap(cool(N_veh)); % run 'colormapeditor' then choose in Tools | Standard Colormaps for examples
   if (enable_CG_trace==1)
       for i=1:N_veh
           line_width = 2;
           handle.line_CG(i) = line(0,0); % N times with the client using each of the N lines for each server restart
           set(handle.line_CG(i),'linestyle','-','color',cmap_CG(i,:),'userdata',0,'clipping','off','LineWidth',line_width); % create line object for trace indicating where the Nth agent has been
       end
   end
   
   cmap_rearAxle = colormap(spring(N_veh)); % colormaps: hsv, gray, hot, cool, copper, pink, flag, jet, autumn, spring, summer, winter
   if (enable_rearAxle_trace==1)
      line_width = 2;
      for i=1:N_veh
          handle.line_rearAxle(i) = line(0,0); % N times with the client using each of the N lines for each server restart
          set(handle.line_rearAxle(i),'linestyle','-.','color',cmap_rearAxle(i,:),'userdata',0,'clipping','off','LineWidth',line_width); % create line object for trace indicating where the Nth agent has been
      end
   end
   
%
% Initialize vehicle object trajectories (chassis, front and rear tires)
%
   % make a generic set of vertices and faces for a 2D vehicle object
   cmap_veh = colormap(summer(N_veh)); % colormaps: hsv, gray, hot, cool, copper, pink, flag, jet, autumn, spring, summer, winter
   for i=1:N_veh
       veh{i}=veh_object2(1,L,W); % see 'help patch' for how to make patch graphics objects
       handle.veh(i) = patch('Vertices',veh{i}.vertices','Faces',veh{i}.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh(i,:),'EdgeColor',[0 0 0],'FaceAlpha',0.1);
       X_loc(i)=sum(veh{i}.vertices(1,:))/length(veh{i}.vertices(1,:));
       Y_loc(i)=sum(veh{i}.vertices(2,:))/length(veh{i}.vertices(2,:));
       handle.veh_text(i) = text(X_loc(i),Y_loc(i),strcat('veh ',num2str(i)),'FontSize',10,'HorizontalAlignment','center','VerticalAlignment','middle'); % default FontSize is 10

       % create tire vertices, then make 4 different graphics patch objects to move around independently
       tire{i}=veh_object2(2,L/5,W/5);
       handle.tire_rf(i) = patch('Vertices',tire{i}.vertices','Faces',tire{i}.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh(i,:),'EdgeColor',[1 1 1],'FaceAlpha',0.1);
       handle.tire_lf(i) = patch('Vertices',tire{i}.vertices','Faces',tire{i}.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh(i,:),'EdgeColor',[1 1 1],'FaceAlpha',0.1);
       handle.tire_rr(i) = patch('Vertices',tire{i}.vertices','Faces',tire{i}.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh(i,:),'EdgeColor',[1 1 1],'FaceAlpha',0.1);
       handle.tire_lr(i) = patch('Vertices',tire{i}.vertices','Faces',tire{i}.faces,'AmbientStrength',0.46,'FaceColor',cmap_veh(i,:),'EdgeColor',[1 1 1],'FaceAlpha',0.1);
   end
   
   % store all unmodified vertices and faces in AXES UserData (not figure window UserData)
   % so mdlUpdate() can orient and position in the global
   % note: these vertices are in the object's local coordinate frames
   pass_these_verts{1} = get(handle.veh ,'Vertices');
   pass_these_verts{2} = get(handle.tire_rf,'Vertices'); % just make one tire object persistent; right-front tire vertices are copied to lf, lr, rr in mdlUpdate()
   set(handle.axes(1),'userdata',pass_these_verts); % store veh object vertices for retrieval in mdlUpdate() below
    if save_anim_frames==1
        t = now; 
        DateString = datestr(t);
        DateString = erase(DateString," ");
        DateString = erase(DateString,":");
        DateString = erase(DateString,"-")
        fname = strcat('C:\Users\userPc\rovidip\rovi\savedanimations\anim_sequences',DateString)
        if not(isfolder(fname))         
            mkdir(fullfile(fname))
        end
     else
       fname = ''; 
    end
%
% Set Handles of graphics in FIGURE UserData
%   
   set(h_anim,'UserData',handle); % store axes, line, and veh objects just created for retrieval in mdlUpdate()

end