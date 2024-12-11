% example making .jpg or .emf (vector) images from a Matlab figure window

makeFigureImages=1; % (0/1) switch to turn on image file creation (be careful about over-writing images)


testName='myTest1'; % any text for the filename
folderName='anim_sequences'; % this folder must exist!

for i=1:100
    t=( 0:0.05:(2*pi) ) + i/100;
    y=sin(t);
    
    hf=figure(1); clf
    plot(t,y,'b.')
    xlabel('time (s)')
    ylabel('My Y_{label} (%)')
    grid on
    axis([0 10, -1.1 +1.1])
    
    % if makeFigureImages==1, printStr=sprintf('print(%i,''figures\\%s_figure_%i.emf'',''-dmeta'')',hf.Number,testName,hf.Number); eval(printStr), end
    if makeFigureImages==1, printStr=sprintf('print(%i,''%s\\%s_figure_%i.jpg'',''-djpeg'')',hf.Number,folderName,testName,i); eval(printStr), end
end





