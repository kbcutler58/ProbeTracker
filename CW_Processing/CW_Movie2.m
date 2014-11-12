% Create animation of CW/Tracking plot
%%
%time_range = (data_time(end)-data_time(1))/1000;
%frame_rate = round(length(data_time)/time_range);

frame_rate = 30;%round(length(data_time)/time_range);

myVideo = VideoWriter('CW_Tracking_Process4.avi');
myVideo.FrameRate = frame_rate;
myVideo.Quality = 50;
open(myVideo);


for i=1:length(xy)
    % 11kHz visual
    scatter(xy(1:i,1),xy(1:i,2),100,mua_cwtracker4(1:i,1))
    axis equal
    colorbar
    title('2D Path Visualization 20khz peak')
    frame = getframe(gcf);
    writeVideo(myVideo,frame);
    
    % 20kHz visual
    
%     scatter(corrected_path_offset(1:i,1),corrected_path_offset(1:i,2),100,-peak_power(1:i,1))
%     axis equal
%     colorbar
%     title('2D Path Visualization 11khz peak')
%     frame = getframe(gcf);
%     writeVideo(myVideo,frame);
end

close(myVideo)
