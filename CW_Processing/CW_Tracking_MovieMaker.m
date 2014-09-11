% Create animation of CW/Tracking plot
time_range = (data_time(end)-data_time(1))/1000;
frame_rate = round(length(data_time)/time_range);


myVideo = VideoWriter('CW_Tracking_Process2.avi');
myVideo.FrameRate = frame_rate;
myVideo.Quality = 50;
open(myVideo);


for i=1:length(data_x)
    % 11kHz visual
    scatter(corrected_path_offset(1:i,1),corrected_path_offset(1:i,2),100,-peak_power(1:i,3))
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
