% Create animation of CW/Tracking plot
%{
    Input ( X position, Y Position, Plotted Data) 
        All doubles of same length
    Output [ Video ]
%}
%% Load Test Data
% clear
% clc
close all

% data1 = load('cw_fit_abs_optical_props.mat');
% data2 = load('xy path.mat');
X_Position = data_x;
Y_Position = data_y;
opticalData = peak_power(:,[1 3 5 7]);
% data1.mua_cwtracker4;
% clear data1 data2
% Which Column of properties/power to plot
DataSelection = 3;

%% Specify Video Information
% Video Name
videoName = 'CW_Tracking_Processed';
% Quality Value - Scales size of video
videoQuality = 50;
% Plot Options
plotTitle = '2D Path Visualization';
plotMarkerSize = 100;

% % Use time data for custom framerate
% time_range = (data_time(end)-data_time(1))/1000; %time_range in ms
% frame_rate = round(length(data_time)/time_range);
% round(length(data_time)/time_range);

% Set Frame rate to 30fps
frame_rate = 50;

%% Create VideoWriter
videoName = strcat(videoName,'.avi');
myVideo = VideoWriter(videoName);
myVideo.FrameRate = frame_rate;
myVideo.Quality = videoQuality;
open(myVideo);

for i=1:length(X_Position)
    % Plot X,Y,Data (Data based on DataSelection Variable)
    scatter(X_Position(1:i),Y_Position(1:i),plotMarkerSize,opticalData(1:i,DataSelection))
    axis equal
    colorbar
    title(plotTitle)
    frame = getframe(gcf);
    writeVideo(myVideo,frame);  
end

close(myVideo)
