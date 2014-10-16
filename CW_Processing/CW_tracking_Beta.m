clear
clc
close all

load Oct_track

label.time = 1;
label.x = 2;
label.y = 3;
label.yaw = 4;
label.pitch = 5;
label.roll = 6;
label.button = 7;
label.squal = 8;
% label.random = 9;
label.waveform1 = 10:60;
label.waveform2 = 61:110;

data_time = data(:,label.time);
data_x = data(:,label.x);
data_y = data(:,label.y);
data_yaw = deg2rad(data(:,label.yaw));
data_pitch = deg2rad(data(:,label.pitch));
data_roll = deg2rad(data(:,label.roll));
data_squal = data(:,label.squal);
% data_random = data(:,label.random);
data_button = data(:,label.button);
data_waveform1 = data(:,label.waveform1);
data_waveform2 = data(:,label.waveform2);

clear data outputFile pathName label

peak_power = [peaks(:,1),peaks(:,3)];

rho = 1500;
data_yaw_offset = data_yaw; %- data_yaw(1); % offset angle to zero
x_location = 0; % initial x location
y_location = 0; % initial y location
x_location_offset = 0; % initial x with no offset
y_location_offset = 0; %initial y with no offset
x_location_offset2 = rho;
y_location_offset2 = 0;
corrected_path = zeros(size(data_x,1),2);
corrected_path_offset = zeros(size(data_x,1),2);

% % No angle offset integration
% for i=1:length(data_x)
%     % Calculate rotation matrix
%     correction_factor = [ cos(data_yaw(i)) -sin(data_yaw(i)); sin(data_yaw(i)) cos(data_yaw(i))];
%     % Matrix for raw x and y
%     path_matrix = [data_x(i),data_y(i)];
%     % Multiply x,y matrix and rotation matrix
%     path_temp = path_matrix*correction_factor;
%     % Add temp path to previous location
%     corrected_path(i,:) = [x_location + path_temp(1) , y_location + path_temp(2)];
%     % Update Location of x and y
%     x_location = corrected_path(i,1);
%     y_location = corrected_path(i,2);
% end
% 
% % Angle offset integration
% for i=1:length(data_x)
%     % Calculate rotation matrix 
%     correction_factor = [ cos(data_yaw_offset(i)) -sin(data_yaw_offset(i)); sin(data_yaw_offset(i)) cos(data_yaw_offset(i))];
%     % Matrix for raw x and y
%     path_matrix = [data_x(i);data_y(i)];
%     % Multiply x,y matrix and rotation matrix
% %     path_temp = path_matrix*correction_factor;
%     path_temp = correction_factor*path_matrix;
%     % Add temp path to previous location
%     corrected_path_offset(i,:) = [x_location_offset + path_temp(1) , y_location_offset + path_temp(2)];
%     % Update Location of x and y 
%     x_location_offset = corrected_path_offset(i,1);
%     y_location_offset = corrected_path_offset(i,2);
% end


% Angle offset integration
for i=1:length(data_x)
    % Calculate rotation matrix 
    correction_factor = [ cos(data_yaw_offset(i)) -sin(data_yaw_offset(i)); sin(data_yaw_offset(i)) cos(data_yaw_offset(i))];
    % Matrix for raw x and y
    path_matrix = [data_x(i);data_y(i)];
    % Multiply x,y matrix and rotation matrix
%     path_temp = path_matrix*correction_factor;
    path_temp = correction_factor*path_matrix;
    path_temp2 = correction_factor*path_matrix + [rho*cos(data_yaw_offset(i));rho*sin(data_yaw_offset(i))];
%     path_temp2 = correction_factor*path_matrix
    % Add temp path to previous location
    corrected_path_offset(i,:) = [x_location_offset + path_temp(1) , y_location_offset + path_temp(2)];
    corrected_path_offset2(i,:) = [x_location_offset + path_temp2(1) , y_location_offset + path_temp2(2)];
    % Update Location of x and y 
    x_location_offset = corrected_path_offset(i,1);
    y_location_offset = corrected_path_offset(i,2);
    x_location_offset2 = corrected_path_offset2(i,1);
    y_location_offset2 = corrected_path_offset2(i,2);
end

%% Plot Corrected Path
markerSize = 4000;
% axis([xmin xmax ymin ymax]
newaxis = ([-10000 10000 -10000 10000]);

% % Plot x y raw path
% close all
% scatter(cumsum(data_x),cumsum(data_y),150,-peak_power(:,3)); colorbar,
% % axis(newaxis); 
% axis equal
% title('2D Raw X and Y Output');

close all
% Plot x y angle corrected path with offset (11khz peak)
figure;
scatter(corrected_path_offset(:,1),corrected_path_offset(:,2),'x');
% ,-peak_power(:,3)), 
colorbar
title('2D Path Angle Corrected Offset');
% axis(newaxis); 
axis equal

hold on

scatter(corrected_path_offset2(:,1),corrected_path_offset2(:,2),markerSize,-peak_power(:,2)), colorbar
title('2D Path Angle Corrected Offset + Measurment Offset');
% axis(newaxis); 
axis equal