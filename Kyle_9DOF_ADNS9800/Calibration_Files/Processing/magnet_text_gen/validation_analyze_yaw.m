% clear
% clc
% hold on
%%
delta = 40;

% Load in File
basename1 = 'DataRecording3_';
basename2 = num2str(delta);
basename3 = 'degrees.txt';
datafile = strcat(basename1,basename2,basename3);
data=load(datafile);

%% Separate Data

data40.time = data(:,1)-data(1,1);
data40.x_displacement= data(:,2);
data40.y_displacement=data(:,3);
data40.yaw = data(:,4);
data40.pitch = data(:,5);
data40.roll = data(:,6);
data40.button = data(:,7);
%%
% plot(time,yaw,time,button/50)

clc
clear
hold off
load yaw_data1
time10 = data10.time;
yaw10 = data10.yaw;
time20 = data20.time;
yaw20 = data20.yaw;
time30 = data30.time;
yaw30 = data30.yaw;
time40 = data40.time;
yaw40 = data40.yaw;

plot(time10,yaw10,time20,yaw20,time30,yaw30,time40,yaw40)
% figure;
% plot(data30.time,data30.yaw,data30.time,data30.button/50,data40.time,data40.yaw,data40.time,data40.button/50)
