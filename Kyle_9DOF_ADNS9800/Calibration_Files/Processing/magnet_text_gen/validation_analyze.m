clear
clc
hold on
%%
delta = 10;

% Load in File
basename1 = 'DataRecording3_';
basename2 = num2str(delta);
basename3 = 'degrees.txt';
datafile = strcat(basename1,basename2,basename3);
data=load(datafile);

%% Separate Data

time = data(:,1)-data(1,1);
x_displacement= data(:,2);
y_displacement=data(:,3);
yaw = data(:,4);
pitch = data(:,5);
roll = data(:,6);
button = data(:,7);
%%
plot(time,yaw,time,button/50)
