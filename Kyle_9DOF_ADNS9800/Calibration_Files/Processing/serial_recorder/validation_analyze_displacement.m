clear
clc
% hold on

%%
delta = 50;

for i = 1:1
basename1 = 'DisplacementData_delta';
basename2 = num2str(delta);
basename3 = strcat('_',num2str(i));
basename4 = '.txt';
datafile = strcat(basename1,basename2,basename3,basename4);
data=load(datafile);
range(i) = length(data(:,1));
time(1:range(i),i) = data(:,1)-data(1,1);
x_raw(1:range(i),i) = data(:,2);
y_raw(1:range(i),i) = data(:,3);
button(1:range(i),i) = data(:,7);
% buttonMark(i) = find(button(:,i)>500,1);
end

% %% Separate Data
% 
% time = data(:,1)-data(1,1);
% x_displacement= data(:,2);
% y_displacement=data(:,3);
% yaw = data(:,4);
% pitch = data(:,5);
% roll = data(:,6);
% button = data(:,7);
% %%
% plot(time,yaw,time,button/50)
