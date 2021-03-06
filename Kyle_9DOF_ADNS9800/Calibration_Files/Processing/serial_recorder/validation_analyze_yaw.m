clear
clc
% hold on

tuning = 4;
% Load in File
for i = 1:5
    
% basename1 = 'OrientationData_raw';
basename1 = 'OrientationData_cal';
% basename1 = strcat('OrientationData_tun',num2str(tuning),'_');
basename2 = num2str(i);
basename3 = '.txt';
datafile = strcat(basename1,basename2,basename3);
data=load(datafile);
range(i) = length(data(:,1));
time(1:range(i),i) = data(:,1)-data(1,1);
yaw(1:range(i),i) = data(:,4);
button(1:range(i),i) = data(:,7);
buttonMark(i) = find(button(:,i)>500,1);
end

%%
 for i=1:5
    
close all
test = i;
hold on 
grid on
% title1 = 'Yaw Angle Response 20 Degree Turn Raw';
title1 = 'Yaw Angle Response 20 Degree Turn Calibrated';
% title1 = 'Yaw Angle Response 20 Degree Turn Tuned Calibrated';
title2 = strcat(' Test ', num2str(test));
% title2 = strcat(' Tuning ',num2str(tuning),' Test ', num2str(test));
title3 = strcat(title1,title2);
title(title3)
xlabel('Time in ms')
ylabel('Output Angle in Degrees')
plot(time(1:range(test),test),yaw(1:range(test),test),'LineWidth',5);%,yaw,time,button/50)
scatter(time(buttonMark(test),test),yaw(buttonMark(test),test),500,'s');
legend('Output','Start of rotation')

% h = gcf;
% saveas(h,strcat(title3,'.fig'))
% 
% h = gcf;
% saveas(h,strcat(title3,'.jpeg'))

 end