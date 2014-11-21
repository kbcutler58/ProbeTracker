clear
clc

%%
displacement = 50;

for i=1:5
% Load in File
basename1 = 'DataRecording5_';
basename2 = num2str(displacement);
basename3 = 'mm_';
trial = num2str(i);
basename4 = '.txt';
datafile = strcat(basename1,basename2,basename3,trial,basename4);
dataname = strcat(basename1,basename2,basename3,trial);
% shortname = dataname
data.(dataname)=load(datafile);
% data.(dataname).time = data.(dataname)(:,1);
data2.(dataname).time =  data.(dataname)(:,1);
data2.(dataname).x_raw =  data.(dataname)(:,2);
data2.(dataname).y_raw =  data.(dataname)(:,3);
data2.(dataname).yaw =  data.(dataname)(:,4);
data2.(dataname).pitch =  data.(dataname)(:,5);
data2.(dataname).roll =  data.(dataname)(:,6);
data2.(dataname).button =  data.(dataname)(:,7);
data2.(dataname).x_int =  cumsum(data.(dataname)(:,2));
data2.(dataname).y_int =  cumsum(data.(dataname)(:,3));
data2.x_max(i) =  data2.(dataname).x_int(end);
data2.y_max(i) =  data2.(dataname).y_int(end);
end
data = data2;
clear data2


%% Separate Data

