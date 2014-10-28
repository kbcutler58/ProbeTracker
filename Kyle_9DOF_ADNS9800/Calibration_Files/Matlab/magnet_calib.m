clear 
clc
cd 'C:\Users\Kyle\Documents\Processing\magnet_text_gen'
cal_data = load('Magnet_Cal_Data3.txt');
x = cal_data(:,1);
y = cal_data(:,2);
z = cal_data(:,3);
%%
scatter3(x,y,z)
axis equal
% Import Data File
% Segment into XYZ
% Fit Ellipsoid
% Output xyz of center
% Output ellipsoid formula constants

[a,b,c,d] = ellipsoid_fit(cal_data);
%%
close all
hold on
scatter3(x,y,z)
[x1,y1,z1] = ellipsoid(a(1),a(2),a(3),b(1),b(2),b(3));
surf(x1,y1,z1)
view(3)