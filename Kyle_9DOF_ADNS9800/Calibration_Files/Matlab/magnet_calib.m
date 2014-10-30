% Import Data File
% Segment into XYZ
% Fit Ellipsoid
% Output xyz of center (a)
% Output ellipsoid formula constants (d)

clear 
clc
cd 'C:\Users\BLI\Desktop\Downloads\ProbeTracker-master (7)\ProbeTracker-master\Kyle_9DOF_ADNS9800\Calibration_Files\Processing\magnet_text_gen'
cal_data = load('Magnet_Cal_Data6.txt');
x = cal_data(:,1);
y = cal_data(:,2);
z = cal_data(:,3);

scatter3(x,y,z)
axis equal

%%
[a,b,c,d] = ellipsoid_fit(cal_data);

close all
hold on
scatter3(x,y,z)
[x1,y1,z1] = ellipsoid(a(1),a(2),a(3),b(1),b(2),b(3));
surf(x1,y1,z1)
view(3)
%%
S = [x - a(1), y - a(2), z - a(3)]';
scale = inv([b(1) 0 0; 0 b(2) 0; 0 0 b(3)]) * min(b);
map = c';
invmap = c;
comp = invmap * scale * map;
S = comp * S;

fprintf('const float magn_ellipsoid_center[3] = {%.6g, %.6g, %.6g};\n', a);
fprintf('const float magn_ellipsoid_transform[3][3] = {{%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}};\n', comp);

%% For use in hard iron calibration
maxMag(1) = max(x);
maxMag(2) = max(y);
maxMag(3) = max(z);
minMag(1) = min(x);
minMag(2) = min(y);
minMag(3) = min(z);