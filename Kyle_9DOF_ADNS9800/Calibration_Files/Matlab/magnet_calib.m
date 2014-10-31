% Import Data File
% Segment into XYZ
% Fit Ellipsoid
% Output xyz of center (a)
% Output ellipsoid formula constants (d)

clear 
clc
cd 'C:\Users\Kyle\Documents\GitHub\ProbeTracker\Kyle_9DOF_ADNS9800\Calibration_Files\Processing\magnet_text_gen'
cal_data = load('Magnet_Cal_Data3.txt');
x = cal_data(:,1);
y = cal_data(:,2);
z = cal_data(:,3);

scatter3(x,y,z)
axis equal

%%
[a,b,c] = ellipsoid_fit(cal_data);

close all
hold on
scatter3(x,y,z)
[x1,y1,z1] = ellipsoid(a(1),a(2),a(3),b(1),b(2),b(3));
surf(x1,y1,z1)
view(3)
%%
% Scaling Matrix
scale = inv([b(1) 0 0; 0 b(2) 0; 0 0 b(3)]) * min(b);
map = c';
invmap = c;
comp = invmap * scale * map;

% Scaled Points
S = [x - a(1), y - a(2), z - a(3)]';
S = comp * S;

fprintf(' To use extended calibration add the next two lines \n')
fprintf('const float magn_ellipsoid_center[3] = {%.6g, %.6g, %.6g};\n', a);
fprintf('const float magn_ellipsoid_transform[3][3] = {{%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}, {%.6g, %.6g, %.6g}};\n', comp);
fprintf('\n');
%% For use in hard iron calibration

fprintf(' To use regular calibration add the next few lines: \n')
fprintf('#define magnet_x_min ((float) %.4g) \n', min(x))
fprintf('#define magnet_y_min ((float) %.4g) \n', min(y))
fprintf('#define magnet_z_min ((float) %.4g) \n', min(z))
fprintf('#define magnet_x_max ((float) %.4g) \n', max(x))
fprintf('#define magnet_y_max ((float) %.4g) \n', max(y))
fprintf('#define magnet_z_max ((float) %.4g) \n', max(z))
