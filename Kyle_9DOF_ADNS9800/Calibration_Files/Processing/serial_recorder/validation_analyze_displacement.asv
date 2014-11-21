clear
clc
%% Load in Data based on total displacement
delta = 100;

for i = 1:5
    basename1 = 'DisplacementData_delta';
    basename1 = 'DisplacementData_deltareturn';
    basename1 = 'DisplacementData_deltarskin';
    basename2 = num2str(delta);
    basename3 = strcat('_',num2str(i));
    basename4 = '.txt';
    datafile = strcat(basename1,basename2,basename3,basename4);
    data=load(datafile);
    range(i) = length(data(:,1));
    time(1:range(i)-1,i) = data(2:end,1)-data(2,1);
    x_raw(1:range(i)-1,i) = data(2:end,2);
    x_int(1:range(i)-1,i) = cumsum(data(2:end,2));
    x_final(i) = x_int(range(i)-1,i);
    y_raw(1:range(i)-1,i) = data(2:end,3);
    y_int(1:range(i)-1,i) = cumsum(data(2:end,3));
    y_final(i) = y_int(range(i)-1,i);
    button(1:range(i)-1,i) = data(2:end,7);
    buttonMark = find(button(:,i)>500);
    buttonStart(i) = min(buttonMark);
    buttonStop(i) = max(buttonMark);
    clear buttonMark
    buttonRange = buttonStart(i):buttonStop(i);
    x_int_button(1:length(buttonRange),i) = cumsum(data(buttonRange,2));
    y_int_button(1:length(buttonRange),i) = cumsum(data(buttonRange,3));
    x_int_button_final(i) = x_int_button(length(buttonRange),i);
    y_int_button_final(i) = y_int_button(length(buttonRange),i);

end

%% Calculations using the button

average_distance = mean(y_int_button_final);
STD = std(y_int_button_final);
percent_error = abs(STD/average_distance*100);
%% Calculations not using the button

% average_distance = mean(y_final);
% STD = std(y_final);
% percent_error = abs(STD/average_distance)*100;
% % percent_error_return = abs(STD./min(y_int))*100;

%% Print Statements
fprintf(' For Delta of%4d mm \n',delta)
fprintf(' Mean displacement = %05.2f, Standard Dev = %06.3f \n', [average_distance, STD])
fprintf(' Percent Error = %.5f \n', percent_error)
% fprintf(' Percent Return Error = %.5f \n', percent_error_return)

