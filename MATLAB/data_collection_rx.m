clear; clc; close all force; format compact;

connection = serialport('/dev/ttyUSB0', 57600);

requested_time_sec = input("How long to sample? (seconds): ");
time_start = tic;

distance_cm_record = [];
distance_cm_avg_record = [];

actuation_record = [];
x_record = [];
time_sec_record = [];

connection.writeline('enable_tx');
while (toc(time_start) < requested_time_sec)
    message = connection.readline();

    % ---message is actuation data---
    if (contains(message, 'Actuation'))
        pieces = split(message);
        message = pieces(2);
        disp(message);
        actuation_record(end+1) = double(message); %#ok<*SAGROW>
    
    % ---message is x data---
    elseif (contains(message, 'X'))
        pieces = split(message);
        message = pieces(2);
        disp(message);
        x_record(end+1) = double(message);
    
    % ---message is time_ms data---
    elseif (contains(message, 'Time_ms'))
        pieces = split(message);
        message = pieces(2);
        disp(message);
        time_sec_record(end+1) = double(message)/1000;
    
    elseif (contains(message, 'Raw_cm'))
        pieces = split(message);
        message = pieces(2);
        disp(message);
        distance_cm_record(end+1) = double(message);
    
    elseif (contains(message, 'Avg_cm'))
        pieces = split(message);
        message = pieces(2);
        disp(message);
        distance_cm_avg_record(end+1) = double(message);
    end
end

connection.writeline('disable_tx');

last_index = min([numel(time_sec_record), ...
                  numel(x_record), ...
                  numel(actuation_record)]);

actuation_record = actuation_record(1:last_index);
x_record = x_record(1:last_index);
time_sec_record = time_sec_record(1:last_index);

time_shift_sec = time_sec_record(1);
for i = 1:last_index
    time_sec_record(i) = time_sec_record(i) - time_shift_sec;
end

clear connection;

data = iddata(x_record', actuation_record', 0.1);
data.InputName = {'Actuation Signal'};
data.OutputName = {'X Position'};

figure(1);
plot(data);
grid;

last_index = min([numel(time_sec_record), ...
                  numel(distance_cm_record), ...
                  numel(distance_cm_avg_record)]);

time_sec_record = time_sec_record(1:last_index);
distance_cm_record = distance_cm_record(1:last_index);
distance_cm_avg_record = distance_cm_avg_record(1:last_index);

figure(2);
plot(time_sec_record, [distance_cm_record; distance_cm_avg_record], 'x-');
grid;
title('Raw vs Averaged Sensor data')
legend('raw (cm)', 'avg (cm)');

plant = tfest(data,2);
